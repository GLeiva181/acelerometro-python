from flask import Flask, jsonify, render_template, request
import threading
import time
from datetime import datetime
import os
import bisect
from collections import deque

from adxl355 import ADXL355
from interrupt import GPIOInterrupt

app = Flask(__name__)

# Instancias
try:
    sensor = ADXL355()
    irq = GPIOInterrupt(pin=25)
    sensor_available = True
    print("Sensor ADXL355 detectado. Usando interrupciones GPIO.")
except Exception as e:
    print(f"No se pudo inicializar el sensor o GPIO: {e}. La aplicación se ejecutará sin datos reales.")
    sensor = None
    irq = None
    sensor_available = False

# Estado de la aplicación
recording = False
recording_start_time = 0.0
# Nuevas variables de estado para configuración
sensor_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}
recording_filename = "datos_acelerometro"
stabilization_time = 0.0

def datos_entre_tiempos(buffer, t_inicio, t_fin):
    """
    Devuelve una lista con los datos del deque entre dos timestamps.
    """
    data_list = list(buffer)
    timestamps = [item["timestamp"] for item in data_list]

    idx_inicio = bisect.bisect_left(timestamps, t_inicio)
    idx_fin = bisect.bisect_right(timestamps, t_fin)

    return deque(data_list[idx_inicio:idx_fin])

def grabar_archivo(t_inicio, t_fin, base_name="datos_acelerometro"):
    if not os.path.exists("data"):
        os.makedirs("data")
    
    print(
        "Grabando datos. Inicio:", time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_inicio)),
        "Fin:", time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_fin))
    )
    timestamp_str = datetime.now().strftime("%y%m%d-%H%M%S")
    
    if not sensor_available or not sensor.buffer:
        print("No hay datos en el buffer para grabar.")
        return

    with sensor.buffer_lock:
        datos = datos_entre_tiempos(sensor.buffer, t_inicio, t_fin)
    
    if not datos:
        print("No hay datos para grabar en el intervalo de tiempo seleccionado.")
        return

    # Aplicar offsets a los datos guardados
    file_path = os.path.join("data", f"{base_name}_{timestamp_str}.csv")
    with open(file_path, "w", newline='') as f:
        f.write("timestamp,x,y,z,temp\n") # Header
        for d in datos:
            ts_str = datetime.fromtimestamp(d["timestamp"]).isoformat()
            x_cal = d['x'] - sensor_offsets['x']
            y_cal = d['y'] - sensor_offsets['y']
            # Para Z, el offset lo acerca a 1.0g, no a 0.
            z_cal = (d['z'] - sensor_offsets['z']) if 'z' in sensor_offsets else d['z']
            f.write(f"{ts_str},{x_cal:.6f},{y_cal:.6f},{z_cal:.6f},{d['temp']:.2f}\n")
    print(f"Archivo guardado en {file_path}")

def irq_handler():
    """
    Hilo que espera interrupciones del sensor y lee los datos del FIFO.
    """
    while True:
        # El timeout evita que se bloquee indefinidamente si algo va mal
        events = irq.wait_event(timeout=1.0)
        if events == [] or events:
            # Leemos el FIFO cada vez que hay una interrupción
            sensor.read_fifo_with_meta()

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/data", methods=["GET"])
def get_data():
    if sensor_available and sensor.buffer:
        latest_data = sensor.buffer[-1].copy()
        latest_data['x'] -= sensor_offsets['x']
        latest_data['y'] -= sensor_offsets['y']
        latest_data['z'] -= sensor_offsets['z']
        return jsonify(latest_data)
    else:
        # Devuelve datos de ejemplo si el sensor no está disponible
        return jsonify({
            'x': 0.1 * (time.time() % 10), 'y': 0.2 * (time.time() % 5), 'z': 1.0, 'temp': 25.0, 
            'timestamp': time.time(), 'error': 'Sensor no disponible'
        })

@app.route('/record', methods=['POST'])
def record_toggle():
    global recording, recording_start_time, recording_filename, stabilization_time
    data = request.get_json()
    action = data.get('recording', False)

    if action and not recording:
        recording = True
        recording_start_time = time.time()
        recording_filename = data.get('filename', 'datos_acelerometro').strip()
        if not recording_filename: # Evitar nombres vacíos
            recording_filename = "datos_acelerometro"
        
        try:
            stabilization_time = float(data.get('stabilization', 0))
        except (ValueError, TypeError):
            stabilization_time = 0.0

        print(f"Iniciando grabación (archivo: {recording_filename}, estabilización: {stabilization_time}s)...")

    elif not action and recording:
        recording = False
        t_inicio_grabacion = recording_start_time + stabilization_time
        t_fin_grabacion = time.time()
        
        if t_fin_grabacion > t_inicio_grabacion:
            grabar_archivo(t_inicio_grabacion, t_fin_grabacion, recording_filename)
        else:
            print("Grabación detenida antes de finalizar el tiempo de estabilización. No se guardó archivo.")

        recording_start_time = 0.0
        print("Grabación detenida.")

    return jsonify({'recording': recording})

@app.route('/zero', methods=['POST'])
def zero_sensor():
    global sensor_offsets
    if not sensor_available:
        return jsonify({'error': 'Sensor no disponible'}), 503

    SAMPLES_FOR_ZEROING = 100
    with sensor.buffer_lock:
        buffer_len = len(sensor.buffer)
        if buffer_len < SAMPLES_FOR_ZEROING:
            return jsonify({'success': False, 'message': f'No hay suficientes muestras ({buffer_len}/{SAMPLES_FOR_ZEROING}). Espere un momento.'}), 400
        samples_to_avg = list(sensor.buffer)[-SAMPLES_FOR_ZEROING:]

    # Calcula el promedio para cada eje
    sensor_offsets['x'] = sum(s['x'] for s in samples_to_avg) / len(samples_to_avg)
    sensor_offsets['y'] = sum(s['y'] for s in samples_to_avg) / len(samples_to_avg)
    # El offset de Z se calcula para que la lectura en reposo sea 1.0g
    sensor_offsets['z'] = (sum(s['z'] for s in samples_to_avg) / len(samples_to_avg)) - 1.0
    
    print(f"Nuevos offsets calculados: {sensor_offsets}")
    return jsonify({'success': True, 'message': 'Sensor puesto a cero.', 'offsets': sensor_offsets})

@app.route('/config', methods=['POST'])
def configure_sensor():
    if not sensor_available:
        return jsonify({'error': 'Sensor no disponible'}), 503

    config = request.get_json()
    try:
        if 'range' in config:
            sensor.set_measure_range(int(config['range']))
        if 'odr' in config:
            sensor.set_odr(int(config['odr']))
        if 'fifo_samples' in config:
            sensor.set_fifo_samples(int(config['fifo_samples']))
        
        with sensor.buffer_lock:
            sensor.buffer.clear()
        return jsonify({'success': True, 'message': 'Configuración aplicada.'})
    except (ValueError, TypeError) as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/status', methods=['GET'])
def get_status():
    return jsonify({
        'recording': recording,
        'sensor_available': sensor_available
    })

if __name__ == "__main__":
    if sensor_available:
        threading.Thread(target=irq_handler, daemon=True).start()
        print("sensor_available")
    app.run(host="0.0.0.0", port=5000, debug=False)
