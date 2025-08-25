from flask import Flask, jsonify, render_template, request
import threading
import time
import json
from datetime import datetime
import os
import bisect
from collections import deque

from adxl355 import ADXL355
from interrupt import GPIOInterrupt

app = Flask(__name__)

CONFIG_FILE = "config.json"

# --- Configuración y Estado ---
config = {
    "range": 1,
    "odr": 0,
    "fifo_samples": 32,
    "interrupt_map": 2, # Default: FIFO_FULL on INT1 (0b00000010)
    "offsets": {'x': 0.0, 'y': 0.0, 'z': 0.0},
    "filename": "datos_acelerometro",
    "stabilization": 0.0
}

def save_config():
    """Guarda la configuración actual en el archivo JSON."""
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=4)

def load_config():
    """Carga la configuración desde el archivo JSON al iniciar."""
    global config
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            try:
                loaded_config = json.load(f)
                config.update(loaded_config)
                print(f"Configuración cargada desde {CONFIG_FILE}.")
            except json.JSONDecodeError:
                print(f"Error al leer {CONFIG_FILE}. Usando y guardando configuración por defecto.")
                save_config()
    else:
        print(f"No se encontró {CONFIG_FILE}. Creando con valores por defecto.")
        save_config()

load_config()

sensor = None
irq = None
sensor_available = False
recording = False
recording_start_time = 0.0

try:
    sensor = ADXL355(measure_range=config['range'])
    sensor.set_odr(config['odr'])
    sensor.set_fifo_samples(config['fifo_samples'])
    sensor.set_interrupt_map(config['interrupt_map'])
    irq = GPIOInterrupt(pin=22)
    sensor_available = True
    print("Sensor ADXL355 detectado. Usando interrupciones GPIO y configuración cargada.")
except Exception as e:
    print(f"No se pudo inicializar el sensor o GPIO: {e}. La aplicación se ejecutará sin datos reales.")

def datos_entre_tiempos(buffer, t_inicio, t_fin):
    """Devuelve una lista con los datos del deque entre dos timestamps."""
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
            x_cal = d['x'] - config['offsets']['x']
            y_cal = d['y'] - config['offsets']['y']
            # Para Z, el offset lo acerca a 1.0g, no a 0.
            z_cal = d['z'] - config['offsets']['z']
            f.write(f"{ts_str},{x_cal:.6f},{y_cal:.6f},{z_cal:.6f},{d['temp']:.2f}\n")
    print(f"Archivo guardado en {file_path}")

def irq_handler():
    """
    Hilo que espera interrupciones del sensor y lee los datos del FIFO.
    """
    while True:
        # El timeout evita que se bloquee indefinidamente si algo va mal
        events = irq.wait_event(timeout=1.0) # Timeout en segundos
        if events:
            sensor.read_fifo_with_meta()

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/data", methods=["GET"])
def get_data():
    if sensor_available and sensor.buffer:
        with sensor.buffer_lock:
            latest_data = sensor.buffer[-1].copy()
        latest_data['x'] -= config['offsets']['x']
        latest_data['y'] -= config['offsets']['y']
        latest_data['z'] -= config['offsets']['z']
        return jsonify(latest_data)
    else:
        # Devuelve datos de ejemplo si el sensor no está disponible
        return jsonify({
            'x': 0.1 * (time.time() % 10), 'y': 0.2 * (time.time() % 5), 'z': 1.0, 'temp': 25.0, 
            'timestamp': time.time(), 'error': 'Sensor no disponible'
        })

@app.route('/record', methods=['POST'])
def record_toggle():
    global recording, recording_start_time, config
    data = request.get_json()
    action = data.get('recording', False)

    if action and not recording:
        recording = True
        recording_start_time = time.time()
        config['filename'] = data.get('filename', config['filename']).strip()
        if not config['filename']: # Evitar nombres vacíos
            config['filename'] = "datos_acelerometro"
        
        try:
            config['stabilization'] = float(data.get('stabilization', config['stabilization']))
        except (ValueError, TypeError):
            config['stabilization'] = 0.0
        
        save_config()
        print(f"Iniciando grabación (archivo: {config['filename']}, estabilización: {config['stabilization']}s)...")

    elif not action and recording:
        recording = False
        t_inicio_grabacion = recording_start_time + config['stabilization']
        t_fin_grabacion = time.time()
        
        if t_fin_grabacion > t_inicio_grabacion:
            grabar_archivo(t_inicio_grabacion, t_fin_grabacion, config['filename'])
        else:
            print("Grabación detenida antes de finalizar el tiempo de estabilización. No se guardó archivo.")

        recording_start_time = 0.0
        print("Grabación detenida.")

    return jsonify({'recording': recording, 'config': config})

@app.route('/zero', methods=['POST'])
def zero_sensor():
    global config
    if not sensor_available:
        return jsonify({'error': 'Sensor no disponible'}), 503

    SAMPLES_FOR_ZEROING = 100
    with sensor.buffer_lock:
        buffer_len = len(sensor.buffer)
        if buffer_len < SAMPLES_FOR_ZEROING:
            return jsonify({'success': False, 'message': f'No hay suficientes muestras ({buffer_len}/{SAMPLES_FOR_ZEROING}). Espere un momento.'}), 400
        samples_to_avg = list(sensor.buffer)[-SAMPLES_FOR_ZEROING:]

    # Calcula el promedio para cada eje
    config['offsets']['x'] = sum(s['x'] for s in samples_to_avg) / len(samples_to_avg)
    config['offsets']['y'] = sum(s['y'] for s in samples_to_avg) / len(samples_to_avg)
    # El offset de Z se calcula para que la lectura en reposo sea 1.0g
    config['offsets']['z'] = (sum(s['z'] for s in samples_to_avg) / len(samples_to_avg)) - 1.0
    
    save_config()
    print(f"Nuevos offsets calculados y guardados: {config['offsets']}")
    return jsonify({'success': True, 'message': 'Sensor puesto a cero.', 'offsets': config['offsets']})

@app.route('/offsets', methods=['POST'])
def set_offsets():
    global config
    if not sensor_available:
        return jsonify({'error': 'Sensor no disponible'}), 503
    
    data = request.get_json()
    try:
        new_offsets = {
            'x': float(data.get('x', config['offsets']['x'])),
            'y': float(data.get('y', config['offsets']['y'])),
            'z': float(data.get('z', config['offsets']['z'])),
        }
        config['offsets'] = new_offsets
        save_config()
        print(f"Offsets manuales guardados: {config['offsets']}")
        return jsonify({'success': True, 'message': 'Offsets manuales guardados.', 'offsets': config['offsets']})
    except (ValueError, TypeError, KeyError) as e:
        return jsonify({'success': False, 'message': f'Datos inválidos: {e}'}), 400

@app.route('/config', methods=['POST'])
def configure_sensor():
    if not sensor_available:
        return jsonify({'error': 'Sensor no disponible'}), 503

    new_config = request.get_json()
    try:
        if 'range' in new_config:
            config['range'] = int(new_config['range'])
            sensor.set_measure_range(config['range'])
        if 'odr' in new_config:
            config['odr'] = int(new_config['odr'])
            sensor.set_odr(config['odr'])
        if 'fifo_samples' in new_config:
            config['fifo_samples'] = int(new_config['fifo_samples'])
            sensor.set_fifo_samples(config['fifo_samples'])
        if 'interrupt_map' in new_config:
            # Value comes as a binary string from frontend
            interrupt_val = int(new_config['interrupt_map'], 2)
            if not 0 <= interrupt_val <= 255:
                raise ValueError("Interrupt map value must be an 8-bit integer.")
            config['interrupt_map'] = interrupt_val
            sensor.set_interrupt_map(config['interrupt_map'])
        
        with sensor.buffer_lock:
            sensor.buffer.clear()
        
        save_config()
        return jsonify({'success': True, 'message': 'Configuración aplicada.'})
    except (ValueError, TypeError) as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/status', methods=['GET'])
def get_status():
    status = {'recording': recording, 'sensor_available': sensor_available}
    # Add the config from file
    status.update({'config': config})
    # Also add the current value from the sensor register if available
    if sensor_available:
        status['config']['interrupt_map_current'] = sensor.get_interrupt_map()
    return jsonify(status)

if __name__ == "__main__":
    if sensor_available:
        threading.Thread(target=irq_handler, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False)
