from flask import Flask, jsonify, render_template, request
import threading
import time
import json
from datetime import datetime
import os
import bisect
from collections import deque
import pandas as pd

from adxl355 import ADXL355
from interrupt import GPIOInterrupt

app = Flask(__name__)

CONFIG_FILE = "config.json"

# --- Configuración y Estado ---
config = {
    "range": 1,
    "odr": 0,
    "fifo_samples": 32,
    "offsets": {'x': 0.0, 'y': 0.0, 'z': 0.0},
    "filename": "datos_acelerometro",
    "stabilization": 0.0,
    # Configuración de Detección de Eventos
    "auto_record": False,
    "cooldown": 5.0,
    "pre_record_time": 2.0,
    "time_window": 10.0,
    "umbral_mode": "Absoluto",  # 'Absoluto' o 'Relativo'
    "center_x": 0.0, "center_y": 0.0, "center_z": 1.0,
    "delta_x": 0.1, "delta_y": 0.1, "delta_z": 0.1,
    "min_x": -0.5, "max_x": 0.5,
    "min_y": -0.5, "max_y": 0.5,
    "min_z": 0.5, "max_z": 1.5,
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
last_event_time = 0.0
auto_recording_active = False

try:
    sensor = ADXL355(measure_range=config['range'])
    sensor.set_odr(config['odr'])
    sensor.set_fifo_samples(config['fifo_samples'])
    irq = GPIOInterrupt(pin=22)
    sensor_available = True
    print("Sensor ADXL355 detectado. Usando interrupciones GPIO y configuración cargada.")
except Exception as e:
    print(f"No se pudo inicializar el sensor o GPIO: {e}. La aplicación se ejecutará sin datos reales.")

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
            x_cal = d['x'] - config['offsets']['x']
            y_cal = d['y'] - config['offsets']['y']
            # Para Z, el offset lo acerca a 1.0g, no a 0.
            z_cal = d['z'] - config['offsets']['z']
            f.write(f"{ts_str},{x_cal:.6f},{y_cal:.6f},{z_cal:.6f},{d['temp']:.2f}\n")
    print(f"Archivo guardado en {file_path}")

def check_for_event(data):
    """Verifica si el punto de datos dispara un evento según la configuración."""
    # Los datos del buffer no tienen la calibración de offset aplicada
    x = data['x'] - config['offsets']['x']
    y = data['y'] - config['offsets']['y']
    z = data['z'] - config['offsets']['z']

    mode = config.get('umbral_mode', 'Absoluto')
    
    min_x =float(config.get('min_x', 0))
    min_y =float(config.get('min_y', 0))
    min_z =float(config.get('min_z', 0))
    max_x =float(config.get('max_x', 0))
    max_y =float(config.get('max_y', 0))
    max_z =float(config.get('max_z', 0))
    
    center_x = float(config.get('center_x', 0))
    center_y = float(config.get('center_y', 0))
    center_z = float(config.get('center_z', 0))
    
    delta_x = float(config.get('delta_x', 0.1))
    delta_y = float(config.get('delta_y', 0.1))
    delta_z = float(config.get('delta_z', 0.1))

    if mode == 'Absoluto':
        if not (min_x <= x <= max_x and
                min_y <= y <= max_y and
                min_z <= z <= max_z):
            return True
    elif mode == 'Relativo':
        if (abs(x - center_x) > delta_x or
            abs(y - center_y) > delta_y or
            abs(z - center_z) > delta_z):
            return True
            
    return False

def stop_auto_record_thread(start_time):
    """Espera a que pase la ventana de grabación y luego detiene la grabación."""
    global recording, auto_recording_active
    
    record_duration = float(config.get('time_window', 10.0))
    time.sleep(record_duration)

    if auto_recording_active and abs(recording_start_time - start_time) < 0.1:
        print(f"Finalizando grabación automática de {config.get('pre_record_time', 2.0) + record_duration}s.")
        
        t_inicio = recording_start_time
        t_fin = time.time()
        
        event_filename = f"{config.get('filename', 'datos')}_evento"
        grabar_archivo(t_inicio, t_fin, event_filename)

        recording = False
        auto_recording_active = False
    else:
        print("Grabación automática cancelada o ya detenida manualmente.")

def irq_handler():
    """
    Hilo que espera interrupciones, lee FIFO y busca eventos.
    """
    global last_event_time, recording, recording_start_time, auto_recording_active
    while True:
        events = irq.wait_event(timeout=1.0)
        if events == [] or events:
            with sensor.buffer_lock:
                start_idx = len(sensor.buffer)
                sensor.read_fifo_with_meta()
                new_data = list(sensor.buffer)[start_idx:]

            if not recording and config.get('auto_record', False) and (time.time() - last_event_time) > float(config.get('cooldown', 5.0)):
                for d in new_data:
                    if check_for_event(d):
                        print(f"¡Evento detectado! Iniciando grabación automática.")
                        last_event_time = time.time()
                        recording, auto_recording_active = True, True
                        recording_start_time = last_event_time - float(config.get('pre_record_time', 2.0))
                        threading.Thread(target=stop_auto_record_thread, args=(recording_start_time,)).start()
                        break

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
    global recording, recording_start_time, config, auto_recording_active
    data = request.get_json()
    action = data.get('recording', False)

    if action and not recording:
        recording = True
        auto_recording_active = False # Es una grabación manual
        recording_start_time = time.time()
        config['filename'] = data.get('filename', config['filename']).strip()
        if not config['filename']: # Evitar nombres vacíos
            config['filename'] = "datos_acelerometro"
        
        try:
            config['stabilization'] = float(data.get('stabilization', config['stabilization']))
        except (ValueError, TypeError):
            config['stabilization'] = 0.0
        
        save_config()
        print(f"Iniciando grabación manual (archivo: {config['filename']}, estabilización: {config['stabilization']}s)...")

    elif not action and recording:
        recording = False
        auto_recording_active = False # Detiene también la lógica de auto-grabación
        t_inicio_grabacion = recording_start_time + config['stabilization']
        t_fin_grabacion = time.time()
        
        if t_fin_grabacion > t_inicio_grabacion:
            grabar_archivo(t_inicio_grabacion, t_fin_grabacion, config['filename'])
        else:
            print("Grabación detenida antes de finalizar el tiempo de estabilización. No se guardó archivo.")

        recording_start_time = 0.0
        print("Grabación detenida manualmente.")

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

@app.route('/event_config', methods=['POST'])
def event_config():
    global config
    data = request.get_json()
    try:
        for key in ['auto_record', 'umbral_mode']:
            if key in data:
                if isinstance(data[key], bool):
                    config[key] = data[key]
                else:
                    config[key] = str(data[key])

        for key in ['cooldown', 'pre_record_time', 'time_window', 
                    'center_x', 'center_y', 'center_z', 'delta_x', 'delta_y', 'delta_z',
                    'min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z']:
            if key in data and data[key] is not None and data[key] != '':
                config[key] = float(data[key])
        
        save_config()
        print(f"Configuración de eventos actualizada.")
        return jsonify({'success': True, 'message': 'Configuración de eventos guardada.', 'config': config})
    except (ValueError, TypeError, KeyError) as e:
        return jsonify({'success': False, 'message': f'Datos inválidos: {e}'}), 400

@app.route('/files', methods=['GET'])
def list_files():
    data_dir = "data"
    if not os.path.exists(data_dir):
        return jsonify([])
    
    try:
        files = [f for f in os.listdir(data_dir) if f.endswith('.csv')]
        files.sort(key=lambda f: os.path.getmtime(os.path.join(data_dir, f)), reverse=True)
        return jsonify(files)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/file/<path:filename>', methods=['GET'])
def get_file_data(filename):
    data_dir = "data"
    # Basic security check to prevent directory traversal
    if '..' in filename or filename.startswith('/'):
        return jsonify({'error': 'Acceso no permitido'}), 400
        
    file_path = os.path.join(data_dir, filename)
    
    if not os.path.exists(file_path):
        return jsonify({'error': 'Archivo no encontrado'}), 404
        
    try:
        df = pd.read_csv(file_path)
        if not all(col in df.columns for col in ['timestamp', 'x', 'y', 'z']):
            return jsonify({'error': 'Formato de archivo CSV inválido'}), 400

        # Convert timestamp to milliseconds since epoch for JavaScript
        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df['timestamp_ms'] = (df['timestamp'] - pd.Timestamp("1970-01-01")) // pd.Timedelta('1ms')
        
        # Select only the columns we need and rename timestamp_ms to timestamp
        result_df = df[['timestamp_ms', 'x', 'y', 'z']].rename(columns={'timestamp_ms': 'timestamp'})
        
        return jsonify(result_df.to_dict(orient='records'))
    except Exception as e:
        return jsonify({'error': f'Error al procesar el archivo: {e}'}), 500

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
        
        with sensor.buffer_lock:
            sensor.buffer.clear()
        
        save_config()
        return jsonify({'success': True, 'message': 'Configuración aplicada.'})
    except (ValueError, TypeError) as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/status', methods=['GET'])
def get_status():
    status = {'recording': recording, 'sensor_available': sensor_available}
    status.update({'config': config})
    return jsonify(status)

if __name__ == "__main__":
    if sensor_available:
        threading.Thread(target=irq_handler, daemon=True).start()
        print("sensor_available")
    app.run(host="0.0.0.0", port=5000, debug=False)
