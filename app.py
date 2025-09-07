from flask import Flask, jsonify, render_template, request
import threading
import time
import json
from datetime import datetime
import os
import bisect
from collections import deque
import pandas as pd
import pigpio

from adxl355 import ADXL355
# from interrupt import GPIOInterrupt
# import RPi.GPIO as GPIO

app = Flask(__name__)

CONFIG_FILE = "config.json"

# --- Configuración y Estado ---
config = {
    "range": 1,
    "odr": 0,
    "hpf_corner": 0,
    "standby": False,
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
simulation_enabled = False
auto_recording_active = False
in_bounds_start_time = None

last_processed_index = 0
# ==== Configuración GPIO ====
# PIN_INT = 12
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(PIN_INT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up para active-low

import gpiod
from gpiod.line import Direction, Edge, Bias

PIN_NO = 12
chip = "/dev/gpiochip0"
# chip = gpiod.Chip('/dev/gpiochip0')
# led_line = chip.get_line(PIN_NO)
# led_line.request(consumer="myLed", type=gpiod.LINE_REQ_DIR_OUT)
gpio = gpiod.request_lines(
            chip,
            consumer="adxl355-int",
            config={
                PIN_NO: gpiod.LineSettings(
                    direction = Direction.INPUT,
                    edge_detection = Edge.FALLING,   # Activo en bajo → flanco descendente
                    bias = Bias.PULL_UP    # Pull-up interno
                )
            }
        )

try:
    sensor = ADXL355(measure_range=config['range'])
    sensor.set_filter(odr_value=config['odr'], hpf_corner=config['hpf_corner'])
    sensor.set_fifo_samples(config['fifo_samples'])
    sensor.set_power_ctl(standby=config['standby'], temp_off=False, drdy_off=False)
    
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

# ==== Función para leer FIFO ====
def leer_sensor(channel=None):
    sensor.read_fifo_with_meta()
    # print("inter")

def sensor_reading_thread():
    """
    Hilo que configura la interrupción y espera a que se llame el callback para leer el sensor.
    El callback 'leer_sensor' se ejecuta en un hilo separado por la librería RPi.GPIO.
    """
    # GPIO.add_event_detect(PIN_INT, GPIO.FALLING, callback=leer_sensor)
    
    # El hilo principal puede simplemente esperar o realizar otras tareas de bajo nivel.
    # En este caso, lo mantenemos vivo para que el programa no termine.
    while True:
        sensor.read_fifo_with_meta()
        #time.sleep(1) # Mantiene el hilo vivo sin consumir mucho CPU.

def event_detection_thread():
    """
    Hilo que procesa los datos del buffer para detectar eventos y gestionar la grabación automática.
    """
    global recording, recording_start_time, auto_recording_active, in_bounds_start_time, last_processed_index

    while True:
        time.sleep(0.1) # Revisa cada 100ms

        if not config.get('auto_record', False):
            continue

        with sensor.buffer_lock:
            buffer_len = len(sensor.buffer)
            if buffer_len <= last_processed_index:
                continue
            new_data = list(sensor.buffer)[last_processed_index:]
            last_processed_index = buffer_len

        for d in new_data:
            is_out_of_bounds = check_for_event(d)

            if is_out_of_bounds and not auto_recording_active:
                print(f"¡Evento detectado! Iniciando grabación automática.")
                recording = True
                auto_recording_active = True
                recording_start_time = d['timestamp'] - float(config.get('pre_record_time', 2.0))
                in_bounds_start_time = None

            if auto_recording_active:
                if not is_out_of_bounds:
                    if in_bounds_start_time is None:
                        in_bounds_start_time = d['timestamp']
                    
                    elapsed_in_bounds = d['timestamp'] - in_bounds_start_time
                    if elapsed_in_bounds >= float(config.get('cooldown', 5.0)):
                        print(f"Valores estables por {config['cooldown']}s. Finalizando grabación automática.")
                        event_filename = f"{config.get('filename', 'datos')}_evento"
                        grabar_archivo(recording_start_time, d['timestamp'], event_filename)
                        recording, auto_recording_active, in_bounds_start_time = False, False, None
                        break
                else:
                    in_bounds_start_time = None

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
    elif not sensor_available and simulation_enabled:
        # Devuelve datos de ejemplo si la simulación está activada
        return jsonify({
            'x': 0.1 * (time.time() % 10), 'y': 0.2 * (time.time() % 5), 'z': 1.0, 'temp': 25.0, 
            'timestamp': time.time()
        })
    else:
        # Sensor no disponible y simulación desactivada
        return jsonify({
            'x': 0, 'y': 0, 'z': 0, 'temp': 0,
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

@app.route('/toggle_simulation', methods=['POST'])
def toggle_simulation():
    global simulation_enabled
    if not sensor_available:
        simulation_enabled = not simulation_enabled
        message = 'Simulación activada.' if simulation_enabled else 'Simulación desactivada.'
        print(message)
        return jsonify({'success': True, 'simulation_enabled': simulation_enabled, 'message': message})
    else:
        return jsonify({'success': False, 'message': 'El sensor está conectado, no se puede simular.'}), 400

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
        if 'hpf_corner' in new_config:
            config['hpf_corner'] = int(new_config['hpf_corner'])
            sensor.set_hpf_corner(config['hpf_corner'])
        if 'standby' in new_config:
            config['standby'] = bool(new_config['standby'])
            sensor.set_standby(config['standby'])
        
        with sensor.buffer_lock:
            sensor.buffer.clear()
        
        save_config()
        return jsonify({'success': True, 'message': 'Configuración aplicada.'})
    except (ValueError, TypeError) as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/status', methods=['GET'])
def get_status():
    status = {
        'recording': recording,
        'sensor_available': sensor_available,
        'simulation_enabled': simulation_enabled
    }
    status.update({'config': config})
    return jsonify(status)

if __name__ == "__main__":
    if sensor_available:
        threading.Thread(target=sensor_reading_thread, daemon=True).start()
        threading.Thread(target=event_detection_thread, daemon=True).start()
        print("Hilos de lectura de sensor y detección de eventos iniciados.")
    app.run(host="0.0.0.0", port=5000, debug=False)
