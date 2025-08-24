from flask import Flask, jsonify, render_template, request
import RPi.GPIO as GPIO
from statistics import mean
import bisect
import random
import time
from datetime import datetime
import numpy as np
import pandas as pd
import os
from collections import deque
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.', 'lib')))
from adxl355 import ADXL355

CONFIG_FILE = "config.json"

import gpiod

# ==== Configuración GPIO con gpiod ====
PIN_INT = 27  # Número BCM, no físico

chip = gpiod.Chip('gpiochip0')
led_line = chip.get_line(PIN_INT)
led_line.request(consumer="mi-app", type=gpiod.LINE_REQ_DIR_OUT)

# Abrir el chip GPIO (normalmente "gpiochip0")
chip = gpiod.Chip("/dev/gpiochip0")

# Configuración del pin (ej: 22)
config = gpiod.LineSettings()
config.direction = gpiod.LineDirection.INPUT
config.edge = gpiod.LineEdge.FALLING  # o RISING/BOTH

# Pedir el pin
request = chip.request_lines(
    consumer="mi-app",
    config={22: config}
)

print("Esperando interrupción en GPIO22...")
while True:
    event = request.read_edge_event()
    print("Evento:", event)


class Sensor:
    last_correct_axes = (0,0,0)
    samples_fifo = 1

    # ==== Configuración ====
    HISTORICO_SEGUNDOS = 60
    FRECUENCIA_MAX_HZ = 4000
    MAX_MUESTRAS = HISTORICO_SEGUNDOS * FRECUENCIA_MAX_HZ  # 1 minuto a 4 kHz

    # Deque para histórico
    data_deque = deque(maxlen=MAX_MUESTRAS)
    data_deque.append({'x':0,'y':0,'z':0,'temp':0,'timestamp':0})

    """Gestiona la comunicación con el sensor ADXL355 o la simulación de datos."""
    def __init__(self, simulate=False):
        self.sensor_detectado = False
        self.simulate = simulate
        if not self.simulate:
            try:
                self.adxl355 = ADXL355()
                self.sensor_detectado = True
                print("Sensor ADXL355 detectado correctamente.")
            except Exception as e:
                print(f"Sensor ADXL355 no detectado: {e}. Pasando a modo simulación.")
                self.simulate = True
        else:
            print("Modo simulación activado.")


    def leer_fifo(self, channel=None):
        if self.simulate:
            data = self.leer_datos_simulados()
        else:
            data = self.adxl355.read_fifo(32)
        
        if data is not None:
            if not self.simulate:
                data["temp"] = self.adxl355.get_temperature()
            data["timestamp"] = time.time()
            self.data_deque.append(data)  # Guardar en histórico

    def leer_datos_simulados(self):
        accel_x, accel_y, accel_z = random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(-2, 2)
        temp = random.uniform(20, 30)
        return {'x': accel_x, 'y': accel_y, 'z': accel_z, 'temp': temp}
    
    def get_latest_data(self):
        if not self.data_deque:
            return {'x':0,'y':0,'z':0,'temp':0,'timestamp':0}
        return self.data_deque[-1]

    def datos_entre_tiempos(self, t_inicio, t_fin):
        """
        Devuelve una lista con los datos del deque entre dos timestamps usando búsqueda binaria.
        Se asume que los datos están ordenados por 'timestamp'.
        """
        # Convertir deque a lista para usar bisect
        data_list = list(self.data_deque)

        # Extraer lista de timestamps
        timestamps = [item["timestamp"] for item in data_list]

        # Encontrar posición inicial y final usando bisect
        idx_inicio = bisect.bisect_left(timestamps, t_inicio)
        idx_fin = bisect.bisect_right(timestamps, t_fin)

        # Retornar solo el rango de datos
        return deque(data_list[idx_inicio:idx_fin])

app = Flask(__name__)
sensor = Sensor(simulate=True) # For now, let's simulate data for development
recording = False
recording_start_time = 0

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def data():
    return jsonify(sensor.get_latest_data())

@app.route('/record', methods=['POST'])
def record():
    global recording, recording_start_time
    data = request.get_json()
    recording = data.get('recording', False)
    if recording:
        recording_start_time = time.time()
    else:
        if recording_start_time > 0:
            grabar_archivo(recording_start_time, time.time())
            recording_start_time = 0

    return jsonify({'recording': recording})

def grabar_archivo(t_inicio, t_fin):
    print(
        "Inicio: ", time.strftime("%y%m%d-%H%M%S", time.localtime(t_inicio)),
        "Fin: ", time.strftime("%y%m%d-%H%M%S", time.localtime(t_fin))
    )   
    base_name = "datos_web"
    timestamp_str = datetime.now().strftime("%y%m%d-%H%M%S")
    datos = sensor.datos_entre_tiempos(t_inicio, t_fin)
    
    current_log_filename = os.path.join("data", f"{base_name}_{timestamp_str}.txt")
    with open(current_log_filename, "w") as f:
        for d in datos:
            ts_str = datetime.fromtimestamp(d["timestamp"]).isoformat()
            f.write(f"{ts_str},{d['x']:.4f},{d['y']:.4f},{d['z']:.4f},{d['temp']:.2f}\n")
    print(f"Archivo guardado en {current_log_filename}")

def sensor_loop():
    while True:
        sensor.leer_fifo()
        time.sleep(0.1) # Adjust as needed

if __name__ == '__main__':
    import threading
    sensor_thread = threading.Thread(target=sensor_loop)
    sensor_thread.daemon = True
    sensor_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=True)