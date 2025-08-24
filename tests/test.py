import RPi.GPIO as GPIO
from statistics import mean
import bisect
import tkinter as tk
from tkinter import ttk, filedialog
import random
import time
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import json
import os
from collections import deque
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.', 'lib')))
from adxl355 import ADXL355

CONFIG_FILE = "config.json"

# ==== Configuración GPIO ====
PIN_INT = 22  # Pin físico
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN_INT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up para active-low

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
    def __init__(self):
        self.sensor_detectado = False
        try:
            self.adxl355 = ADXL355()
            self.sensor_detectado = True
            print("Sensor ADXL355 detectado correctamente.")
        except Exception as e:
            print(f"Sensor ADXL355 no detectado: {e}")

    def leer_fifo(self, channel=None):
        data = self.adxl355.read_fifo(32)
        # data = self.adxl355.get_axes_norm(self.adxl355.get_axes())
        # print(data)
        if data is not None:
            data["temp"] = self.adxl355.get_temperature()
            data["timestamp"] = time.time()
            self.data_deque.append(data)  # Guardar en histórico
            if data["z"] > -1:
                print(data)


    def leer_datos_simulados(self, event_state):
        if event_state == 'RECORDING' or (random.random() < 0.01 and event_state == 'IDLE'):
            accel_x, accel_y, accel_z = random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(-2, 2)
        else:
            accel_x, accel_y, accel_z = random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(0.95, 1.05)
        temp = random.uniform(20, 30)
        return accel_x, accel_y, accel_z, temp
    
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

class AcelerometroApp:
    
    def __init__(self, root):
        self.root = root
        self.root.title("Visor de Acelerómetro")
        self.root.geometry("650x800")

        self.sensor = Sensor()
        self.limpiar_datos_grafico()
        self.crear_widgets()

        self.after_job = self.root.after(1, self.actualizar_datos)

    def limpiar_datos_grafico(self):
        self.datos_grafico = {'tiempos': [], 'x': [], 'y': [], 'z': []}
        self.start_time = time.time()

    def crear_widgets(self):
        main_tab_control = ttk.Notebook(self.root)
        self.monitor_tab = ttk.Frame(main_tab_control)
        self.config_tab = ttk.Frame(main_tab_control)
        self.analyzer_tab = ttk.Frame(main_tab_control)
        main_tab_control.add(self.monitor_tab, text='Monitor')
        main_tab_control.pack(expand=1, fill="both", padx=5, pady=(5, 10))

        self.crear_pestana_monitor()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def crear_pestana_monitor(self):
        top_frame = tk.Frame(self.monitor_tab); top_frame.pack(pady=5, padx=10, fill=tk.X)
        info_frame = tk.Frame(top_frame); info_frame.pack(side=tk.LEFT)
        
        self.var_x, self.var_y, self.var_z, self.var_temp = tk.StringVar(), tk.StringVar(), tk.StringVar(), tk.StringVar()
        labels_data = [("X:", self.var_x), ("Y:", self.var_y), ("Z:", self.var_z), ("Temp:", self.var_temp)]
        for i, (txt, var) in enumerate(labels_data):
            tk.Label(info_frame, text=txt, font=('Arial', 12, 'bold')).grid(row=0, column=i*2, sticky='w', padx=(0, 2))
            tk.Label(info_frame, textvariable=var, width=8, font=('Arial', 12)).grid(row=0, column=i*2 + 1, sticky='w', padx=(0, 15))

        fig_vector = plt.figure(figsize=(1.0, 1.0), dpi=80)
        fig_vector.patch.set_facecolor('lightgray')
        fig_vector.patch.set_alpha(0.0)
        self.ax_vector = fig_vector.add_subplot(111, projection='3d')
        self.ax_vector.patch.set_alpha(0.0)
        self.ax_vector.axis('off')
        self.canvas_vector = FigureCanvasTkAgg(fig_vector, master=top_frame); self.canvas_vector.get_tk_widget().pack(side=tk.RIGHT)

        fig_principal, self.ax_principal = plt.subplots(figsize=(6, 3), dpi=100)
        self.linea_x, = self.ax_principal.plot([], [], label='X', color='r')
        self.linea_y, = self.ax_principal.plot([], [], label='Y', color='g')
        self.linea_z, = self.ax_principal.plot([], [], label='Z', color='b')
        self.ax_principal.legend(); self.ax_principal.set_ylabel("Aceleración (g)")
        canvas_principal = FigureCanvasTkAgg(fig_principal, master=self.monitor_tab); canvas_principal.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10)
        self.ani = animation.FuncAnimation(fig_principal, self.animar_grafico_principal, interval=1, blit=False, save_count=100)

    def leer_datos_sensor(self):
        return self.sensor.data_deque[-1]

    def actualizar_datos(self):

        data = self.leer_datos_sensor()
        x_raw = data["x"]
        y_raw = data["y"]
        z_raw = data["z"]
        temp = data["temp"]
        x, y, z = x_raw, y_raw, z_raw

        self.actualizar_gui_datos(x, y, z, temp, x_raw, y_raw, z_raw)
        self.actualizar_datos_grafico(x, y, z)

        self.after_job = self.root.after(1, self.actualizar_datos)
        

    def actualizar_gui_datos(self, x, y, z, temp, x_raw, y_raw, z_raw):
        self.var_x.set(f"{x:+.4f} g"); 
        self.var_y.set(f"{y:+.4f} g"); 
        self.var_z.set(f"{z:+.4f} g"); 
        self.var_temp.set(f"{temp:.1f} °C")
        # self.var_raw_x.set(f"{x_raw:+.4f}"); 
        # self.var_raw_y.set(f"{y_raw:+.4f}"); 
        # self.var_raw_z.set(f"{z_raw:+.4f}")
        # self.var_cal_x.set(f"{x:+.4f}"); 
        # self.var_cal_y.set(f"{y:+.4f}"); 
        # self.var_cal_z.set(f"{z:+.4f}")

    def actualizar_datos_grafico(self, x, y, z):
        try: 
            time_window = float(20)
        except ValueError: time_window = 10.0
        current_time = time.time() - self.start_time
        self.datos_grafico['tiempos'].append(current_time); 
        self.datos_grafico['x'].append(x); 
        self.datos_grafico['y'].append(y); 
        self.datos_grafico['z'].append(z)
        while self.datos_grafico['tiempos'] and self.datos_grafico['tiempos'][0] < current_time - time_window:
            [self.datos_grafico[k].pop(0) for k in self.datos_grafico]

    def animar_grafico_principal(self, i):
        if self.datos_grafico['tiempos']:
            self.linea_x.set_data(self.datos_grafico['tiempos'], self.datos_grafico['x'])
            self.linea_y.set_data(self.datos_grafico['tiempos'], self.datos_grafico['y'])
            self.linea_z.set_data(self.datos_grafico['tiempos'], self.datos_grafico['z'])
            self.ax_principal.set_xlim(self.datos_grafico['tiempos'][0], self.datos_grafico['tiempos'][-1])
            all_vals = self.datos_grafico['x'] + self.datos_grafico['y'] + self.datos_grafico['z']
            if all_vals:
                self.ax_principal.set_ylim(min(all_vals) - 0.1, max(all_vals) + 0.1)
        return self.linea_x, self.linea_y, self.linea_z


    def on_closing(self):
        plt.close('all'); 
        self.root.quit(); 
        self.root.destroy()
        GPIO.cleanup()

first=0
if __name__ == "__main__":
    root = tk.Tk()
    app = AcelerometroApp(root)
    GPIO.add_event_detect(PIN_INT, GPIO.FALLING, callback=app.sensor.leer_fifo)
    if GPIO.input(PIN_INT) == 0 and first==0:
        first=1
        app.sensor.leer_fifo()
    root.mainloop()
