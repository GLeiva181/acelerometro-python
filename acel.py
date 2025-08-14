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

class Sensor:
    """Gestiona la comunicación con el sensor ADXL355 o la simulación de datos."""
    def __init__(self):
        self.sensor_detectado = False
        try:
            self.adxl355 = ADXL355()
            self.sensor_detectado = True
            print("Sensor ADXL355 detectado correctamente.")
        except Exception as e:
            print(f"Sensor ADXL355 no detectado: {e}")

    def leer_datos_reales(self):
        if not self.sensor_detectado: return 0.0, 0.0, 0.0, 0.0
        # axes = self.adxl355.get_axes_norm()
        axes = self.adxl355.read_fifo(32)
        temp = self.adxl355.get_temperature()
        return axes['x'], axes['y'], axes['z'], temp

    def leer_datos_simulados(self, event_state):
        if event_state == 'RECORDING' or (random.random() < 0.01 and event_state == 'IDLE'):
            accel_x, accel_y, accel_z = random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(-2, 2)
        else:
            accel_x, accel_y, accel_z = random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(0.95, 1.05)
        temp = random.uniform(20, 30)
        return accel_x, accel_y, accel_z, temp

class AcelerometroApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Visor de Acelerómetro")
        self.root.geometry("650x800")

        self.sensor = Sensor()
        self.inicializar_variables_estado()
        self.crear_widgets()
        self.cargar_configuracion()

        self.after_job = self.root.after(1, self.actualizar_datos)

    def inicializar_variables_estado(self):
        self.grabando = False
        self.was_grabbing_last_frame = False
        self.event_state = 'IDLE'
        self.last_unstable_time = 0
        self.current_log_filename = None
        self.matriz_calibracion = np.eye(3)
        self.pre_event_buffer = deque(maxlen=40)
        self.var_promedio_habilitado = tk.BooleanVar(value=False)
        self.var_promedio_muestras = tk.StringVar(value='10')
        self.datos_promedio = deque(maxlen=10)

        # Nuevas variables para umbrales
        self.var_umbral_mode = tk.StringVar(value="Relativo")
        self.var_auto_center = tk.BooleanVar(value=False)
        # Relativo
        self.var_center_x = tk.StringVar(value='0.0')
        self.var_center_y = tk.StringVar(value='0.0')
        self.var_center_z = tk.StringVar(value='1.0')
        self.var_delta_x = tk.StringVar(value='0.5')
        self.var_delta_y = tk.StringVar(value='0.5')
        self.var_delta_z = tk.StringVar(value='0.5')
        # Absoluto
        self.var_min_x = tk.StringVar(value='-0.5')
        self.var_max_x = tk.StringVar(value='0.5')
        self.var_min_y = tk.StringVar(value='-0.5')
        self.var_max_y = tk.StringVar(value='0.5')
        self.var_min_z = tk.StringVar(value='0.5')
        self.var_max_z = tk.StringVar(value='1.5')

        # Historial para cálculo de centro automático
        self.history_x = deque(maxlen=40)
        self.history_y = deque(maxlen=40)
        self.history_z = deque(maxlen=40)

        self.limpiar_datos_grafico()

    def crear_widgets(self):
        main_tab_control = ttk.Notebook(self.root)
        self.monitor_tab = ttk.Frame(main_tab_control)
        self.config_tab = ttk.Frame(main_tab_control)
        self.analyzer_tab = ttk.Frame(main_tab_control)
        main_tab_control.add(self.monitor_tab, text='Monitor')
        main_tab_control.add(self.analyzer_tab, text='Analizador de Archivos')
        main_tab_control.add(self.config_tab, text='Configuración')
        main_tab_control.pack(expand=1, fill="both", padx=5, pady=(5, 10))

        self.crear_pestana_monitor()
        self.crear_pestana_analizador()
        self.crear_pestana_configuracion()

        self.error_banner = tk.Label(self.root, text="SENSOR NO DETECTADO. Active el modo simulación o revise la conexión.", bg="red", fg="white")
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

        self.boton_grabar = tk.Button(self.monitor_tab, text="▶ Iniciar Grabación", command=self.toggle_grabacion, bg="green", fg="white"); self.boton_grabar.pack(pady=5)

        fig_principal, self.ax_principal = plt.subplots(figsize=(6, 3), dpi=100)
        self.linea_x, = self.ax_principal.plot([], [], label='X', color='r')
        self.linea_y, = self.ax_principal.plot([], [], label='Y', color='g')
        self.linea_z, = self.ax_principal.plot([], [], label='Z', color='b')
        self.ax_principal.legend(); self.ax_principal.set_ylabel("Aceleración (g)")
        canvas_principal = FigureCanvasTkAgg(fig_principal, master=self.monitor_tab); canvas_principal.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10)
        self.ani = animation.FuncAnimation(fig_principal, self.animar_grafico_principal, interval=1, blit=False, save_count=100)

    def _actualizar_visibilidad_centro(self, *args):
        if self.var_auto_center.get():
            self.entry_center_x.config(state='disabled')
            self.entry_center_y.config(state='disabled')
            self.entry_center_z.config(state='disabled')
        else:
            self.entry_center_x.config(state='normal')
            self.entry_center_y.config(state='normal')
            self.entry_center_z.config(state='normal')

    def _actualizar_visibilidad_umbrales(self, *args):
        mode = self.var_umbral_mode.get()
        if mode == 'Relativo':
            self.relativo_frame.pack(fill=tk.X, pady=5)
            self.absoluto_frame.pack_forget()
            self._actualizar_visibilidad_centro()
        else: # Absoluto
            self.relativo_frame.pack_forget()
            self.absoluto_frame.pack(fill=tk.X, pady=5)

    def crear_pestana_configuracion(self):
        config_notebook = ttk.Notebook(self.config_tab)
        config_notebook.pack(expand=True, fill='both', padx=10, pady=10)

        cal_tab = ttk.Frame(config_notebook)
        rec_tab = ttk.Frame(config_notebook)
        vis_tab = ttk.Frame(config_notebook)

        config_notebook.add(cal_tab, text='Calibración')
        config_notebook.add(rec_tab, text='Grabación y Eventos')
        config_notebook.add(vis_tab, text='Visualización y Modo')

        # --- Pestaña de Calibración ---
        cal_frame = ttk.LabelFrame(cal_tab, text="Calibración de Ejes", padding=10); cal_frame.pack(fill=tk.X, pady=5, padx=5)
        realtime_frame = ttk.LabelFrame(cal_frame, text="Valores en Tiempo Real", padding=5); realtime_frame.pack(fill=tk.X, pady=5)
        self.var_raw_x, self.var_raw_y, self.var_raw_z = tk.StringVar(), tk.StringVar(), tk.StringVar()
        self.var_cal_x, self.var_cal_y, self.var_cal_z = tk.StringVar(), tk.StringVar(), tk.StringVar()
        tk.Label(realtime_frame, text="Raw X:").grid(row=0, column=0, sticky='w'); tk.Label(realtime_frame, textvariable=self.var_raw_x, width=10).grid(row=0, column=1)
        tk.Label(realtime_frame, text="Raw Y:").grid(row=1, column=0, sticky='w'); tk.Label(realtime_frame, textvariable=self.var_raw_y, width=10).grid(row=1, column=1)
        tk.Label(realtime_frame, text="Raw Z:").grid(row=2, column=0, sticky='w'); tk.Label(realtime_frame, textvariable=self.var_raw_z, width=10).grid(row=2, column=1)
        tk.Label(realtime_frame, text="Cal. X:").grid(row=0, column=2, sticky='w'); tk.Label(realtime_frame, textvariable=self.var_cal_x, width=10).grid(row=0, column=3)
        tk.Label(realtime_frame, text="Cal. Y:").grid(row=1, column=2, sticky='w'); tk.Label(realtime_frame, textvariable=self.var_cal_y, width=10).grid(row=1, column=3)
        tk.Label(realtime_frame, text="Cal. Z:").grid(row=2, column=2, sticky='w'); tk.Label(realtime_frame, textvariable=self.var_cal_z, width=10).grid(row=2, column=3)

        matrix_control_frame = ttk.LabelFrame(cal_frame, text="Matriz de Calibración", padding=5); matrix_control_frame.pack(fill=tk.X, pady=5)
        cal_btn_frame = tk.Frame(matrix_control_frame); cal_btn_frame.pack()
        tk.Button(cal_btn_frame, text="Calibrar (Poner a Cero)", command=self.calibrar_ejes).pack(side=tk.LEFT, padx=5)
        tk.Button(cal_btn_frame, text="Restablecer Matriz", command=self.reset_calibracion).pack(side=tk.LEFT, padx=5)
        matrix_frame = tk.Frame(matrix_control_frame); matrix_frame.pack(pady=5)
        self.cal_vars = [tk.StringVar() for _ in range(9)]
        for i in range(3): 
            for j in range(3): 
                tk.Entry(matrix_frame, textvariable=self.cal_vars[i*3+j], width=12).grid(row=i, column=j, padx=2, pady=2)
        tk.Button(matrix_control_frame, text="Aplicar Matriz Manual", command=self.aplicar_matriz_manual).pack()

        # --- Pestaña de Grabación y Eventos ---
        file_frame = ttk.LabelFrame(rec_tab, text="Archivo de Grabación", padding=10); file_frame.pack(fill=tk.X, pady=5, padx=5)
        self.var_nombre_archivo = tk.StringVar(value="datos"); tk.Label(file_frame, text="Nombre base:").pack(anchor='w'); tk.Entry(file_frame, textvariable=self.var_nombre_archivo, width=40).pack(anchor='w', fill=tk.X)

        event_frame = ttk.LabelFrame(rec_tab, text="Detección Automática de Eventos", padding=10); event_frame.pack(fill=tk.X, pady=5, padx=5)
        self.var_auto_record_enabled = tk.BooleanVar(value=False); tk.Checkbutton(event_frame, text="Activar detección de eventos", variable=self.var_auto_record_enabled).pack(anchor='w')
        
        params_frame = tk.Frame(event_frame); params_frame.pack(fill=tk.X, pady=5)
        self.var_cooldown = tk.StringVar(value='2.0'); self.var_pre_record_time = tk.StringVar(value='2.0')
        tk.Label(params_frame, text="Estabilidad (s):").grid(row=0, column=0, sticky='w'); tk.Entry(params_frame, textvariable=self.var_cooldown, width=8).grid(row=0, column=1, padx=5)
        tk.Label(params_frame, text="Tiempo Pre-grabación (s):").grid(row=1, column=0, sticky='w'); tk.Entry(params_frame, textvariable=self.var_pre_record_time, width=8).grid(row=1, column=1, padx=5)

        umbral_mode_frame = ttk.LabelFrame(event_frame, text="Modo de Umbral de Estabilidad", padding=10); umbral_mode_frame.pack(fill=tk.X, pady=(10, 5), padx=5)
        tk.Radiobutton(umbral_mode_frame, text="Relativo (Centro ± Delta)", variable=self.var_umbral_mode, value="Relativo", command=self._actualizar_visibilidad_umbrales).pack(anchor='w')
        tk.Radiobutton(umbral_mode_frame, text="Absoluto (Mínimo y Máximo)", variable=self.var_umbral_mode, value="Absoluto", command=self._actualizar_visibilidad_umbrales).pack(anchor='w')

        self.relativo_frame = tk.Frame(event_frame)
        tk.Checkbutton(self.relativo_frame, text="Calcular centro automáticamente", variable=self.var_auto_center, command=self._actualizar_visibilidad_centro).grid(row=0, column=0, columnspan=4, sticky='w')
        
        tk.Label(self.relativo_frame, text="Centro X:").grid(row=1, column=0, sticky='w', pady=2)
        self.entry_center_x = tk.Entry(self.relativo_frame, textvariable=self.var_center_x, width=8); 
        self.entry_center_x.grid(row=1, column=1, padx=5, pady=2)
        tk.Label(self.relativo_frame, text="Centro Y:").grid(row=2, column=0, sticky='w', pady=2)
        self.entry_center_y = tk.Entry(self.relativo_frame, textvariable=self.var_center_y, width=8); 
        self.entry_center_y.grid(row=2, column=1, padx=5, pady=2)
        tk.Label(self.relativo_frame, text="Centro Z:").grid(row=3, column=0, sticky='w', pady=2)
        self.entry_center_z = tk.Entry(self.relativo_frame, textvariable=self.var_center_z, width=8); 
        self.entry_center_z.grid(row=3, column=1, padx=5, pady=2)

        tk.Label(self.relativo_frame, text="Delta X:").grid(row=1, column=2, sticky='w', pady=2)
        tk.Entry(self.relativo_frame, textvariable=self.var_delta_x, width=8).grid(row=1, column=3, padx=5, pady=2)
        tk.Label(self.relativo_frame, text="Delta Y:").grid(row=2, column=2, sticky='w', pady=2)
        tk.Entry(self.relativo_frame, textvariable=self.var_delta_y, width=8).grid(row=2, column=3, padx=5, pady=2)
        tk.Label(self.relativo_frame, text="Delta Z:").grid(row=3, column=2, sticky='w', pady=2)
        tk.Entry(self.relativo_frame, textvariable=self.var_delta_z, width=8).grid(row=3, column=3, padx=5, pady=2)

        self.absoluto_frame = tk.Frame(event_frame)
        
        tk.Label(self.absoluto_frame, text="Mín X:").grid(row=1, column=0, sticky='w', pady=2)
        self.entry_min_x = tk.Entry(self.absoluto_frame, textvariable=self.var_min_x, width=8); 
        self.entry_min_x.grid(row=1, column=1, padx=5, pady=2)
        tk.Label(self.absoluto_frame, text="Mín Y:").grid(row=2, column=0, sticky='w', pady=2)
        self.entry_min_y = tk.Entry(self.absoluto_frame, textvariable=self.var_min_y, width=8); 
        self.entry_min_y.grid(row=2, column=1, padx=5, pady=2)
        tk.Label(self.absoluto_frame, text="Mín Z:").grid(row=3, column=0, sticky='w', pady=2)
        self.entry_min_z = tk.Entry(self.absoluto_frame, textvariable=self.var_min_z, width=8); 
        self.entry_min_z.grid(row=3, column=1, padx=5, pady=2)

        tk.Label(self.absoluto_frame, text="Máx X:").grid(row=1, column=2, sticky='w', pady=2)
        tk.Entry(self.absoluto_frame, textvariable=self.var_max_x, width=8).grid(row=1, column=3, padx=5, pady=2)
        tk.Label(self.absoluto_frame, text="Máx Y:").grid(row=2, column=2, sticky='w', pady=2)
        tk.Entry(self.absoluto_frame, textvariable=self.var_max_y, width=8).grid(row=2, column=3, padx=5, pady=2)
        tk.Label(self.absoluto_frame, text="Máx Z:").grid(row=3, column=2, sticky='w', pady=2)
        tk.Entry(self.absoluto_frame, textvariable=self.var_max_z, width=8).grid(row=3, column=3, padx=5, pady=2)

        self._actualizar_visibilidad_umbrales()

        # --- Pestaña de Visualización y Modo ---
        graph_frame = ttk.LabelFrame(vis_tab, text="Configuración del Gráfico", padding=10); graph_frame.pack(fill=tk.X, pady=5, padx=5)
        self.var_time_window = tk.StringVar(value='10'); tk.Label(graph_frame, text="Ventana de tiempo (s):").pack(side=tk.LEFT); tk.Entry(graph_frame, textvariable=self.var_time_window, width=8).pack(side=tk.LEFT)

        avg_frame = ttk.LabelFrame(vis_tab, text="Promediado de Señal", padding=10); avg_frame.pack(fill=tk.X, pady=5, padx=5)
        tk.Checkbutton(avg_frame, text="Habilitar promediado de señal", variable=self.var_promedio_habilitado).pack(anchor='w')
        avg_params_frame = tk.Frame(avg_frame); avg_params_frame.pack(fill=tk.X, pady=5)
        tk.Label(avg_params_frame, text="Número de muestras para promediar:").grid(row=0, column=0, sticky='w')
        tk.Entry(avg_params_frame, textvariable=self.var_promedio_muestras, width=8).grid(row=0, column=1, padx=5)

        mode_frame = ttk.LabelFrame(vis_tab, text="Modo de Operación", padding=10); mode_frame.pack(fill=tk.X, pady=5, padx=5)
        self.var_simulacion_activada = tk.BooleanVar(value=not self.sensor.sensor_detectado); tk.Checkbutton(mode_frame, text="Activar modo simulación", variable=self.var_simulacion_activada).pack(anchor='w')

    def crear_pestana_analizador(self):
        analyzer_frame = tk.Frame(self.analyzer_tab, padx=10, pady=10)
        analyzer_frame.pack(fill=tk.BOTH, expand=True)

        file_controls_frame = tk.Frame(analyzer_frame)
        file_controls_frame.pack(fill=tk.X, pady=5)
        tk.Button(file_controls_frame, text="Seleccionar Archivo", command=self.cargar_y_mostrar_archivo).pack(side=tk.LEFT)
        self.var_loaded_filename = tk.StringVar(value="Ningún archivo cargado")
        tk.Label(file_controls_frame, textvariable=self.var_loaded_filename, fg="gray").pack(side=tk.LEFT, padx=10)

        fig_analyzer, self.ax_analyzer = plt.subplots(figsize=(6, 4), dpi=100)
        self.ax_analyzer.set_title("Datos del Archivo Cargado")
        self.ax_analyzer.set_xlabel("Tiempo (s)")
        self.ax_analyzer.set_ylabel("Aceleración Raw (g)")
        self.ax_analyzer.grid(True)

        self.canvas_analyzer = FigureCanvasTkAgg(fig_analyzer, master=analyzer_frame)
        self.canvas_analyzer.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def cargar_y_mostrar_archivo(self):
        filepath = filedialog.askopenfilename(
            initialdir="data",
            title="Seleccionar archivo de datos",
            filetypes=(("Text files", "*.txt"), ("All files", "*.*"))
        )
        if not filepath: return

        try:
            df = pd.read_csv(filepath, header=None, names=['timestamp', 'x', 'y', 'z', 'temp'])
            df['timestamp'] = pd.to_datetime(df['timestamp'])
            df['seconds'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()

            self.ax_analyzer.clear()
            self.ax_analyzer.plot(df['seconds'], df['x'], label='X', color='r')
            self.ax_analyzer.plot(df['seconds'], df['y'], label='Y', color='g')
            self.ax_analyzer.plot(df['seconds'], df['z'], label='Z', color='b')
            self.ax_analyzer.legend()
            self.ax_analyzer.grid(True)
            self.ax_analyzer.set_title(f"Datos de: {filepath.split('/')[-1]}")
            self.ax_analyzer.set_xlabel("Tiempo (s)")
            self.ax_analyzer.set_ylabel("Aceleración Raw (g)")
            self.canvas_analyzer.draw()
            self.var_loaded_filename.set(filepath.split('/')[-1])
        except Exception as e:
            print(f"Error al cargar o procesar el archivo: {e}")
            self.var_loaded_filename.set("Error al cargar el archivo")

    def limpiar_datos_grafico(self):
        self.datos_grafico = {'tiempos': [], 'x': [], 'y': [], 'z': []}
        self.start_time = time.time()

    def leer_datos_sensor(self):
        if self.var_simulacion_activada.get():
            return self.sensor.leer_datos_simulados(self.event_state)
        else:
            return self.sensor.leer_datos_reales()

    def calibrar_ejes(self):
        x, y, z, _ = self.leer_datos_sensor()
        g_ref = np.array([x, y, z])
        self.matriz_calibracion = self.get_rotation_matrix_to_align(g_ref, np.array([0, 0, 1]))
        self.actualizar_matriz_calibracion_vars()
        self.limpiar_datos_grafico()

    def reset_calibracion(self):
        self.matriz_calibracion = np.eye(3)
        self.actualizar_matriz_calibracion_vars()
        self.limpiar_datos_grafico()

    def aplicar_matriz_manual(self):
        try:
            new_matrix_vals = [float(v.get()) for v in self.cal_vars]
            self.matriz_calibracion = np.array(new_matrix_vals).reshape((3, 3))
            self.limpiar_datos_grafico()
        except ValueError:
            print("Error: La matriz de calibración contiene valores no numéricos.")
            self.actualizar_matriz_calibracion_vars()

    def actualizar_matriz_calibracion_vars(self):
        for i in range(3):
            for j in range(3):
                self.cal_vars[i*3 + j].set(f"{self.matriz_calibracion[i, j]:.6f}")

    def toggle_grabacion(self):
        self.grabando = not self.grabando

    def get_rotation_matrix_to_align(self, vec_a, vec_b):
        vec_a_norm = np.linalg.norm(vec_a)
        if vec_a_norm == 0: return np.eye(3)
        vec_a = vec_a / vec_a_norm
        vec_b = vec_b / np.linalg.norm(vec_b)
        v = np.cross(vec_a, vec_b); s = np.linalg.norm(v); c = np.dot(vec_a, vec_b)
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        if s == 0: return np.eye(3)
        return np.eye(3) + vx + vx.dot(vx) * ((1 - c) / (s**2))

    def actualizar_datos(self):
        # --- Actualizar tamaños de buffers y deques según configuración ---
        try: pre_record_time = float(self.var_pre_record_time.get()); max_len_pre = int(pre_record_time / 0.05)
        except ValueError: max_len_pre = 40
        if self.pre_event_buffer.maxlen != max_len_pre: self.pre_event_buffer = deque(list(self.pre_event_buffer), maxlen=max_len_pre)

        try: num_muestras = int(self.var_promedio_muestras.get())
        except ValueError: num_muestras = 10
        if self.datos_promedio.maxlen != num_muestras: self.datos_promedio = deque(list(self.datos_promedio), maxlen=num_muestras)

        try: cooldown_duration = float(self.var_cooldown.get()); max_len_hist = int(cooldown_duration / 0.05)
        except ValueError: max_len_hist = 40
        if self.history_x.maxlen != max_len_hist: self.history_x = deque(list(self.history_x), maxlen=max_len_hist)
        if self.history_y.maxlen != max_len_hist: self.history_y = deque(list(self.history_y), maxlen=max_len_hist)
        if self.history_z.maxlen != max_len_hist: self.history_z = deque(list(self.history_z), maxlen=max_len_hist)

        # --- Lectura y promediado de datos del sensor ---
        if not self.sensor.sensor_detectado and not self.var_simulacion_activada.get(): self.error_banner.pack(side=tk.BOTTOM, fill=tk.X)
        else: self.error_banner.pack_forget()

        x_raw, y_raw, z_raw, temp = self.leer_datos_sensor()

        if self.var_promedio_habilitado.get():
            self.datos_promedio.append((x_raw, y_raw, z_raw, temp))
            if self.datos_promedio:
                datos = np.array(list(self.datos_promedio))
                x_raw, y_raw, z_raw, temp = np.mean(datos, axis=0)

        log_line = f"{datetime.now().isoformat()},{x_raw:.4f},{y_raw:.4f},{z_raw:.4f},{temp:.2f}\n"
        self.pre_event_buffer.append(log_line)

        # --- Calibración y actualización de historial ---
        g_calibrado = self.matriz_calibracion.dot(np.array([x_raw, y_raw, z_raw]))
        x, y, z = g_calibrado[0], g_calibrado[1], g_calibrado[2]

        self.history_x.append(x)
        self.history_y.append(y)
        self.history_z.append(z)

        # --- Gestión de lógica y actualización de GUI ---
        self.gestionar_grabacion_eventos(x, y, z, log_line)
        self.actualizar_gui_datos(x, y, z, temp, x_raw, y_raw, z_raw)
        self.actualizar_datos_grafico(x, y, z)
        self.actualizar_visualizacion_3d(x, y, z)

        self.after_job = self.root.after(1, self.actualizar_datos)

    def gestionar_grabacion_eventos(self, x, y, z, current_log_line):
        was_auto_recording = self.event_state == 'RECORDING'

        if self.var_auto_record_enabled.get() and (self.sensor.sensor_detectado or self.var_simulacion_activada.get()):
            try:
                cooldown_duration = float(self.var_cooldown.get())
                umbral_mode = self.var_umbral_mode.get()

                if umbral_mode == 'Relativo':
                    delta_x = float(self.var_delta_x.get())
                    delta_y = float(self.var_delta_y.get())
                    delta_z = float(self.var_delta_z.get())

                    if self.var_auto_center.get() and self.history_x:
                        center_x = (min(self.history_x) + max(self.history_x)) / 2
                        center_y = (min(self.history_y) + max(self.history_y)) / 2
                        center_z = (min(self.history_z) + max(self.history_z)) / 2
                    else:
                        center_x = float(self.var_center_x.get())
                        center_y = float(self.var_center_y.get())
                        center_z = float(self.var_center_z.get())
                    
                    is_stable = (
                        (center_x - delta_x) < x < (center_x + delta_x) and
                        (center_y - delta_y) < y < (center_y + delta_y) and
                        (center_z - delta_z) < z < (center_z + delta_z)
                    )
                else: # Absoluto
                    min_x = float(self.var_min_x.get()); max_x = float(self.var_max_x.get())
                    min_y = float(self.var_min_y.get()); max_y = float(self.var_max_y.get())
                    min_z = float(self.var_min_z.get()); max_z = float(self.var_max_z.get())
                    is_stable = (
                        min_x < x < max_x and
                        min_y < y < max_y and
                        min_z < z < max_z
                    )
            except (ValueError, IndexError):
                is_stable = True # Falla a un estado seguro (no grabar) si la config es inválida o los buffers están vacíos

            if not is_stable:
                self.last_unstable_time = time.time()

            if self.event_state == 'IDLE':
                if not is_stable:
                    self.event_state = 'RECORDING'
            elif self.event_state == 'RECORDING':
                if (time.time() - self.last_unstable_time) > cooldown_duration:
                    self.event_state = 'IDLE'
        else:
            self.event_state = 'IDLE'

        is_now_auto_recording = self.event_state == 'RECORDING'
        just_started_manual = self.grabando and not self.was_grabbing_last_frame
        just_started_auto = is_now_auto_recording and not was_auto_recording

        if just_started_manual or just_started_auto:
            base_name = self.var_nombre_archivo.get() or "datos"
            timestamp_str = datetime.now().strftime("%y%m%d-%H%M%S")
            self.current_log_filename = os.path.join("data", f"{base_name}_{timestamp_str}.txt")
            with open(self.current_log_filename, "w") as f:
                f.writelines(list(self.pre_event_buffer))
        elif self.grabando or is_now_auto_recording:
            if self.current_log_filename:
                with open(self.current_log_filename, "a") as f:
                    f.write(current_log_line)

        if self.grabando: self.boton_grabar.config(text="⏹ Detener Grabación", bg="red", state='normal')
        elif is_now_auto_recording: self.boton_grabar.config(text="Grabando Evento...", bg="orange", state='disabled')
        else: self.boton_grabar.config(text="▶ Iniciar Grabación", bg="green", state='normal')
        self.was_grabbing_last_frame = self.grabando

    def actualizar_gui_datos(self, x, y, z, temp, x_raw, y_raw, z_raw):
        self.var_x.set(f"{x:+.2f} g"); self.var_y.set(f"{y:+.2f} g"); self.var_z.set(f"{z:+.2f} g"); self.var_temp.set(f"{temp:.1f} °C")
        self.var_raw_x.set(f"{x_raw:+.4f}"); self.var_raw_y.set(f"{y_raw:+.4f}"); self.var_raw_z.set(f"{z_raw:+.4f}")
        self.var_cal_x.set(f"{x:+.4f}"); self.var_cal_y.set(f"{y:+.4f}"); self.var_cal_z.set(f"{z:+.4f}")

    def actualizar_datos_grafico(self, x, y, z):
        try: time_window = float(self.var_time_window.get())
        except ValueError: time_window = 10.0
        current_time = time.time() - self.start_time
        self.datos_grafico['tiempos'].append(current_time); self.datos_grafico['x'].append(x); self.datos_grafico['y'].append(y); self.datos_grafico['z'].append(z)
        while self.datos_grafico['tiempos'] and self.datos_grafico['tiempos'][0] < current_time - time_window:
            [self.datos_grafico[k].pop(0) for k in self.datos_grafico]

    def actualizar_visualizacion_3d(self, x, y, z):
        self.ax_vector.cla()
        self.ax_vector.patch.set_alpha(0.0)
        self.ax_vector.axis('off')
        r = self.get_rotation_matrix_to_align(np.array([x, y, z]), np.array([0,0,1]))
        for i, axis in enumerate([np.eye(3)[j] for j in range(3)]):
            rot_axis = r.dot(axis)
            self.ax_vector.quiver(0, 0, 0, rot_axis[0], rot_axis[1], rot_axis[2], color=['r','g','b'][i], arrow_length_ratio=0.2)
        self.ax_vector.set_xlim([-1, 1]); self.ax_vector.set_ylim([-1, 1]); self.ax_vector.set_zlim([-1, 1])
        self.canvas_vector.draw()

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

    def guardar_configuracion(self):
        config = {
            'nombre_archivo': self.var_nombre_archivo.get(),
            'auto_record': self.var_auto_record_enabled.get(),
            'cooldown': self.var_cooldown.get(),
            'pre_record_time': self.var_pre_record_time.get(),
            'time_window': self.var_time_window.get(),
            'simulacion': self.var_simulacion_activada.get(),
            'matriz_calibracion': self.matriz_calibracion.tolist(),
            'promedio_habilitado': self.var_promedio_habilitado.get(),
            'promedio_muestras': self.var_promedio_muestras.get(),
            # Nuevas variables de umbral
            'umbral_mode': self.var_umbral_mode.get(),
            'auto_center': self.var_auto_center.get(),
            'center_x': self.var_center_x.get(),
            'center_y': self.var_center_y.get(),
            'center_z': self.var_center_z.get(),
            'delta_x': self.var_delta_x.get(),
            'delta_y': self.var_delta_y.get(),
            'delta_z': self.var_delta_z.get(),
            'min_x': self.var_min_x.get(),
            'max_x': self.var_max_x.get(),
            'min_y': self.var_min_y.get(),
            'max_y': self.var_max_y.get(),
            'min_z': self.var_min_z.get(),
            'max_z': self.var_max_z.get(),
        }
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)

    def cargar_configuracion(self):
        if not os.path.exists(CONFIG_FILE): return
        try:
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
            
            self.var_nombre_archivo.set(config.get('nombre_archivo', "datos"))
            self.var_auto_record_enabled.set(config.get('auto_record', False))
            self.var_cooldown.set(config.get('cooldown', '2.0'))
            self.var_pre_record_time.set(config.get('pre_record_time', '2.0'))
            self.var_time_window.set(config.get('time_window', '10'))
            self.var_simulacion_activada.set(config.get('simulacion', not self.sensor.sensor_detectado))
            self.var_promedio_habilitado.set(config.get('promedio_habilitado', False))
            self.var_promedio_muestras.set(config.get('promedio_muestras', '10'))
            
            self.var_umbral_mode.set(config.get('umbral_mode', 'Relativo'))
            self.var_auto_center.set(config.get('auto_center', False))
            self.var_center_x.set(config.get('center_x', '0.0'))
            self.var_center_y.set(config.get('center_y', '0.0'))
            self.var_center_z.set(config.get('center_z', '1.0'))
            self.var_delta_x.set(config.get('delta_x', '0.5'))
            self.var_delta_y.set(config.get('delta_y', '0.5'))
            self.var_delta_z.set(config.get('delta_z', '0.5'))
            self.var_min_x.set(config.get('min_x', '-0.5'))
            self.var_max_x.set(config.get('max_x', '0.5'))
            self.var_min_y.set(config.get('min_y', '-0.5'))
            self.var_max_y.set(config.get('max_y', '0.5'))
            self.var_min_z.set(config.get('min_z', '0.5'))
            self.var_max_z.set(config.get('max_z', '1.5'))

            matriz = config.get('matriz_calibracion')
            if matriz:
                self.matriz_calibracion = np.array(matriz)
                self.actualizar_matriz_calibracion_vars()

            if hasattr(self, 'relativo_frame'):
                self._actualizar_visibilidad_umbrales()

        except (json.JSONDecodeError, KeyError) as e:
            print(f"Error al cargar el archivo de configuración: {e}")

    def on_closing(self):
        self.guardar_configuracion()
        if self.after_job: self.root.after_cancel(self.after_job)
        try: self.ani.event_source.stop() 
        except: pass
        plt.close('all'); self.root.quit(); self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = AcelerometroApp(root)
    root.mainloop()
