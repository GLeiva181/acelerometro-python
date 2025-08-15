import threading
import time
import tkinter as tk
import spidev
import os
from collections import deque
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.', 'lib')))
from adxl355 import ADXL355

# ==== Configuración ====
HISTORICO_SEGUNDOS = 60
FRECUENCIA_MAX_HZ = 4000
MAX_MUESTRAS = HISTORICO_SEGUNDOS * FRECUENCIA_MAX_HZ  # 1 minuto a 4 kHz

# Deque para histórico
data_deque = deque(maxlen=MAX_MUESTRAS)

# ==== Función para leer ADXL355 por SPI ====
def leer_adxl355():
    try:
        adxl355 = ADXL355()
        print("Sensor ADXL355 detectado correctamente.")
    except Exception as e:
        print(f"Sensor ADXL355 no detectado: {e}")
        return

    try:
        while True:
            data = adxl355.read_fifo(32)
            if data is not None:
                data["temp"] = adxl355.get_temperature()
                data["timestamp"] = time.time()
                data_deque.append(data)  # Guardar en histórico
    except KeyboardInterrupt:
        print("\n⏹ Lectura interrumpida.")

# ==== Función para actualizar la interfaz ====
def actualizar_ui():
    if data_deque:
        dato = data_deque[-1]  # último dato
        print(dato["timestamp"], dato["x"], dato["y"], dato["z"], dato["temp"])
        label_x.config(text=f"X: {dato['x']:.4f}")
        label_y.config(text=f"Y: {dato['y']:.4f}")
        label_z.config(text=f"Z: {dato['z']:.4f}")
    root.after(100, actualizar_ui)  # vuelve a llamar en 100 ms

# ==== Interfaz Tkinter ====
root = tk.Tk()
root.title("Lectura ADXL355 SPI")

label_x = tk.Label(root, text="X: ---", font=("Arial", 14))
label_x.pack()

label_y = tk.Label(root, text="Y: ---", font=("Arial", 14))
label_y.pack()

label_z = tk.Label(root, text="Z: ---", font=("Arial", 14))
label_z.pack()

# ==== Iniciar hilo de lectura del sensor ====
hilo_sensor = threading.Thread(target=leer_adxl355, daemon=True)
hilo_sensor.start()

# ==== Iniciar actualización de la UI ====
root.after(100, actualizar_ui)

# ==== Loop principal Tkinter ====
root.mainloop()
