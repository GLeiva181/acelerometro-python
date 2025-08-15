import RPi.GPIO as GPIO
import time
import os
from collections import deque
import sys
import threading
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.', 'lib')))
from adxl355 import ADXL355

# ==== Direcciones ====
FIFO_DATA = 0x11
INTERRUPT_MAP = 0x2A

# ==== Configuración GPIO ====
PIN_INT = 22  # Pin físico
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN_INT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up para active-low

# ==== Inicializar sensor ====
adxl = ADXL355()

counter = 0  # contador de lecturas
first= 0
# ==== Función para leer FIFO ====
def leer_fifo(channel=None):
    global counter
    counter += 1
    fifo_data = adxl.read_fifo(32)
    if fifo_data:
        print(f"FIFO leída ({counter}):", fifo_data, time.time())

# 1️⃣ Lectura inicial si el pin ya está LOW (activo)
if GPIO.input(PIN_INT) == 0 and first==0:
    first=1
    leer_fifo()

# 2️⃣ Configurar interrupción para futuros eventos (falling edge)
GPIO.add_event_detect(PIN_INT, GPIO.FALLING, callback=leer_fifo)

# 3️⃣ Función para mostrar estado del pin cada segundo
def mostrar_estado_pin():
    estado = GPIO.input(PIN_INT)
    print(f"Estado de pin {PIN_INT}: {'LOW (activo)' if estado==0 else 'HIGH'} - Count: {counter}")
    threading.Timer(1.0, mostrar_estado_pin).start()

# Iniciar monitoreo del pin
mostrar_estado_pin()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
