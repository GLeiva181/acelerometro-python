import RPi.GPIO as GPIO
import time
import os
from collections import deque
import sys
import threading
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.', 'lib')))
from adxl355 import ADXL355

#Addresses
XDATA3 = 0x08
XDATA2 = 0x09
XDATA1 = 0x0A
YDATA3 = 0x0B
YDATA2 = 0x0C
YDATA1 = 0x0D
ZDATA3 = 0x0E
ZDATA2 = 0x0F
ZDATA1 = 0x10
RANGE = 0x2C
POWER_CTL = 0x2D
FILTER = 0x28 # New constant for Filter register
DEVID_AD = 0x00 # New constant for Device ID register
TEMP02 = 0x06
TEMP01 = 0x07
FIFO_DATA = 0x11
FIFO_SAMPLES = 0x29
FIFA_ENTRY = 0x05
INTERRUPT_MAP = 0x2A

PIN_INT = 22  # pin físico
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN_INT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

adxl = ADXL355()

counter = 0

# Función para leer FIFO
def leer_fifo(channel=None):
    counter+=1
    fifo_data = adxl.read_fifo(32)
    if fifo_data:
        print("FIFO leída:", fifo_data)

# 1️⃣ Lectura inicial si el pin ya está HIGH
if GPIO.input(PIN_INT):
    leer_fifo()

# 2️⃣ Configurar interrupción para futuros flancos de subida
GPIO.add_event_detect(PIN_INT, GPIO.RISING, callback=leer_fifo)

# 3️⃣ Función para mostrar estado del pin cada segundo
def mostrar_estado_pin():
    estado = GPIO.input(PIN_INT)
    print(adxl.read_data(INTERRUPT_MAP))
    print(f"Estado de pin {PIN_INT}: {'HIGH' if estado else 'LOW'}", "Count:", counter)
    threading.Timer(1.0, mostrar_estado_pin).start()

# Iniciar monitoreo del pin
mostrar_estado_pin()

try:
    while True:
        time.sleep(1)  # Mantener vivo
except KeyboardInterrupt:
    GPIO.cleanup()