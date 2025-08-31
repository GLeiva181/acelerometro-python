import os
import time
from collections import deque
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.', 'lib')))
from adxl355 import ADXL355

DEVID_AD = 0x00
FIFO_DATA = 0x11

sensor_detectado = False
try:
    adxl355 = ADXL355()
    sensor_detectado = True
    print("Sensor ADXL355 detectado correctamente.")
except Exception as e:
    print(f"Sensor ADXL355 no detectado: {e}")

try:
    print("DEVID_AD: ", adxl355.read_data(DEVID_AD))
    print("INT MAP: ", adxl355.get_interrupt())

    while True:
        # print(adxl355.fifo_entries())
        temp = adxl355.get_temperature()
        xyz = adxl355.get_axes()
        print(temp, xyz, time.time())
        time.sleep(5)

except KeyboardInterrupt:
    print("\n⏹ Lectura interrumpida.")