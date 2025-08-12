"""
Usage example for ADXL355 Python library

This example prints on console the current values
of axes on accelerometer
"""

import sys
#sys.path.append('../lib/')
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'lib')))

from adxl355 import ADXL355  # pylint: disable=wrong-import-position

device = ADXL355()           # pylint: disable=invalid-name
valor = device.read_data(0x00)  # devuelve un entero, por ejemplo 173
print(f"DevID: 0x{valor:02X}")
valor = device.read_data(0x01)  # devuelve un entero, por ejemplo 173
print(f"DevID: 0x{valor:02X}")
valor = device.read_data(0x02)  # devuelve un entero, por ejemplo 173
print(f"DevID: 0x{valor:02X}")
valor = device.read_data(0x03)  # devuelve un entero, por ejemplo 173
print(f"DevID: 0x{valor:02X}")
valor = device.read_data(0x2C)  # devuelve un entero, por ejemplo 173
print(f"DevID: 0x{valor:02X}")
axes = device.get_axes()     # pylint: disable=invalid-name
print(axes)
print("Temp: ", device.get_temperature())
print("Range: ", device.get_measure_range())
print("Norm: ", device.get_axes_norm())

