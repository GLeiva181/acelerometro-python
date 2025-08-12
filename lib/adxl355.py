"""ADXL355 Python library for Raspberry PI.

This module implements basic operations for ADXL355
accelerometer chip for Raspberry Pi boards

.. _ADXL355 PMDZ Info:
   https://wiki.analog.com/resources/eval/user-guides/eval-adicup360/hardware/adxl355

"""

import spidev

# ADXL345 constants

# SPI config
SPI_MAX_CLOCK_HZ = 10000000
SPI_MODE = 0b00
SPI_BUS = 0
SPI_DEVICE = 0

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
DEVID_AD = 0x00 # New constant for Device ID register
TEMP02 = 0x06
TEMP01 = 0x07


# Data Range
RANGE_2G = 0x01
RANGE_4G = 0x02
RANGE_8G = 0x03

# Values
READ_BIT = 0x01
WRITE_BIT = 0x00
DUMMY_BYTE = 0xAA
MEASURE_MODE = 0x04 # Only accelerometer

class ADXL355:
    """
    Class to interact with ADXL355 device

    Allows user to read, write and obtain data
    from the accelerometers
    """
    def __init__(self, measure_range=RANGE_2G):
        # SPI init
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_MAX_CLOCK_HZ
        self.spi.mode = SPI_MODE

        # Check device ID
        device_id = self.read_data(DEVID_AD)
        if device_id != 0xAD:
            raise RuntimeError(f"ADXL355 sensor not found or not responding. Expected DEVID_AD 0xAD, got 0x{device_id:02X}")

        # Device init
        self._set_measure_range(measure_range)
        self._enable_measure_mode()

    def write_data(self, address, value):
        """Writes data on ADXL355 device address.

        Args:
            address (int): Address to write in ADXL355.
            value (int): Value to write in address.

        Returns:
            None
        """
        device_address = address << 1 | WRITE_BIT
        self.spi.xfer2([device_address, value])

    def read_data(self, address):
        """Reads data from ADXL355 device.

        Args:
            address (int): Address to read from ADXL355.

        Returns:
            int: Value in speficied address in accelerometer
        """
        device_address = address << 1 | READ_BIT
        return self.spi.xfer2([device_address, DUMMY_BYTE])[1]

    def read_multiple_data(self, address_list):
        """Reads multiple data from ADXL355 device.

        Args:
            address_list (list): List of addresses to read from.

        Returns:
            list: Value of each address in accelerometer
        """
        spi_ops = []
        for address in address_list:
            spi_ops.append(address << 1 | READ_BIT)
        spi_ops.append(DUMMY_BYTE)

        return self.spi.xfer2(spi_ops)[1:]

    def _set_measure_range(self, measure_range):
        """Sets measure range on ADXL355 device.

        Args:
            measure_range (int): Measure range to set in ADXL355.

        Returns:
            None
        """
        self.write_data(RANGE, measure_range)
    
    def get_measure_range(self):
        range_value = self.read_data(RANGE) & 0x03

        if range_value == RANGE_2G:
            return 2
        elif range_value == RANGE_4G:
            return 4
        elif range_value == RANGE_8G:
            return 8
        else:
            raise ValueError("Invalid measure range value")
        return -1

    def _enable_measure_mode(self):
        """
        Enables measure mode on ADXL355 device.

        Returns:
            None
        """
        self.write_data(POWER_CTL, MEASURE_MODE)

    def get_axes(self):
        """
        Gets the current data from the axes.

        Returns:
            dict: Current value for x, y and z axis
        """

        # Reading data
        raw_data = self.read_multiple_data(
            [XDATA3, XDATA2, XDATA1, YDATA3, YDATA2, YDATA1, ZDATA3, ZDATA2, ZDATA1]
        )

        # Split data
        x_data = raw_data[0] << 16 | raw_data[1] << 8 | raw_data[2]
        x_data >>= 4
        if x_data & (1 << 19):
            x_data = x_data - (1 << 20)
        y_data = raw_data[3] << 16 | raw_data[4] << 8 | raw_data[5]
        y_data >>= 4
        if y_data & (1 << 19):
            y_data = y_data - (1 << 20)
        z_data = raw_data[6] << 16 | raw_data[7] << 8 | raw_data[8]
        z_data >>= 4
        if z_data & (1 << 19):
            z_data = z_data - (1 << 20)
        # Return values
        return {'x': x_data, 'y': y_data, 'z': z_data}
    
    def get_axes_norm(self):
        measure_range=self.get_measure_range
        raw=self.get_axes()
        if measure_range == 2:
            scale_factor = 256000  # LSB por g en ±2g
        elif measure_range == 4:
            scale_factor = 128000  # LSB por g en ±4g
        elif measure_range == 8:
            scale_factor = 64000  # LSB por g en ±8g
        else:
            raise ValueError("Invalid measure range value")
        accel_g = {axis: value / scale_factor for axis, value in raw.items()}
        return accel_g

    def get_temperature(self):
        temp2=self.read_data(TEMP02)
        temp1=self.read_data(TEMP01)
        temp_raw = (temp2 << 8) | temp1
        # if temp_raw & (1 << 11):
        #     temp_raw -= (1 << 12)  # sign extend, no aplica aquí
        offset = 1885
        slope = 9.05
        temp_c = 25 - (temp_raw - offset) / slope
        return temp_c
    

