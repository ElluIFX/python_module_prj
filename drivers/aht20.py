import time

from .interface import request_interface


class AHT20:
    # I2C communication driver for AHT20, using only smbus2
    _AHT20_I2CADDR = 0x38
    _AHT20_CMD_SOFTRESET = [0xBA]
    _AHT20_CMD_INITIALIZE = [0xBE, 0x08, 0x00]
    _AHT20_CMD_MEASURE = [0xAC, 0x33, 0x00]

    def __init__(self):
        # Initialize AHT20
        self.dev = request_interface("i2c", "AHT20", self._AHT20_I2CADDR)
        self._cmd_soft_reset()

        # Check for calibration, if not done then do and wait 10 ms
        if not self._calibrated:
            self._cmd_initialize()
            while not self._calibrated:
                time.sleep(0.01)

    def _cmd_soft_reset(self):
        # Send the command to soft reset
        self.dev.write_reg_data(0x0, self._AHT20_CMD_SOFTRESET)
        time.sleep(0.04)  # Wait 40 ms after poweron
        return True

    def _cmd_initialize(self):
        # Send the command to initialize (calibrate)
        self.dev.write_reg_data(0x0, self._AHT20_CMD_INITIALIZE)

    def _cmd_measure(self):
        # Send the command to measure
        self.dev.write_reg_data(0x0, self._AHT20_CMD_MEASURE)
        time.sleep(0.01)  # Wait 80 ms after measure

    @property
    def _status(self):
        # Get the full status byte
        return self.dev.read_reg_data(0x0, 1)[0]

    @property
    def _calibrated(self):
        # Get the calibrated bit
        return self._status >> 3 & 0x01 == 0x01

    @property
    def _is_busy(self):
        # Get the busy bit
        return self._status >> 7 & 0x01 == 0x01

    def measure(self):
        # Get the full measure

        self._cmd_measure()
        while self._is_busy == 1:
            time.sleep(0.01)
        self._measure_data = self.dev.read_reg_data(0x0, 7)

    @property
    def temperature(self):
        # Get a measure, select proper bytes, return converted data
        measure = self._measure_data
        temp = ((measure[3] & 0xF) << 16) | (measure[4] << 8) | measure[5]
        temp = temp / (pow(2, 20)) * 200 - 50
        return temp

    @property
    def humidity(self):
        # Get a measure, select proper bytes, return converted data
        measure = self._measure_data
        humi = (measure[1] << 12) | (measure[2] << 4) | (measure[3] >> 4)
        humi = humi * 100 / pow(2, 20)
        return humi
