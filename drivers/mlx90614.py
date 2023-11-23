import time

from .interface import InterfaceManager

MLX90614_RAWIR1 = 0x04
MLX90614_RAWIR2 = 0x05
MLX90614_TA = 0x06
MLX90614_TOBJ1 = 0x07
MLX90614_TOBJ2 = 0x08

MLX90614_TOMAX = 0x20
MLX90614_TOMIN = 0x21
MLX90614_PWMCTRL = 0x22
MLX90614_TARANGE = 0x23
MLX90614_EMISS = 0x24
MLX90614_CONFIG = 0x25
MLX90614_ADDR = 0x0E
MLX90614_ID1 = 0x3C
MLX90614_ID2 = 0x3D
MLX90614_ID3 = 0x3E
MLX90614_ID4 = 0x3F


class MLX90614:
    def __init__(self, address=0x5A):
        self._bus = InterfaceManager.request_i2c_interface("MLX90614", address)

    def _read_word(self, reg):
        data = self._bus.read_reg_data(reg, 2)
        return data[0] + (data[1] << 8)

    def _write_word(self, reg, value):
        self._bus.write_reg_data(reg, [value & 0xFF, value >> 8])

    def _read_temp(self, reg):
        temp = self._read_word(reg)
        temp *= 0.02
        temp -= 273.15
        return temp

    @property
    def ambient_temperature(self):
        return self._read_temp(MLX90614_TA)

    @property
    def object_temperature(self):
        return self._read_temp(MLX90614_TOBJ1)

    @property
    def emissivity(self):
        return self._read_word(MLX90614_EMISS) / 65535.0

    @emissivity.setter
    def emissivity(self, value):
        self._write_word(MLX90614_EMISS, 0)
        time.sleep(0.1)
        self._write_word(MLX90614_EMISS, int(value * 65535))
        time.sleep(0.1)
