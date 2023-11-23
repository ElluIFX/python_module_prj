import time

from .interface import request_interface


class MCP4725:
    """
    MCP4725 12bit DAC driver
    """

    def __init__(self, address=0x60, vref=3.3):
        self._bus = request_interface("i2c","MCP4725", address)
        while self.read_por():  # Wait for POR
            time.sleep(0.1)
        self._vref = vref

    def write_raw_value(self, raw):
        """
        Set the DAC output to the given 12bit raw value.
        """
        data1 = (raw >> 8) & 0xFF
        data2 = raw & 0xFF
        self._bus.write_reg_byte(data1, data2)

    def write_legacy(self, data, powerdown=0, save_to_eeprom=False):
        """
        Support writing to eeprom.
        *Single data*
        """
        assert 0 <= data <= 0xFFF
        if save_to_eeprom:
            data1 = ((0x03 << 5) & 0xFF) | ((powerdown << 1) & 0xFF)
        else:
            data1 = ((0x02 << 5) & 0xFF) | ((powerdown << 1) & 0xFF)
        data2 = (data >> 4) & 0xFF
        data3 = (data << 4) & 0xFF
        self._bus.write_reg_data(data1, [data2, data3])
        if save_to_eeprom:
            while self.read_eeprom_busy():
                time.sleep(0.1)

    def read_raw(self):
        return self._bus.read_reg_data(0x00, 3)

    def read_eeprom_busy(self):
        return self.read_raw()[0] & 0b10000000 == 0

    def read_powerdown(self):
        return self.read_raw()[0] & 0b00000110 >> 1

    def read_por(self):
        """
        POR: Power On Reset
        """
        return self.read_raw()[0] & 0b01000000 == 1

    def read_data(self):
        raw = self.read_raw()
        return (raw[1] << 4 | raw[2] >> 4) & 0xFFF

    def set_powerdown(self, powerdown, data=None):
        """
        powerdown:
        0: Normal operation
        1: 1kOhm to GND
        2: 100kOhm to GND
        3: 500kOhm to GND
        """
        assert powerdown in [0, 1, 2, 3]
        if data is None:
            data = self.read_data()
        data1 = (powerdown << 4) & 0xFF | (data >> 8) & 0xFF
        data2 = data & 0xFF
        self._bus.write_reg_byte(data1, data2)

    @property
    def vref(self):
        return self._vref

    @vref.setter
    def vref(self, vref):
        self._vref = vref

    @property
    def raw_value(self):
        """
        Read raw value in 0 - max_raw
        value depends on resolution
        """
        return self.read_data()

    @raw_value.setter
    def raw_value(self, raw):
        self.write_raw_value(raw)

    @property
    def voltage(self):
        """
        Write/Read as voltage (0-Vref)
        """
        return self.read_data() / 0xFFF * self._vref

    @voltage.setter
    def voltage(self, voltage):
        voltage = max(0, min(voltage, self._vref))
        self.write_raw_value(int(voltage / self._vref * 0xFFF))

    @property
    def max_raw(self):
        return 0xFFF

    @property
    def normalized(self):
        """
        Write/Read as normalized value (0-1.0)
        """
        return self.read_data() / 0xFFF

    @normalized.setter
    def normalized(self, value):
        value = max(0, min(value, 1.0))
        self.write_raw_value(int(value * 0xFFF))

    def write_voltage_to_eeprom(self, voltage):
        """
        Set the DAC output to the given voltage and save to eeprom.
        DAC will read setting from eeprom on power up.
        """
        self.write_legacy(int(voltage / self._vref * 0xFFF), save_to_eeprom=True)

    def global_reset(self):
        """
        Reset all devices on the bus
        """
        addr = self._bus.address
        self._bus.address = 0x00
        self._bus.write_raw_byte(0x06)
        self._bus.address = addr

    def global_wakeup(self):
        """
        Wake up all devices on the bus
        """
        addr = self._bus.address
        self._bus.address = 0x00
        self._bus.write_raw_byte(0x09)
        self._bus.address = addr
