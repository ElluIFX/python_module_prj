import time
from typing import Iterable

from smbus2 import SMBus


class MCP4725:
    def __init__(self, address=0x60, bus=3, vref=3.3):
        self._address = address
        self._bus = bus
        while self.read_por():  # Wait for POR
            time.sleep(0.1)
        self._vref = vref
        self._continuous_i2c = None

    def write_fastmode(self, data, delay=0, ):
        """
        Set the DAC output to the given 12bit raw value list.
        """
        if not issubclass(type(data), Iterable):
            data = [data]
        with SMBus(self._bus) as i2c_bus:
            cnt = 0
            for raw in data:
                data1 = (raw >> 8) & 0xFF
                data2 = raw & 0xFF
                if cnt == 0:
                    i2c_bus.write_byte_data(
                        self._address,
                        data1,
                        data2,
                    )
                else:
                    i2c_bus.write_byte(data1, data2)
                if delay:
                    time.sleep(delay)

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
        with SMBus(self._bus) as i2c_bus:
            i2c_bus.write_i2c_block_data(self._address, data1, [data2, data3])
        if save_to_eeprom:
            while self.read_eeprom_busy():
                time.sleep(0.1)

    def read_raw(self):
        with SMBus(self._bus) as i2c_bus:
            data = i2c_bus.read_i2c_block_data(self._address, 0x00, 3)
        return data

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
        with SMBus(self._bus) as i2c_bus:
            i2c_bus.write_i2c_block_data(self._address, data1, [data2])

    def set_vref(self, vref):
        self._vref = vref

    @property
    def voltage(self):
        """
        Write/Read as voltage (0-Vref)
        """
        return self.read_data() / 0xFFF * self._vref

    @voltage.setter
    def voltage(self, voltage):
        voltage = max(0, min(voltage, self._vref))
        self.write_fastmode(int(voltage / self._vref * 0xFFF))

    @property
    def normalized(self):
        """
        Write/Read as normalized value (0-1.0)
        """
        return self.read_data() / 0xFFF

    @normalized.setter
    def normalized(self, value):
        value = max(0, min(value, 1.0))
        self.write_fastmode(int(value * 0xFFF))

    def write_voltages(self, voltages, delay=0):
        """
        Set the DAC output to the given voltage list.
        """
        lst = [int(v / self._vref * 0xFFF) for v in voltages]
        self.write_fastmode(lst, delay)

    def write_voltage_to_eeprom(self, voltage):
        """
        Set the DAC output to the given voltage and save to eeprom.
        DAC will read setting from eeprom on power up.
        """
        self.write_legacy(int(voltage / self._vref * 0xFFF), save_to_eeprom=True)

    def global_reset(self):
        with SMBus(self._bus) as i2c_bus:
            i2c_bus.write_byte(0x00, 0x06)

    def global_wakeup(self):
        with SMBus(self._bus) as i2c_bus:
            i2c_bus.write_byte(0x00, 0x09)
