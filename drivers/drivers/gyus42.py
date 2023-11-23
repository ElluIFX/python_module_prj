import time

from smbus2 import SMBus


class GYUS42:
    def __init__(self, address=0x70, bus=3):
        self._address = address
        self._bus = bus

    def measure(self):
        with SMBus(self._bus) as i2c_bus:
            i2c_bus.write_byte(self._address, 0x51)
        time.sleep(0.1)
        with SMBus(self._bus) as i2c_bus:
            data = i2c_bus.read_i2c_block_data(self._address, 0x00, 2)
        distance = data[0] << 8 | data[1]
        return distance

    @property
    def distance(self):
        return self.measure()

    def set_module_address(self, address):
        assert address >> 7 == 0, "Address must be 7 bits"
        address <<= 1
        with SMBus(self._bus) as i2c_bus:
            i2c_bus.write_i2c_block_data(self._address, 0xAA, [0xA5, address])
        raise Exception("Command sent, Reconect the module to make it work")
