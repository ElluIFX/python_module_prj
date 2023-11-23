from typing import List

from loguru import logger
from periphery import I2C as PI2C
from smbus2 import SMBus, i2c_msg


class I2CSmbus2:
    def __init__(self, bus_num: int, addr: int) -> None:
        self.bus = SMBus(bus_num)
        self.addr = addr

    def read(self) -> List[bytes]:
        try:
            read_len = i2c_msg.read(self.addr, 2)
            self.bus.i2c_rdwr(i2c_msg.write(self.addr, [0xFF]), read_len)
            len_data = int.from_bytes(bytes(read_len), "little")
            if len_data == 0:
                return []
            read_data = i2c_msg.read(self.addr, len_data + 2)
            self.bus.i2c_rdwr(i2c_msg.write(self.addr, [0xFF]), read_data)
            raw = bytes(read_data)[2:]
            data = []
            while len(raw) > 2 and raw[0] != 0:
                sz = raw[0]
                data.append(raw[1 : sz + 1])
                raw = raw[sz + 1 :]
            return data
        except Exception:
            logger.exception("Error reading from I2C bus")
        return []

    def write(self, data: bytes) -> None:
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, b"\xFF" + data))

    def close(self) -> None:
        self.bus.close()


class I2CPeriphery:
    def __init__(self, bus_num: int, addr: int) -> None:
        self.bus = PI2C(f"/dev/i2c-{bus_num}")
        self.addr = addr

    def read(self) -> List[bytes]:
        try:
            msgs = [PI2C.Message(b"\xFF"), PI2C.Message(b"\x00\x00", read=True)]
            self.bus.transfer(self.addr, msgs)
            # uint16_t
            len_data = int.from_bytes(msgs[1].data, "little")
            if len_data == 0:
                return []
            msgs = [PI2C.Message(b"\xFF"), PI2C.Message(bytes(len_data + 2), read=True)]
            self.bus.transfer(self.addr, msgs)
            raw = msgs[1].data[2:]
            data = []
            while len(raw) > 2 and raw[0] != 0:
                sz = raw[0]
                data.append(raw[1 : sz + 1])
                raw = raw[sz + 1 :]
            return data
        except Exception:
            logger.exception("Error reading from I2C bus")
        return []

    def write(self, data: bytes) -> None:
        self.bus.transfer(self.addr, [PI2C.Message(b"\xFF" + data)])

    def close(self) -> None:
        self.bus.close()


# I2C = I2CPeriphery
I2C = I2CSmbus2
