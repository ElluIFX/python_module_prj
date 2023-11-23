from typing import List

from loguru import logger

from drivers.interface import InterfaceManager


class I2C:
    def __init__(self, addr: int) -> None:
        self.bus = InterfaceManager.request_i2c_interface("Zero-Hat", addr)

    def read(self) -> List[bytes]:
        try:
            read_len = self.bus.new_msg.read(2)
            self.bus.transfer_msg([self.bus.new_msg.write([0xFF]), read_len])
            len_data = int.from_bytes(bytes(read_len), "little")
            if len_data == 0:
                return []
            read_data = self.bus.new_msg.read(len_data + 2)
            self.bus.transfer_msg([self.bus.new_msg.write([0xFF]), read_data])
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
        self.bus.transfer_msg([self.bus.new_msg.write(b"\xFF" + data)])

    def close(self) -> None:
        self.bus.close()
