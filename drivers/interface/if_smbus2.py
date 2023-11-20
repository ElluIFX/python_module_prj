from typing import List, Optional, Union

from smbus2 import SMBus, i2c_msg

from .manager import I2CInterfaceTemplate, I2CMessageTemplate, InterfaceBuilderTemplate


def build_msg(addr) -> type[I2CMessageTemplate]:
    class SMBus2_I2CMessage(I2CMessageTemplate):
        _addr = addr

        def __init__(
            self,
            read: bool,
            data: Optional[bytes] = None,
            len: Optional[int] = None,
        ) -> None:
            if read and len is not None:
                self._msg = i2c_msg.read(self._addr, len)
            elif not read and data is not None:
                self._msg = i2c_msg.write(self._addr, data)
            else:
                raise ValueError("Invalid message")

        @staticmethod
        def write(data: bytes) -> "SMBus2_I2CMessage":
            return SMBus2_I2CMessage(False, data)

        @staticmethod
        def read(length: int) -> "SMBus2_I2CMessage":
            return SMBus2_I2CMessage(True, None, length)

        def __len__(self) -> int:
            return len(self._msg)

        def __bytes__(self) -> bytes:
            return bytes(self._msg)

        def __repr__(self) -> str:
            return repr(self._msg)

        def __iter__(self):
            return iter(bytes(self))

    return SMBus2_I2CMessage


class SMBus2_I2CInterface(I2CInterfaceTemplate):
    def __init__(self, bus: int, address: int):
        self._bus = bus
        self._address = address

    def set_address(self, address: int):
        self._address = address

    def write_raw_byte(self, value: int):
        with SMBus(self._bus) as bus:
            bus.write_byte(self._address, value)

    def read_raw_byte(self) -> int:
        with SMBus(self._bus) as bus:
            return bus.read_byte(self._address)

    def write_byte(self, register: int, value: int):
        with SMBus(self._bus) as bus:
            bus.write_byte_data(self._address, register, value)

    def read_byte(self, register: int) -> int:
        with SMBus(self._bus) as bus:
            return bus.read_byte_data(self._address, register)

    def write_data(self, register: int, data: Union[bytes, List[int]]):
        with SMBus(self._bus) as bus:
            bus.write_i2c_block_data(self._address, register, data)

    def read_data(self, register: int, length: int) -> bytes:
        with SMBus(self._bus) as bus:
            return bytes(bus.read_i2c_block_data(self._address, register, length))

    def new_msg(self) -> type[I2CMessageTemplate]:
        return build_msg(self._address)

    def exchange_msg(self, msgs: list[I2CMessageTemplate]):
        with SMBus(self._bus) as bus:
            bus.i2c_rdwr(msgs)  # type: ignore

    def check_address(self) -> bool:
        try:
            with SMBus(self._bus) as bus:
                bus.write_quick(self._address)
            return True
        except IOError:
            return False


class SMBus2_I2CInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, bus: int):
        self._bus = bus
        self.dev_type = "i2c"

    def build(self, address: int) -> SMBus2_I2CInterface:
        return SMBus2_I2CInterface(self._bus, address)
