from functools import partial
from typing import TYPE_CHECKING, List, Optional, Union

from smbus2 import SMBus, i2c_msg

from .manager import BaseInterfaceBuilder
from .templates import I2CInterfaceTemplate, I2CMessageTemplate
from .utils import get_permission


def build_msg(addr) -> type["SMBus2_I2CMessage"]:
    if TYPE_CHECKING:
        global SMBus2_I2CMessage

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
        def write(data: Union[bytes, List[int]]) -> "SMBus2_I2CMessage":
            return SMBus2_I2CMessage(False, bytes(data))

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
            return iter(bytes(self._msg))

    return SMBus2_I2CMessage


class _FakeSMBus:
    def __init__(self, bus: SMBus) -> None:
        self._bus = bus

    def __enter__(self) -> SMBus:
        return self._bus

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


class SMBus2_I2CInterface(I2CInterfaceTemplate):
    def __init__(self, bus: Union[int, str], address: int, keep_alive: bool = False):
        self._address = address
        self._keep_alive = keep_alive
        self._bus_num = bus
        try:
            self._bus_instance = SMBus(bus)
        except IOError as e:
            if e.errno == 13:
                if isinstance(bus, str):
                    get_permission(bus)
                else:
                    get_permission(f"/dev/i2c-{bus}")
                self._bus_instance = SMBus(bus)
            else:
                raise e
        if keep_alive:
            self._bus_instance = SMBus(bus)
            self._bus = partial(_FakeSMBus, self._bus_instance)
        else:
            self._bus_instance.close()
            self._bus = partial(SMBus, bus=bus)
        return super().__init__()

    @property
    def address(self) -> int:
        return self._address

    @address.setter
    def address(self, address: int):
        self._address = address

    def write_raw_byte(self, value: int):
        with self._bus() as bus:
            bus.write_byte(self._address, value)

    def read_raw_byte(self) -> int:
        with self._bus() as bus:
            return bus.read_byte(self._address)

    def write_reg_byte(self, register: int, value: int):
        with self._bus() as bus:
            bus.write_byte_data(self._address, register, value)

    def read_reg_byte(self, register: int) -> int:
        with self._bus() as bus:
            return bus.read_byte_data(self._address, register)

    def write_reg_data(self, register: int, data: Union[bytes, List[int]]):
        with self._bus() as bus:
            bus.write_i2c_block_data(self._address, register, data)

    def read_reg_data(self, register: int, length: int) -> bytes:
        with self._bus() as bus:
            return bytes(bus.read_i2c_block_data(self._address, register, length))

    @property
    def new_msg(self) -> type["SMBus2_I2CMessage"]:
        return build_msg(self._address)

    def exchange_msgs(self, msgs: list["SMBus2_I2CMessage"]):
        smbus_msgs = [msg._msg for msg in msgs]
        with self._bus() as bus:
            bus.i2c_rdwr(*smbus_msgs)

    def check_address(self) -> bool:
        try:
            with self._bus() as bus:
                bus.write_quick(self._address)
            return True
        except IOError:
            return False

    def close(self):
        if self._keep_alive:
            self._bus_instance.close()

    def reopen(self):
        assert not self._destroyed, "Interface has been destroyed"
        if self._keep_alive:
            self._bus_instance = SMBus(self._bus_num)
            self._bus = partial(_FakeSMBus, self._bus_instance)


class SMBus2_I2CInterfaceBuilder(BaseInterfaceBuilder):
    def __init__(self, bus: Union[int, str], keep_alive: bool = False):
        self._bus = bus
        self._keep_alive = keep_alive
        self.dev_type = "i2c"

    def build(self, address: int) -> SMBus2_I2CInterface:
        return SMBus2_I2CInterface(self._bus, address)
