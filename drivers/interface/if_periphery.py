from typing import List, Optional, Union

from periphery import I2C, SPI, I2CError, Serial

from .manager import (
    I2CInterfaceTemplate,
    I2CMessageTemplate,
    InterfaceBuilderTemplate,
    SPIInterfaceTemplate,
    UartInterfaceTemplate,
)


class Periphery_I2CMessage(I2CMessageTemplate):
    def __init__(
        self,
        read: bool,
        data: Optional[bytes] = None,
        len: Optional[int] = None,
    ) -> None:
        if read and len is not None:
            self._msg = I2C.Message(bytes(len), read=True)
        elif not read and data is not None:
            self._msg = I2C.Message(data)
        else:
            raise ValueError("Invalid message")

    @staticmethod
    def write(data: bytes) -> "Periphery_I2CMessage":
        return Periphery_I2CMessage(False, data)

    @staticmethod
    def read(length: int) -> "Periphery_I2CMessage":
        return Periphery_I2CMessage(True, None, length)

    def __len__(self) -> int:
        return len(self._msg.data)

    def __bytes__(self) -> bytes:
        return bytes(self._msg.data)

    def __repr__(self) -> str:
        return repr(self._msg)

    def __iter__(self):
        """Return an iterator over the data."""
        return iter(bytes(self))


class Periphery_I2CInterface(I2CInterfaceTemplate):
    def __init__(self, devpath: str, addr: int) -> None:
        self._i2c = I2C(devpath)
        self._addr = addr

    def set_address(self, address: int) -> None:
        self._addr = address

    def write_raw_byte(self, value: int) -> None:
        self._i2c.transfer(self._addr, [I2C.Message([value])])

    def read_raw_byte(self) -> int:
        msg = I2C.Message([0], read=True)
        self._i2c.transfer(self._addr, [msg])
        return int(msg.data[0])

    def write_byte(self, register: int, value: int) -> None:
        self._i2c.transfer(self._addr, [I2C.Message([register, value & 0xFF])])

    def read_byte(self, register: int) -> int:
        msg = I2C.Message([0], read=True)
        self._i2c.transfer(self._addr, [I2C.Message([register]), msg])
        return int(msg.data[0])

    def write_data(self, register: int, data: Union[bytes, List[int]]) -> None:
        self._i2c.transfer(self._addr, [I2C.Message([register] + list(data))])

    def read_data(self, register: int, length: int) -> bytes:
        msg = I2C.Message(bytes(length), read=True)
        self._i2c.transfer(self._addr, [I2C.Message([register]), msg])
        return bytes(msg.data)

    def new_msg(self) -> type[Periphery_I2CMessage]:
        return Periphery_I2CMessage

    def exchange_msg(self, msgs: list[Periphery_I2CMessage]) -> None:
        self._i2c.transfer(self._addr, [msg._msg for msg in msgs])

    def check_address(self) -> bool:
        try:
            self._i2c.transfer(self._addr, [I2C.Message([0])])
            return True
        except I2CError:
            return False


class Periphery_I2CInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, devpath: str) -> None:
        self._devpath = devpath
        self.dev_type = "i2c"

    def build(self, address: int) -> Periphery_I2CInterface:
        return Periphery_I2CInterface(self._devpath, address)


class Periphery_UartInterface(UartInterfaceTemplate):
    def __init__(self, devpath: str, baudrate: int) -> None:
        self._uart = Serial(devpath, baudrate=baudrate)
        self._devpath = devpath
        self._baudrate = baudrate

    def set_baudrate(self, baudrate: int):
        self._uart.baudrate = baudrate

    def set_option(self, data_bits: int, parity: str, stop_bits: int):
        self._uart.databits = data_bits
        self._uart.parity = parity
        self._uart.stopbits = stop_bits

    def write(self, data: bytes) -> None:
        self._uart.write(data)

    def read(self, length: int, timeout: Optional[float] = None) -> bytes:
        return self._uart.read(length, timeout)

    def flush(self) -> None:
        self._uart.flush()

    def close(self) -> None:
        self._uart.close()

    def reopen(self):
        self._uart = Serial(self._devpath, baudrate=self._baudrate)

    @property
    def in_waiting(self) -> int:
        return self._uart.input_waiting()


class Periphery_UartInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, devpath: str) -> None:
        self._devpath = devpath
        self.dev_type = "uart"

    def build(self, baudrate: int) -> Periphery_UartInterface:
        return Periphery_UartInterface(self._devpath, baudrate)


class Periphery_SPIInterface(SPIInterfaceTemplate):
    def __init__(self, devpath: str, mode: int, speed_hz: int) -> None:
        self._spi = SPI(devpath, mode=mode, max_speed=speed_hz)
        self._devpath = devpath
        self._mode = mode
        self._max_speed = speed_hz

    def set_mode(self, mode: int) -> None:
        self._spi.mode = mode
        self._mode = mode

    def set_speed_hz(self, speed_hz: int) -> None:
        self._spi.max_speed = speed_hz
        self._max_speed = speed_hz

    def write(self, data: bytes) -> None:
        self._spi.transfer(data)

    def read(self, length: int) -> bytes:
        return bytes(self._spi.transfer(bytes(length)))

    def transfer(self, data: bytes) -> bytes:
        return bytes(self._spi.transfer(data))

    def close(self) -> None:
        self._spi.close()

    def reopen(self) -> None:
        self._spi = SPI(self._devpath, mode=self._mode, max_speed=self._max_speed)


class Periphery_SPIInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, devpath: str) -> None:
        self._devpath = devpath
        self.dev_type = "spi"

    def build(self, mode: int, speed_hz: int) -> Periphery_SPIInterface:
        return Periphery_SPIInterface(self._devpath, mode, speed_hz)
