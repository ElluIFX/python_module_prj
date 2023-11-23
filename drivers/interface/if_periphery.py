from functools import partial
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
    def write(data: Union[bytes, List[int]]) -> "Periphery_I2CMessage":
        return Periphery_I2CMessage(False, bytes(data))

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


class _FakeI2C:
    def __init__(self, bus: I2C) -> None:
        self._bus = bus

    def __enter__(self) -> I2C:
        return self._bus

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


class Periphery_I2CInterface(I2CInterfaceTemplate):
    def __init__(self, devpath: str, addr: int, keep_alive: bool = False) -> None:
        self._addr = addr
        self._keep_alive = keep_alive
        if keep_alive:
            self._i2c_instance = I2C(devpath)
            self._i2c = partial(_FakeI2C, self._i2c_instance)
        else:
            self._i2c = partial(I2C, devpath=devpath)

    @property
    def address(self) -> int:
        return self._addr

    @address.setter
    def address(self, address: int) -> None:
        self._addr = address

    def write_raw_byte(self, value: int) -> None:
        with self._i2c() as i2c:
            i2c.transfer(self._addr, [I2C.Message([value])])

    def read_raw_byte(self) -> int:
        msg = I2C.Message([0], read=True)
        with self._i2c() as i2c:
            i2c.transfer(self._addr, [msg])
        return int(msg.data[0])

    def write_reg_byte(self, register: int, value: int) -> None:
        with self._i2c() as i2c:
            i2c.transfer(self._addr, [I2C.Message([register, value & 0xFF])])

    def read_reg_byte(self, register: int) -> int:
        msg = I2C.Message([0], read=True)
        with self._i2c() as i2c:
            i2c.transfer(self._addr, [I2C.Message([register]), msg])
        return int(msg.data[0])

    def write_reg_data(self, register: int, data: Union[bytes, List[int]]) -> None:
        with self._i2c() as i2c:
            i2c.transfer(self._addr, [I2C.Message([register] + list(data))])

    def read_reg_data(self, register: int, length: int) -> bytes:
        msg = I2C.Message(bytes(length), read=True)
        with self._i2c() as i2c:
            i2c.transfer(self._addr, [I2C.Message([register]), msg])
        return bytes(msg.data)

    @property
    def new_msg(self) -> type[Periphery_I2CMessage]:
        return Periphery_I2CMessage

    def transfer_msg(self, msgs: list[Periphery_I2CMessage]) -> None:
        with self._i2c() as i2c:
            i2c.transfer(self._addr, [msg._msg for msg in msgs])

    def check_address(self) -> bool:
        try:
            with self._i2c() as i2c:
                i2c.transfer(self._addr, [I2C.Message([0])])
            return True
        except I2CError:
            return False

    def close(self):
        if self._keep_alive:
            self._i2c_instance.close()

    def reopen(self):
        if self._keep_alive:
            self._i2c_instance = I2C(self._i2c_instance.devpath)
            self._i2c = partial(_FakeI2C, self._i2c_instance)


class Periphery_I2CInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, devpath: str, keep_alive: bool = False) -> None:
        self._devpath = devpath
        self._keep_alive = keep_alive
        self.dev_type = "i2c"

    def build(self, address: int) -> Periphery_I2CInterface:
        return Periphery_I2CInterface(self._devpath, address, self._keep_alive)


class Periphery_UartInterface(UartInterfaceTemplate):
    def __init__(self, devpath: str, baudrate: int) -> None:
        self._uart = Serial(devpath, baudrate=baudrate)
        self._devpath = devpath

    @property
    def baudrate(self) -> int:
        return self._uart.baudrate

    @baudrate.setter
    def baudrate(self, baudrate: int) -> None:
        self._uart.baudrate = baudrate

    @property
    def data_bits(self) -> int:
        return self._uart.databits

    @data_bits.setter
    def data_bits(self, data_bits: int) -> None:
        self._uart.databits = data_bits

    @property
    def stop_bits(self) -> int:
        return self._uart.stopbits

    @stop_bits.setter
    def stop_bits(self, stop_bits: int) -> None:
        self._uart.stopbits = stop_bits

    @property
    def parity(self) -> str:
        return self._uart.parity

    @parity.setter
    def parity(self, parity: str) -> None:
        self._uart.parity = parity

    def write(self, data: bytes) -> None:
        self._uart.write(data)

    def read(self, length: int, timeout: Optional[float] = None) -> bytes:
        return self._uart.read(length, timeout)

    def flush(self) -> None:
        self._uart.flush()

    def close(self) -> None:
        self._uart.close()

    def reopen(self):
        baudrate = self._uart.baudrate
        data_bits = self._uart.databits
        stop_bits = self._uart.stopbits
        parity = self._uart.parity
        self._uart = Serial(
            self._devpath,
            baudrate=baudrate,
            databits=data_bits,
            stopbits=stop_bits,
            parity=parity,
        )

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

    @property
    def mode(self) -> int:
        return self._spi.mode

    @mode.setter
    def mode(self, mode: int) -> None:
        self._spi.mode = mode

    @property
    def speed_hz(self) -> float:
        return self._spi.max_speed

    @speed_hz.setter
    def speed_hz(self, speed_hz: int) -> None:
        self._spi.max_speed = speed_hz

    def write(self, data: bytes) -> None:
        self._spi.transfer(data)

    def read(self, length: int) -> bytes:
        return bytes(self._spi.transfer(bytes(length)))

    def transfer(self, data: bytes) -> bytes:
        return bytes(self._spi.transfer(data))

    def close(self) -> None:
        self._spi.close()

    def reopen(self) -> None:
        mode = self._spi.mode
        speed_hz = self._spi.max_speed
        self._spi = SPI(self._devpath, mode=mode, max_speed=speed_hz)

    def set_auto_cs(self, enable: bool, polarity: bool):
        pass


class Periphery_SPIInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, devpath: str) -> None:
        self._devpath = devpath
        self.dev_type = "spi"

    def build(self, mode: int, speed_hz: int) -> Periphery_SPIInterface:
        return Periphery_SPIInterface(self._devpath, mode, speed_hz)
