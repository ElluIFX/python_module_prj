from functools import lru_cache, partial
from typing import Any, Dict, List, Optional, Union

from periphery import I2C, SPI, CdevGPIO, I2CError, Serial

from .errors import InterfaceNotFoundError
from .manager import BaseInterfaceBuilder
from .templates import (
    GPIOInterfaceTemplate,
    GpioModes_T,
    I2CInterfaceTemplate,
    I2CMessageTemplate,
    SPIInterfaceTemplate,
    UARTInterfaceTemplate,
)
from .utils import get_permission, list_gpio


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
        return super().__init__()

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

    def exchange_msgs(self, msgs: list[Periphery_I2CMessage]) -> None:
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
        assert not self._destroyed, "Interface has been destroyed"
        if self._keep_alive:
            self._i2c_instance = I2C(self._i2c_instance.devpath)
            self._i2c = partial(_FakeI2C, self._i2c_instance)


class Periphery_I2CInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "i2c"

    def __init__(self, devpath: str, keep_alive: bool = False) -> None:
        self._devpath = devpath
        self._keep_alive = keep_alive

    def build(self, address: int) -> Periphery_I2CInterface:
        return Periphery_I2CInterface(self._devpath, address, self._keep_alive)


class Periphery_UARTInterface(UARTInterfaceTemplate):
    def __init__(self, devpath: str, baudrate: int) -> None:
        try:
            self._uart = Serial(devpath, baudrate=baudrate)
        except IOError as e:
            if e.errno == 13:
                get_permission(devpath)
                self._uart = Serial(devpath, baudrate=baudrate)
            else:
                raise e
        self._devpath = devpath
        self._timeout = None
        return super().__init__()

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
    def stop_bits(self) -> str:
        return str(self._uart.stopbits)

    @stop_bits.setter
    def stop_bits(self, stop_bits: str) -> None:
        self._uart.stopbits = int(stop_bits)

    @property
    def parity(self) -> str:
        remap = {"none": "N", "even": "E", "odd": "O"}
        return remap[self._uart.parity]

    @parity.setter
    def parity(self, parity: str) -> None:
        remap = {"N": "none", "E": "even", "O": "odd"}
        self._uart.parity = remap[parity]

    @property
    def timeout(self) -> Optional[float]:
        return self._timeout

    @timeout.setter
    def timeout(self, timeout: Optional[float]) -> None:
        self._timeout = timeout

    def write(self, data: bytes) -> None:
        self._uart.write(data)

    def read(self, length: int = 1) -> bytes:
        return self._uart.read(length, timeout=self._timeout)

    def flush(self) -> None:
        self._uart.flush()

    def close(self) -> None:
        self._uart.close()

    def reopen(self):
        assert not self._destroyed, "Interface has been destroyed"
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


class Periphery_UARTInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "uart"

    def __init__(self, devpath: str) -> None:
        self._devpath = devpath

    def build(self, baudrate: int) -> Periphery_UARTInterface:
        return Periphery_UARTInterface(self._devpath, baudrate)


class Periphery_SPIInterface(SPIInterfaceTemplate):
    def __init__(self, devpath: str, mode: int, speed_hz: int) -> None:
        try:
            self._spi = SPI(devpath, mode=mode, max_speed=speed_hz)
        except IOError as e:
            if e.errno == 13:
                get_permission(devpath)
                self._spi = SPI(devpath, mode=mode, max_speed=speed_hz)
            else:
                raise e
        self._devpath = devpath
        return super().__init__()

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

    @property
    def byteorder(self) -> str:
        return self._spi.bit_order

    @byteorder.setter
    def byteorder(self, byteorder: str) -> None:
        self._spi.bit_order = byteorder

    @property
    def bits_per_word(self) -> int:
        return self._spi.bits_per_word

    @bits_per_word.setter
    def bits_per_word(self, bits_per_word: int) -> None:
        self._spi.bits_per_word = bits_per_word

    def write(self, data: bytes) -> None:
        if len(data) > 4096:
            for i in range(0, len(data), 4096):
                self._spi.transfer(data[i : i + 4096])
        else:
            self._spi.transfer(data)

    def read(self, length: int) -> bytes:
        return bytes(self._spi.transfer(bytes(length)))

    def transfer(self, data: bytes) -> bytes:
        return bytes(self._spi.transfer(data))

    def close(self) -> None:
        self._spi.close()

    def reopen(self) -> None:
        assert not self._destroyed, "Interface has been destroyed"
        mode = self._spi.mode
        speed_hz = self._spi.max_speed
        byteorder = self._spi.bit_order
        bits_per_word = self._spi.bits_per_word
        self._spi = SPI(
            self._devpath,
            mode=mode,
            max_speed=speed_hz,
            bit_order=byteorder,
            bits_per_word=bits_per_word,
        )


class Periphery_SPIInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "spi"

    def __init__(self, devpath: str) -> None:
        self._devpath = devpath

    def build(self, mode: int, speed_hz: int) -> Periphery_SPIInterface:
        return Periphery_SPIInterface(self._devpath, mode, speed_hz)


class Periphery_GPIOInterface(GPIOInterfaceTemplate):
    def __init__(
        self,
        pinmap: Optional[Dict[str, str]],
        modemap: Optional[Dict[str, GpioModes_T]],
    ) -> None:
        self._pinmap = pinmap if pinmap is not None else {}
        self._modemap = modemap if modemap is not None else {}
        self._pins: Dict[str, CdevGPIO] = {}
        self._gpios = list_gpio()
        self._pinmap_inv = {v: k for k, v in self._pinmap.items()}
        self._pinmodes: Dict[str, GpioModes_T] = {}
        return super().__init__()

    @lru_cache(64)
    def _remap(self, pin_name: str) -> str:
        if self._pinmap is not None:
            pin_name = self._pinmap.get(pin_name, pin_name)
        if pin_name not in self._gpios:
            raise InterfaceNotFoundError(f"GPIO {pin_name} not found")
        return pin_name

    @lru_cache(1)
    def get_available_pins(self) -> Dict[str, List[GpioModes_T]]:
        return {
            self._pinmap_inv.get(name, name): [
                "input_no_pull",
                "input_pull_down",
                "input_pull_up",
                "output_open_drain",
                "output_open_source",
                "output_push_pull",
            ]
            for name in self._gpios.keys()
        }

    def free(self, pin_name: str) -> None:
        pin_name = self._remap(pin_name)
        if pin_name in self._pins:
            self._pins[pin_name].close()
            self._pins.pop(pin_name)

    def set_mode(self, pin_name: str, mode: GpioModes_T) -> None:
        mode_map: Dict[str, Any] = {
            "input_no_pull": {"direction": "in", "bias": "disable"},
            "input_pull_down": {"direction": "in", "bias": "pull_down"},
            "input_pull_up": {"direction": "in", "bias": "pull_up"},
            "output_open_drain": {"direction": "out", "drive": "open_drain"},
            "output_open_source": {"direction": "out", "drive": "open_source"},
            "output_push_pull": {"direction": "out", "drive": "default"},
        }
        pin_name = self._remap(pin_name)
        if pin_name in self._pins:
            self._pins[pin_name].close()
            self._pins.pop(pin_name)
        if pin_name in self._modemap:
            if self._modemap[pin_name].split("_")[0] == mode.split("_")[0]:
                mode = self._modemap[pin_name]
        chip, offset, used = self._gpios[pin_name]
        # assert not used, f"GPIO {pin_name} is already used"
        try:
            self._pins[pin_name] = CdevGPIO(
                path=f"/dev/gpiochip{chip}", line=offset, **mode_map[mode]
            )
        except IOError as e:
            if e.errno == 13:
                get_permission(f"/dev/gpiochip{chip}")
                self._pins[pin_name] = CdevGPIO(
                    path=f"/dev/gpiochip{chip}", line=offset, **mode_map[mode]
                )
            else:
                raise e
        self._pinmodes[pin_name] = mode

    def get_mode(self, pin_name: str) -> GpioModes_T:
        pin_name = self._remap(pin_name)
        return self._pinmodes.get(pin_name, "none")

    def write(self, pin_name: str, value: bool):
        pin_name = self._remap(pin_name)
        assert pin_name in self._pins, f"GPIO {pin_name} not initialized"
        self._pins[pin_name].write(value)

    def read(self, pin_name: str) -> bool:
        pin_name = self._remap(pin_name)
        assert pin_name in self._pins, f"GPIO {pin_name} not initialized"
        return self._pins[pin_name].read()

    def close(self) -> None:
        for pin in self._pins.values():
            pin.close()
        self._pins.clear()


class Periphery_GPIOInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "gpio"

    def __init__(
        self,
        pinmap: Optional[Dict[str, str]] = None,
        modemap: Optional[Dict[str, GpioModes_T]] = None,
    ) -> None:
        self._pinmap = pinmap
        self._modemap = modemap

    def build(self) -> Periphery_GPIOInterface:
        return Periphery_GPIOInterface(self._pinmap, self._modemap)
