import time
from functools import lru_cache
from threading import Lock
from typing import Dict, List, Literal, Optional, Union

from .driver.ch347 import CH347
from .errors import (
    InterfacefIOError,
    InterfaceInitError,
    InterfaceIOTimeout,
    InterfaceNotFoundError,
)
from .manager import BaseInterfaceBuilder
from .templates import (
    GPIOInterfaceTemplate,
    GpioModes_T,
    I2CInterfaceTemplate,
    I2CMessageTemplate,
    SPIInterfaceTemplate,
    UARTInterfaceTemplate,
)
from .utils import FakeLock

_dev: Optional[CH347] = None  # device should be opened only once
_lock = FakeLock()


class CH347_I2CMessage(I2CMessageTemplate):
    def __init__(
        self,
        read: bool,
        data: Optional[bytes] = None,
        len: Optional[int] = None,
    ) -> None:
        self._data = data
        self._rd_len = len
        self._read = read

    @staticmethod
    def write(data) -> "CH347_I2CMessage":
        return CH347_I2CMessage(False, bytes(data))

    @staticmethod
    def read(length):
        return CH347_I2CMessage(True, None, length)

    def __len__(self):
        return len(self._data) if self._data else 0

    def __repr__(self):
        return f"CH347_I2CMessage( {self._read}, {self._data}, {self._rd_len})"

    def __iter__(self):
        return iter(self._data) if self._data else iter([])

    def __bytes__(self):
        return bytes(self._data) if self._data else bytes()


class CH347_I2CInterface(I2CInterfaceTemplate):
    def __init__(self, addr: int) -> None:
        self._addr = addr << 1
        return super().__init__()

    @property
    def address(self) -> int:
        return self._addr >> 1

    @address.setter
    def address(self, address: int):
        self._addr = address << 1

    def write_raw_byte(self, value: int):
        assert _dev is not None
        with _lock:
            if _dev.i2c_stream(bytes([self._addr, value]), 0) is None:
                raise InterfacefIOError("I2C write failed")

    def read_raw_byte(self) -> int:
        assert _dev is not None
        with _lock:
            ret = _dev.i2c_stream(bytes([self._addr]), 1)
            if ret is None:
                raise InterfacefIOError("I2C read failed")
            return ret[0]

    def write_reg_byte(self, register: int, value: int):
        assert _dev is not None
        with _lock:
            if _dev.i2c_stream(bytes([self._addr, register, value]), 0) is None:
                raise InterfacefIOError("I2C write failed")

    def read_reg_byte(self, register: int) -> int:
        assert _dev is not None
        with _lock:
            ret = _dev.i2c_stream(bytes([self._addr, register]), 1)
            if ret is None:
                raise InterfacefIOError("I2C read failed")
            return ret[0]

    def write_reg_data(self, register: int, data: Union[bytes, List[int]]):
        assert _dev is not None
        with _lock:
            if _dev.i2c_stream(bytes([self._addr, register]) + bytes(data), 0) is None:
                raise InterfacefIOError("I2C write failed")

    def read_reg_data(self, register: int, length: int) -> bytes:
        assert _dev is not None
        with _lock:
            ret = _dev.i2c_stream(bytes([self._addr, register]), length)
            if ret is None:
                raise InterfacefIOError("I2C read failed")
            return bytes(ret)

    @property
    def new_msg(self) -> type[CH347_I2CMessage]:
        return CH347_I2CMessage

    def check_address(self) -> bool:
        assert _dev is not None
        with _lock:
            return _dev.i2c_stream(bytes([self._addr, 0x00]), 1) is not None

    def exchange_msgs(self, msgs: list[CH347_I2CMessage]):
        assert _dev is not None
        with _lock:
            for msg in msgs:
                if msg._read:
                    assert msg._rd_len is not None
                    ret = _dev.i2c_stream(bytes([self._addr]), msg._rd_len)
                    if ret is None:
                        raise InterfacefIOError("I2C read failed")
                    msg._data = bytes(ret)
                else:
                    assert msg._data is not None
                    if _dev.i2c_stream(bytes([self._addr]) + msg._data, 1) is None:
                        raise InterfacefIOError("I2C write failed")


class CH347_I2CInterfaceBuilder(BaseInterfaceBuilder):
    def __init__(
        self,
        clock: Literal[20000, 50000, 100000, 200000, 400000, 750000, 1000000] = 400000,
        add_lock: bool = True,
    ) -> None:
        global _dev
        if _dev is None:
            _dev = CH347()
            if _dev.open_device() is None:
                raise InterfaceInitError("CH347 open failed")
            if add_lock:
                global _lock
                _lock = Lock()
        clock_dict: Dict[int, Literal[0, 1, 2, 3, 4, 5, 6]] = {
            20000: 0,
            50000: 4,
            100000: 1,
            200000: 5,
            400000: 2,
            750000: 3,
            1000000: 6,
        }
        _dev.i2c_set(clock_dict[clock])
        self.dev_type = "i2c"

    def build(self, address: int) -> CH347_I2CInterface:
        return CH347_I2CInterface(address)


class CH347_UARTInterface(UARTInterfaceTemplate):
    def __init__(self, uart_index: int, baudrate: int) -> None:
        self._idx = uart_index
        self._baudrate = baudrate
        self._bytesize: Literal[5, 6, 7, 8, 16] = 8
        self._parity: Literal[0, 1, 2, 3, 4] = 0
        self._stopbits: Literal[0, 1, 2] = 0
        self._timeout = None
        self._rbuf = bytearray()
        assert _dev is not None
        if not _dev.open_uart(self._idx):
            raise InterfaceInitError("CH347 open failed")
        self._reset()
        return super().__init__()

    def _reset(self):
        assert _dev is not None
        if not _dev.uart_init(
            uart_index=self._idx,
            baudrate=self._baudrate,
            bytesize=self._bytesize,
            parity=self._parity,
            stopbits=self._stopbits,
        ):
            raise InterfaceIOTimeout("UART Set params failed")

    @property
    def baudrate(self) -> int:
        return self._baudrate

    @baudrate.setter
    def baudrate(self, baudrate: int):
        self._baudrate = baudrate
        self._reset()

    @property
    def data_bits(self) -> int:
        return self._bytesize

    @data_bits.setter
    def data_bits(self, data_bits: Literal[5, 6, 7, 8, 16]):
        self._bytesize = data_bits
        self._reset()

    @property
    def stop_bits(self) -> str:
        _remap = {0: "1", 1: "1.5", 2: "2"}
        return _remap[self._stopbits]

    @stop_bits.setter
    def stop_bits(self, stop_bits: str):
        _remap = {"1": 0, "1.5": 1, "2": 2}
        self._stopbits = _remap[stop_bits]  # type: ignore
        self._reset()

    @property
    def parity(self) -> str:
        _remap = {0: "N", 1: "O", 2: "E", 3: "M", 4: "S"}
        return _remap[self._parity]

    @parity.setter
    def parity(self, parity: str):
        _remap = {"N": 0, "O": 1, "E": 2, "M": 3, "S": 4}
        self._parity = _remap[parity]  # type: ignore
        self._reset()

    @property
    def timeout(self) -> Optional[float]:
        return self._timeout

    @timeout.setter
    def timeout(self, timeout: Optional[float]):
        self._timeout = timeout

    def write(self, data: Union[bytes, List[int]]):
        assert _dev is not None
        with _lock:
            if not _dev.uart_write(self._idx, bytes(data)):
                raise InterfacefIOError("UART write failed")

    def _update_rbuf(self):
        assert _dev is not None
        ret = _dev.uart_read(self._idx, 1024)
        if ret is None:
            raise InterfacefIOError("UART read failed")
        self._rbuf.extend(ret)

    def read(self, length: int = 1) -> bytes:
        self._update_rbuf()
        if len(self._rbuf) < length and self._timeout is not None:
            start = time.perf_counter()
            while True:
                time.sleep(0.001)
                self._update_rbuf()
                if (
                    len(self._rbuf) >= length
                    or time.perf_counter() - start > self._timeout
                ):
                    break
        data = bytes(self._rbuf[:length])
        self._rbuf = self._rbuf[length:]
        return data

    def readline(self, length: int = -1) -> bytes:
        self._update_rbuf()
        start = time.perf_counter()
        while True:
            time.sleep(0.001)
            if (
                self._timeout is not None
                and time.perf_counter() - start > self._timeout
            ):
                return b""
            self._update_rbuf()
            fd = self._rbuf.find(b"\n")
            if fd != -1:
                data = bytes(self._rbuf[: fd + 1])
                self._rbuf = self._rbuf[fd + 1 :]
                return data
            if length > -1 and len(self._rbuf) >= length:
                data = bytes(self._rbuf[:length])
                self._rbuf = self._rbuf[length:]
                return data

    @property
    def in_waiting(self) -> int:
        self._update_rbuf()
        return len(self._rbuf)

    def close(self):
        assert _dev is not None
        _dev.close_uart(self._idx)

    def reopen(self):
        assert _dev is not None
        if not _dev.open_uart(self._idx):
            raise InterfaceInitError("CH347 open failed")
        self._reset()


class CH347_UARTInterfaceBuilder(BaseInterfaceBuilder):
    def __init__(self, uart_index: int = 0, add_lock: bool = True) -> None:
        global _dev
        if _dev is None:
            _dev = CH347()
            if add_lock:
                global _lock
                _lock = Lock()
        # check if uart_index is valid
        if _dev.open_uart(uart_index) is None:
            raise InterfaceInitError(f"CH347 open uart-{uart_index} failed")
        _dev.close_uart(uart_index)
        self._opened = True
        self._idx = uart_index
        self.dev_type = "uart"

    def build(self, baudrate: int) -> CH347_UARTInterface:
        return CH347_UARTInterface(self._idx, baudrate)


class CH347_SPIInterface(SPIInterfaceTemplate):
    def __init__(
        self,
        enable_cs: bool,
        cs: Literal[0, 1],
        cs_high: bool,
        auto_reset: bool,
        mode: int,
        speed_hz: int,
    ) -> None:
        self._cs = cs
        self._cs_high = cs_high  # active low
        self._enable_cs = enable_cs
        self._mode = mode
        self._auto_reset = auto_reset
        self._clock, self._speed = self._get_speed(speed_hz)
        self._byteorder = 1  # MSB first
        self._bytesize = 8
        self._reset()
        return super().__init__()

    def _reset(self, dummy: bool = True):
        assert _dev is not None
        if not _dev.spi_init(
            mode=self._mode,  # type: ignore
            clock=self._clock,  # type: ignore
            byte_order=self._byteorder,  # type: ignore
            write_read_interval=0,
            default_data=0,
            chip_select=1 << 7 | self._cs,
            cs1_polarity=0,
            cs2_polarity=0,
            is_auto_deactive_cs=1,
            active_delay=0,
            delay_deactive=0,
        ):
            raise InterfaceIOTimeout("SPI Set config failed")
        if self._enable_cs:
            self._cs_update()
        if dummy:
            self.transfer([0x00])  # dummy transfer

    def _cs_update(self):
        if self._enable_cs:
            if self._cs == 1:
                iEnableSelect = 1 << 8
                iChipSelect = 1 << 7
                iIsAutoDeativeCS = 1 << 0
            else:
                iEnableSelect = 1 << 0
                iChipSelect = 1 << 15
                iIsAutoDeativeCS = 1 << 8
        else:
            iEnableSelect = 0
            iIsAutoDeativeCS = 0
            iChipSelect = 0
        assert _dev is not None
        if not _dev.spi_set_chip_select(
            iEnableSelect, iChipSelect, iIsAutoDeativeCS, 0, 0
        ):
            raise InterfaceIOTimeout("SPI Set CS failed")
        _dev.spi_change_cs(0)

    def _get_speed(self, speed_hz: int):
        available_speed = [
            60_000_000,
            30_000_000,
            15_000_000,
            7_500_000,
            3_750_000,
            1_875_000,
            937_500,
            468_750,
        ]
        for i, s in enumerate(available_speed):
            if s <= speed_hz:
                return i, available_speed[
                    i
                ]  # return the highest speed that is lower than speed_hz
        return len(available_speed) - 1, available_speed[-1]  # return the lowest speed

    @property
    def mode(self) -> int:
        return self._mode

    @mode.setter
    def mode(self, mode: int):
        self._mode = mode
        self._reset()

    @property
    def speed_hz(self) -> int:
        return self._speed

    @speed_hz.setter
    def speed_hz(self, speed_hz: int):
        self._clock, self._speed = self._get_speed(speed_hz)
        self._reset()

    @property
    def byteorder(self) -> str:
        return "lsb" if self._byteorder == 0 else "msb"

    @byteorder.setter
    def byteorder(self, byteorder: str):
        self._byteorder = 0 if byteorder == "lsb" else 1
        self._reset()

    @property
    def bits_per_word(self) -> int:
        return self._bytesize

    @bits_per_word.setter
    def bits_per_word(self, bits_per_word: int):
        if bits_per_word not in [8, 16]:
            raise ValueError("bits_per_word should be 8 or 16")
        self._bytesize = bits_per_word
        assert _dev is not None
        _dev.spi_set_databits(0 if bits_per_word == 8 else 1)

    def _check(self):
        if self._auto_reset:
            self._reset(False)

    def write(self, data: Union[bytes, List[int]]):
        assert _dev is not None
        with _lock:
            cs = (0x80) if self._enable_cs else 0x00
            self._check()
            if not _dev.spi_write(cs, bytes(data)):
                # if _dev.spi_read(cs, bytes(data), 0) is None:
                raise InterfacefIOError("SPI write failed")

    def read(self, length: int) -> bytes:
        assert _dev is not None
        with _lock:
            cs = (0x80) if self._enable_cs else 0x00
            self._check()
            ret = _dev.spi_read(cs, b"", length)
            if ret is None:
                raise InterfacefIOError("SPI read failed")
            return bytes(ret)

    def transfer(self, data: Union[bytes, List[int]]) -> bytes:
        assert _dev is not None
        with _lock:
            cs = (0x80) if self._enable_cs else 0x00
            self._check()
            ret = _dev.spi_stream_write_read(cs, bytes(data))
            # ret = _dev.spi_write_read(cs, bytes(data))
            if ret is None:
                raise InterfacefIOError("SPI exchange failed")
            return bytes(ret)


class CH347_SPIInterfaceBuilder(BaseInterfaceBuilder):
    def __init__(
        self,
        enable_cs: bool = True,
        cs: Literal[0, 1] = 0,
        cs_high: bool = False,
        auto_reset: bool = False,
        add_lock: bool = True,
    ) -> None:
        global _dev
        if _dev is None:
            _dev = CH347()
            if add_lock:
                global _lock
                _lock = Lock()
            if not _dev.open_device():
                raise InterfaceInitError("CH347 open failed")
        self._cs: Literal[0, 1] = cs
        self._enable_cs = enable_cs
        self._cs_high = cs_high
        self._auto_reset = auto_reset
        self.dev_type = "spi"

    def build(self, mode: int, speed_hz: int) -> CH347_SPIInterface:
        return CH347_SPIInterface(
            self._enable_cs,
            self._cs,
            self._cs_high,
            self._auto_reset,
            mode,
            speed_hz,
        )


CH347AvailablePins = Literal[
    "GPIO0", "GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5", "GPIO6", "GPIO7"
]


def _SET_BIT(x, bit):
    return x | 1 << bit


def _CLEAR_BIT(x, bit):
    return x & ~(1 << bit)


class CH347_GPIOInterface(GPIOInterfaceTemplate):
    def __init__(self, pinmap: Optional[Dict[str, CH347AvailablePins]]) -> None:
        self._pinmap = pinmap if pinmap is not None else {}
        self._pinmap_inv = {v: k for k, v in self._pinmap.items()}
        self._gpio_enable = 0
        self._gpio_dir = 0
        self._gpio_out = 0
        self._pinmodes: Dict[str, GpioModes_T] = {}
        return super().__init__()

    @lru_cache(64)
    def _remap(self, pin_name: str) -> str:
        pin_name = self._pinmap.get(pin_name, pin_name)
        if pin_name not in [f"GPIO{i}" for i in range(8)]:
            raise InterfaceNotFoundError(f"Pin {pin_name} not found")
        return pin_name

    def get_available_pins(self) -> Dict[str, List[GpioModes_T]]:
        lst: Dict[str, List[GpioModes_T]] = {
            f"GPIO{i}": [
                "input_no_pull",
                "output_push_pull",
            ]
            for i in range(8)
        }
        return {self._pinmap_inv.get(k, k): v for k, v in lst.items()}

    def set_mode(self, pin_name: str, mode: GpioModes_T):
        assert _dev is not None
        pin_name = self._remap(pin_name)
        offset = int(pin_name[-1])
        if mode == "none":
            self._gpio_enable = _CLEAR_BIT(self._gpio_enable, offset)
        else:
            self._gpio_enable = _SET_BIT(self._gpio_enable, offset)
            if mode == "input_no_pull":
                self._gpio_dir = _CLEAR_BIT(self._gpio_dir, offset)
            elif mode == "output_push_pull":
                self._gpio_dir = _SET_BIT(self._gpio_dir, offset)
            else:
                raise ValueError(f"Invalid GPIO mode {mode}")
            self._gpio_out = _CLEAR_BIT(self._gpio_out, offset)
        with _lock:
            if not _dev.gpio_set(self._gpio_enable, self._gpio_dir, self._gpio_out):
                raise InterfacefIOError("GPIO set config failed")
        self._pinmodes[pin_name] = mode

    def get_mode(self, pin_name: str) -> GpioModes_T:
        pin_name = self._remap(pin_name)
        return self._pinmodes.get(pin_name, "none")

    def write(self, pin_name: str, level: bool):
        assert _dev is not None
        pin_name = self._remap(pin_name)
        offset = int(pin_name[-1])
        if level:
            self._gpio_out = _SET_BIT(self._gpio_out, offset)
        else:
            self._gpio_out = _CLEAR_BIT(self._gpio_out, offset)
        with _lock:
            if not _dev.gpio_set(self._gpio_enable, self._gpio_dir, self._gpio_out):
                raise InterfacefIOError("GPIO write failed")

    def read(self, pin_name: str) -> bool:
        assert _dev is not None
        pin_name = self._remap(pin_name)
        offset = int(pin_name[-1])
        with _lock:
            ret = _dev.gpio_get()
            if ret is None:
                raise InterfacefIOError("GPIO read failed")
        _, rd = ret
        return bool(rd & (1 << offset))

    def close(self):
        if _dev is not None:
            _dev.gpio_set(0, 0, 0)

    def free(self, pin_name: str):
        self.set_mode(pin_name, "none")


class CH347_GPIOInterfaceBuilder(BaseInterfaceBuilder):
    def __init__(
        self,
        pinmap: Optional[Dict[str, CH347AvailablePins]] = None,
        add_lock: bool = True,
    ) -> None:
        global _dev
        if _dev is None:
            _dev = CH347()
            if not _dev.open_device():
                raise InterfaceInitError("CH347 open failed")
            if add_lock:
                global _lock
                _lock = Lock()
        self._pinmap = pinmap
        self.dev_type = "gpio"

    def build(self) -> CH347_GPIOInterface:
        return CH347_GPIOInterface(self._pinmap)
