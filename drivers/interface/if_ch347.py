from threading import Lock
from typing import Dict, List, Literal, Optional, Union

from .driver.ch347 import CH347
from .manager import (
    I2CInterfaceTemplate,
    I2CMessageTemplate,
    InterfaceBuilderTemplate,
    SPIInterfaceTemplate,
    UARTInterfaceTemplate,
)
from .utils import FakeLock, InterfacefIOError, InterfaceInitError, InterfaceIOTimeout

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


class CH347_I2CInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(
        self,
        clock: Literal[20000, 50000, 100000, 200000, 400000, 750000, 1000000] = 400000,
        add_lock=True,
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
        self._timeout = 0xFFFFFFFF
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
            bytetimout=self._timeout,
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
        return self._timeout / 1000 if self._timeout != 0xFFFFFFFF else None

    @timeout.setter
    def timeout(self, timeout: Optional[float]):
        if timeout is None:
            self._timeout = 0xFFFFFFFF
        else:
            self._timeout = int(timeout * 1000)
        self._reset()

    def write(self, data: Union[bytes, List[int]]):
        assert _dev is not None
        with _lock:
            if not _dev.uart_write(self._idx, bytes(data)):
                raise InterfacefIOError("UART write failed")

    def read(self, length: int) -> bytes:
        assert _dev is not None
        with _lock:
            ret = _dev.uart_read(self._idx, length)
            if ret is None:
                raise InterfacefIOError("UART read failed")
            return bytes(ret)

    @property
    def in_waiting(self) -> int:
        assert _dev is not None
        with _lock:
            ret = _dev.uart_in_waiting(self._idx)
            if ret is None:
                raise InterfacefIOError("UART query in-waiting failed")
            return ret

    def close(self):
        assert _dev is not None
        _dev.close_uart(self._idx)

    def reopen(self):
        assert _dev is not None
        if not _dev.open_uart(self._idx):
            raise InterfaceInitError("CH347 open failed")
        self._reset()


class CH347_UARTInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, uart_index: int = 0, add_lock=True) -> None:
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
        auto_cs: bool,
        cs: Literal[1, 2],
        cs_polarity: bool,
        auto_reset: bool,
        mode: int,
        speed_hz: int,
    ) -> None:
        self._cs = cs
        self._cs_pol = cs_polarity  # active low
        self._auto_cs = auto_cs
        self._mode = mode
        self._auto_reset = auto_reset
        self._clock, self._speed = self._get_speed(speed_hz)
        self._byteorder = 1  # MSB first
        self._bytesize = 8
        self._reset()
        self._cs_update()
        self.transfer([0x00])  # dummy transfer
        return super().__init__()

    def _reset(self):
        assert _dev is not None
        spi_config = _dev.spi_get_cfg()
        if spi_config is None:
            raise InterfaceIOTimeout("SPI Get config failed")
        spi_config.Mode = self._mode
        spi_config.Clock = self._clock
        spi_config.ByteOrder = self._byteorder
        if self._cs == 1:
            spi_config.CS1Polarity = self._cs_pol
        else:
            spi_config.CS2Polarity = self._cs_pol
        spi_config.IsAutoDeativeCS = 1
        spi_config.ChipSelect = 0xFF
        if not _dev.spi_init_with_config(spi_config):
            raise InterfaceIOTimeout("SPI Set config failed")

    def _cs_update(self):
        if self._auto_cs:
            if self._cs == 2:
                iEnableSelect = 1 << 8
                iIsAutoDeativeCS = 1 << 0
                iChipSelect = 1 << 0
            else:
                iEnableSelect = 1 << 0
                iIsAutoDeativeCS = 1 << 8
                iChipSelect = 1 << 8
        else:
            iEnableSelect = 0
            iIsAutoDeativeCS = 0
            iChipSelect = 0
        assert _dev is not None
        if not _dev.spi_set_chip_select(
            iEnableSelect, iChipSelect, iIsAutoDeativeCS, 0, 0
        ):
            raise InterfaceIOTimeout("SPI Set CS failed")

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
            self._reset()
        if self._auto_cs:
            self._cs_update()

    def write(self, data: Union[bytes, List[int]]):
        assert _dev is not None
        with _lock:
            cs = 0x80 if self._auto_cs else 0x00
            self._check()
            if not _dev.spi_write(cs, bytes(data)):
                raise InterfacefIOError("SPI write failed")

    def read(self, length: int) -> bytes:
        assert _dev is not None
        with _lock:
            cs = 0x80 if self._auto_cs else 0x00
            self._check()
            ret = _dev.spi_read(cs, b"", length)
            if ret is None:
                raise InterfacefIOError("SPI read failed")
            return bytes(ret)

    def transfer(self, data: Union[bytes, List[int]]) -> bytes:
        assert _dev is not None
        with _lock:
            cs = 0x80 if self._auto_cs else 0x00
            self._check()
            ret = _dev.spi_stream_write_read(cs, bytes(data))
            # ret = _dev.spi_write_read(cs, bytes(data))
            if ret is None:
                raise InterfacefIOError("SPI exchange failed")
            return bytes(ret)


class CH347_SPIInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(
        self,
        auto_cs: bool = False,
        cs: Literal[1, 2] = 1,
        cs_polarity: bool = False,
        auto_reset: bool = False,
        add_lock=True,
    ) -> None:
        global _dev
        if _dev is None:
            _dev = CH347()
            if add_lock:
                global _lock
                _lock = Lock()
            if not _dev.open_device():
                raise InterfaceInitError("CH347 open failed")
        self._cs: Literal[1, 2] = cs
        self._auto_cs = auto_cs
        self._cs_polarity = cs_polarity
        self._auto_reset = auto_reset
        self.dev_type = "spi"

    def build(self, mode: int, speed_hz: int) -> CH347_SPIInterface:
        return CH347_SPIInterface(
            self._auto_cs,
            self._cs,
            self._cs_polarity,
            self._auto_reset,
            mode,
            speed_hz,
        )
