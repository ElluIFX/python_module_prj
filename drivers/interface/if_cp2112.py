from functools import cached_property, lru_cache
from threading import Lock
from typing import Dict, List, Literal, Optional, Tuple, Union

from drivers.cp2112 import CP2112

from .manager import (
    GPIOInterfaceTemplate,
    I2CInterfaceTemplate,
    I2CMessageTemplate,
    InterfaceBuilderTemplate,
)


class _FakeLock:
    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


_dev: Optional[CP2112] = None  # hid device should be opened only once
_lock = _FakeLock()


def bus_scan():
    assert _dev is not None
    return _dev.bus_scan()


class CP2112_I2CMessage(I2CMessageTemplate):
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
    def write(data) -> "CP2112_I2CMessage":
        return CP2112_I2CMessage(False, bytes(data))

    @staticmethod
    def read(length):
        return CP2112_I2CMessage(True, None, length)

    def __len__(self):
        return len(self._data) if self._data else 0

    def __repr__(self):
        return f"CP2112_I2CMessage( {self._read}, {self._data}, {self._rd_len})"

    def __iter__(self):
        return iter(self._data) if self._data else iter([])

    def __bytes__(self):
        return bytes(self._data) if self._data else bytes()


class CP2112_I2CInterface(I2CInterfaceTemplate):
    def __init__(self, addr: int) -> None:
        self._addr = addr

    @property
    def address(self) -> int:
        return self._addr

    @address.setter
    def address(self, address: int):
        assert address != 0x00, "Address 0x00 is not supported by CP2112"
        self._addr = address

    def write_raw_byte(self, value: int):
        assert _dev is not None
        with _lock:
            _dev.write_i2c(self._addr, [value])

    def read_raw_byte(self) -> int:
        assert _dev is not None
        with _lock:
            return _dev.read_i2c(self._addr, 1)[0]

    def write_reg_byte(self, register: int, value: int):
        assert _dev is not None
        with _lock:
            _dev.write_i2c(self._addr, [register, value])

    def read_reg_byte(self, register: int) -> int:
        assert _dev is not None
        with _lock:
            return _dev.write_read_i2c(self._addr, [register], 1)[0]

    def write_reg_data(self, register: int, data: Union[bytes, List[int]]):
        assert _dev is not None
        with _lock:
            _dev.write_i2c(self._addr, bytes([register]) + bytes(data))

    @cached_property
    def max_transfer_size(self) -> int:
        return 60  # 61 - addr

    def read_reg_data(self, register: int, length: int) -> bytes:
        assert _dev is not None
        with _lock:
            return bytes(_dev.write_read_i2c(self._addr, [register], length))

    def new_msg(self) -> type[CP2112_I2CMessage]:
        return CP2112_I2CMessage

    def check_address(self) -> bool:
        assert _dev is not None
        with _lock:
            return _dev.check_i2c_device(self._addr)

    def transfer_msg(self, msgs: list[CP2112_I2CMessage]):
        assert _dev is not None
        with _lock:
            for msg in msgs:
                if msg._read:
                    msg._data = bytes(_dev.read_i2c(self._addr, msg._rd_len))
                else:
                    _dev.write_i2c(self._addr, msg._data)


class CP2112_I2CInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, clock=400000, retry=3, txrx_leds=True, add_lock=True) -> None:
        global _dev
        if _dev is None:
            _dev = CP2112(clock=clock, retry=retry, txrx_leds=txrx_leds)
            if add_lock:
                global _lock
                _lock = Lock()
        self.dev_type = "i2c"

    def build(self, address: int) -> CP2112_I2CInterface:
        return CP2112_I2CInterface(address)


def _SET_BIT(x, bit):
    return x | 1 << bit


def _CLEAR_BIT(x, bit):
    return x & ~(1 << bit)


class CP2112_GPIOInterface(GPIOInterfaceTemplate):
    GPIOModes = GPIOInterfaceTemplate.GPIOModes
    AvailableGPIOs = Literal[
        "Pin_0", "Pin_1", "Pin_2", "Pin_3", "Pin_4", "Pin_5", "Pin_6", "Pin_7"
    ]

    def __init__(self, pinmap: Optional[Dict[str, AvailableGPIOs]]) -> None:
        self._pinmap = pinmap

    @lru_cache(64)
    def _remap(self, pin_name: str) -> str:
        if self._pinmap is not None:
            pin_name = self._pinmap.get(pin_name, pin_name)
        return pin_name

    def get_available_pins(self) -> List[Tuple[str, List[GPIOModes]]]:
        return [
            (
                "Pin_0",
                [
                    "input_no_pull",
                    "output_open_drain",
                    "output_push_pull",
                    "special_func",
                ],
            ),
            (
                "Pin_1",
                [
                    "input_no_pull",
                    "output_open_drain",
                    "output_push_pull",
                    "special_func",
                ],
            ),
            ("Pin_2", ["input_no_pull", "output_open_drain", "output_push_pull"]),
            ("Pin_3", ["input_no_pull", "output_open_drain", "output_push_pull"]),
            ("Pin_4", ["input_no_pull", "output_open_drain", "output_push_pull"]),
            ("Pin_5", ["input_no_pull", "output_open_drain", "output_push_pull"]),
            ("Pin_6", ["input_no_pull", "output_open_drain", "output_push_pull"]),
            (
                "Pin_7",
                [
                    "input_no_pull",
                    "output_open_drain",
                    "output_push_pull",
                    "output_pwm",
                ],
            ),
        ]

    def set_mode(self, pin_name: str, mode: GPIOModes):
        assert _dev is not None
        pin_name = self._remap(pin_name)
        offset = int(pin_name[-1])
        with _lock:
            dir, push_pull, special, clock_divider = _dev.get_gpio_config()
        if mode == "output_pwm" and pin_name == "Pin_7":
            special = _SET_BIT(special, 0)
            dir = _SET_BIT(dir, offset)
            push_pull = _SET_BIT(push_pull, offset)
        elif mode == "special_func" and pin_name in ("Pin_0", "Pin_1"):
            special = _SET_BIT(special, offset + 1)
            dir = _SET_BIT(dir, offset)
            push_pull = _SET_BIT(push_pull, offset)
        elif mode in ("input_no_pull", "output_open_drain", "output_push_pull"):
            if pin_name == "Pin_7":
                special = _CLEAR_BIT(special, 0)
            elif pin_name in ("Pin_0", "Pin_1"):
                special = _CLEAR_BIT(special, offset + 1)
            if mode.startswith("input"):
                dir = _CLEAR_BIT(dir, offset)
            else:
                dir = _SET_BIT(dir, offset)
                if mode == "output_open_drain":
                    push_pull = _CLEAR_BIT(push_pull, offset)
                else:
                    push_pull = _SET_BIT(push_pull, offset)
        else:
            raise ValueError(f"Invalid mode {mode} for pin {pin_name}")
        with _lock:
            _dev.set_gpio_config(dir, push_pull, special, clock_divider)

    def write_pwm_freq(self, pin_name: str, freq: int):
        assert _dev is not None
        pin_name = self._remap(pin_name)
        assert pin_name == "Pin_7", "Only Pin_7 supports PWM"
        # 0=48 MHz; Otherwise freq=(48 MHz)/(2*clock_divider)
        if freq >= 48000000:
            clock_divider = 0
        else:
            clock_divider = 48000000 // (2 * freq)
        with _lock:
            dir, push_pull, special, _ = _dev.get_gpio_config()
            special = _SET_BIT(special, 0)
            dir = _SET_BIT(dir, 7)
            push_pull = _SET_BIT(push_pull, 7)
            _dev.set_gpio_config(dir, push_pull, special, clock_divider)

    def write(self, pin_name: str, value: bool):
        assert _dev is not None
        pin_name = self._remap(pin_name)
        offset = int(pin_name[-1])
        with _lock:
            _dev.set_pin(offset, 1 if value else 0)

    def read(self, pin_name: str) -> bool:
        assert _dev is not None
        pin_name = self._remap(pin_name)
        offset = int(pin_name[-1])
        with _lock:
            return _dev.get_pin(offset)


class CP2112_GPIOInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(
        self, pinmap: Optional[Dict[str, CP2112_GPIOInterface.AvailableGPIOs]] = None
    ) -> None:
        global _dev
        if _dev is None:
            _dev = CP2112()
        self.dev_type = "gpio"
        self._pinmap = pinmap

    def build(self) -> CP2112_GPIOInterface:
        return CP2112_GPIOInterface(self._pinmap)
