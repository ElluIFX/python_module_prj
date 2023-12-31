from functools import cached_property, lru_cache
from threading import Lock
from typing import Dict, List, Literal, Optional, Union

from ..drivers.cp2112 import CP2112, _reset_error_type
from ..errortype import InterfacefIOError, InterfaceNotFoundError
from ..manager import BaseInterfaceBuilder
from ..template import (
    BaseInterfaceTemplate,
    GPIOInterfaceTemplate,
    GpioModes_T,
    I2CInterfaceTemplate,
    I2CMessageTemplate,
)

_dev: Optional[CP2112] = None  # hid device should be opened only once
_lock = Lock()
_shared_count = 0
_reset_error_type(InterfacefIOError)


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
        return super().__init__()

    @property
    def address(self) -> int:
        return self._addr

    @address.setter
    def address(self, address: int):
        if address == 0x00:
            raise InterfacefIOError("Address 0x00 is not supported by CP2112")
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

    @property
    def new_msg(self) -> type[CP2112_I2CMessage]:
        return CP2112_I2CMessage

    def check_address(self) -> bool:
        assert _dev is not None
        with _lock:
            return _dev.check_i2c_device(self._addr)

    def exchange_msgs(self, msgs: list[CP2112_I2CMessage]):
        assert _dev is not None
        with _lock:
            for msg in msgs:
                if msg._read and msg._rd_len is not None:
                    msg._data = bytes(_dev.read_i2c(self._addr, msg._rd_len))
                elif not msg._read and msg._data is not None:
                    _dev.write_i2c(self._addr, msg._data)
                else:
                    raise ValueError(f"Invalid message: {msg}")


class CP2112_I2CInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "i2c"

    def __init__(
        self,
        clock: int = 400000,
        retry: int = 3,
        txrx_leds: bool = True,
    ) -> None:
        self._clock = clock
        self._retry = retry
        self._txrx_leds = txrx_leds
        super().__init__()

    def build(self, address: int) -> CP2112_I2CInterface:
        global _dev, _shared_count
        _dev = CP2112(clock=self._clock, retry=self._retry, txrx_leds=self._txrx_leds)
        _shared_count += 1
        return CP2112_I2CInterface(address)

    def destroy(self, instance: BaseInterfaceTemplate):
        global _dev, _shared_count
        _shared_count -= 1
        if _shared_count == 0 and _dev is not None:
            _dev.close()
            _dev = None


def _SET_BIT(x, bit):
    return x | 1 << bit


def _CLEAR_BIT(x, bit):
    return x & ~(1 << bit)


CP2112AvailablePins = Literal[
    "Pin_0", "Pin_1", "Pin_2", "Pin_3", "Pin_4", "Pin_5", "Pin_6", "Pin_7"
]


class CP2112_GPIOInterface(GPIOInterfaceTemplate):
    def __init__(self, pinmap: Optional[Dict[str, CP2112AvailablePins]]) -> None:
        self._pinmap = pinmap if pinmap is not None else {}
        self._pinmap_inv = {v: k for k, v in self._pinmap.items()}
        self._pinmodes: Dict[str, GpioModes_T] = {}
        return super().__init__()

    @lru_cache(64)
    def _remap(self, pin_name: str) -> str:
        pin_name = self._pinmap.get(pin_name, pin_name)
        if pin_name not in [f"Pin_{i}" for i in range(8)]:
            raise InterfaceNotFoundError(f"Pin {pin_name} not found")
        return pin_name

    def get_available_pinmodes(self) -> Dict[str, List[GpioModes_T]]:
        lst: Dict[str, List[GpioModes_T]] = {
            "Pin_0": [
                "none",
                "input_no_pull",
                "output_open_drain",
                "output_push_pull",
                "special_func",
            ],
            "Pin_1": [
                "none",
                "input_no_pull",
                "output_open_drain",
                "output_push_pull",
                "special_func",
            ],
            "Pin_2": ["none", "input_no_pull", "output_open_drain", "output_push_pull"],
            "Pin_3": ["none", "input_no_pull", "output_open_drain", "output_push_pull"],
            "Pin_4": ["none", "input_no_pull", "output_open_drain", "output_push_pull"],
            "Pin_5": ["none", "input_no_pull", "output_open_drain", "output_push_pull"],
            "Pin_6": ["none", "input_no_pull", "output_open_drain", "output_push_pull"],
            "Pin_7": [
                "none",
                "input_no_pull",
                "output_open_drain",
                "output_push_pull",
                "pwm_output",
            ],
        }
        return {self._pinmap_inv.get(k, k): v for k, v in lst.items()}

    def set_mode(self, pin_name: str, mode: GpioModes_T):
        assert _dev is not None
        pin_name = self._remap(pin_name)
        offset = int(pin_name[-1])
        with _lock:
            dir, push_pull, special, clock_divider = _dev.get_gpio_config()
        if mode == "pwm_output" and pin_name == "Pin_7":
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
        self._pinmodes[pin_name] = mode

    def get_mode(self, pin_name: str) -> GpioModes_T:
        pin_name = self._remap(pin_name)
        return self._pinmodes.get(pin_name, "none")

    def write_pwm_freq(self, pin_name: str, freq: int):
        assert _dev is not None
        pin_name = self._remap(pin_name)
        assert pin_name == "Pin_7", "Only Pin_7 supports PWM"
        # 0=48 MHz; Otherwise freq=(48 MHz)/(2*clock_divider)
        if freq >= 48000000:
            clock_divider = 0
        else:
            clock_divider = 48000000 // (2 * freq)
        clock_divider = min(255, clock_divider)
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


class CP2112_GPIOInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "gpio"

    def __init__(
        self,
        pinmap: Optional[Dict[str, CP2112AvailablePins]] = None,
    ) -> None:
        self._pinmap = pinmap
        super().__init__()

    def build(self) -> CP2112_GPIOInterface:
        global _dev, _shared_count
        if _dev is None:
            _dev = CP2112(txrx_leds=False)
        _shared_count += 1
        return CP2112_GPIOInterface(self._pinmap)

    def destroy(self, instance: BaseInterfaceTemplate):
        global _dev, _shared_count
        _shared_count -= 1
        if _shared_count == 0 and _dev is not None:
            _dev.close()
            _dev = None
