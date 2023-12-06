from functools import lru_cache
from typing import Dict, List, Literal, Optional

from .driver.pcal6416a import PCAL6416A
from .errors import InterfaceNotFoundError
from .manager import BaseInterfaceBuilder
from .templates import GPIOInterfaceTemplate, GpioModes_T

PCAL6416AAvailablePins = Literal[
    "P00", "P01", "P02", "P03", "P04", "P05", "P06", "P07",
    "P10", "P11", "P12", "P13", "P14", "P15", "P16", "P17",
]  # fmt: off


class PCAL6416A_GPIOInterface(GPIOInterfaceTemplate):
    def __init__(
        self,
        pinmap: Optional[Dict[str, PCAL6416AAvailablePins]],
        address: int = 0x21,
        cache_reg: bool = True,
    ) -> None:
        self._io = PCAL6416A(address, cache_reg)
        self._pinmap = pinmap if pinmap is not None else {}
        self._pinmap_inv = {v: k for k, v in self._pinmap.items()}
        self._pinmodes: Dict[str, GpioModes_T] = {}
        super().__init__()

    @lru_cache(64)
    def _remap(self, pin_name: str) -> tuple[str, int, int]:
        pin_name = self._pinmap.get(pin_name, pin_name)
        if pin_name not in [f"P{port}{pin}" for port in range(2) for pin in range(8)]:
            raise InterfaceNotFoundError(f"Pin {pin_name} not found")
        return pin_name, int(pin_name[1]), int(pin_name[2])

    def get_available_pins(self) -> Dict[str, List[GpioModes_T]]:
        lst: Dict[str, List[GpioModes_T]] = {
            f"P{port}{pin}": [
                "input_no_pull",
                "input_pull_down",
                "input_pull_up",
                "output_push_pull",
                "output_open_drain",
            ]
            for port in range(2)
            for pin in range(8)
        }
        return {self._pinmap_inv.get(k, k): v for k, v in lst.items()}

    def set_mode(self, pin_name: str, mode: GpioModes_T):
        pin_name, port, pin = self._remap(pin_name)
        if mode.startswith("input"):
            self._io.set_direction(port, pin, True)
            if mode == "input_pull_up":
                self._io.set_pull_mode(port, pin, True, True)
            elif mode == "input_pull_down":
                self._io.set_pull_mode(port, pin, True, False)
            elif mode == "input_no_pull":
                self._io.set_pull_mode(port, pin, False, False)
            else:
                raise ValueError(f"Invalid mode {mode}")
        elif mode.startswith("output"):
            self._io.set_direction(port, pin, False)
            self._io.set_pull_mode(port, pin, False, False)
            if mode == "output_push_pull":
                self._io.set_output_opendrain(port, False)
            elif mode == "output_open_drain":
                self._io.set_output_opendrain(port, True)
            else:
                raise ValueError(f"Invalid mode {mode}")
        else:
            raise ValueError(f"Invalid mode {mode}")
        self._pinmodes[pin_name] = mode

    def get_mode(self, pin_name: str) -> GpioModes_T:
        pin_name, port, pin = self._remap(pin_name)
        return self._pinmodes.get(pin_name, "none")

    def write(self, pin_name: str, level: bool):
        pin_name, port, pin = self._remap(pin_name)
        self._io.write(port, pin, level)

    def read(self, pin_name: str) -> bool:
        pin_name, port, pin = self._remap(pin_name)
        return self._io.read(port, pin)

    def close(self):
        self._io.write_port(0, 0xFF)
        self._io.write_port(1, 0xFF)

    def free(self, pin_name: str):
        pin_name, port, pin = self._remap(pin_name)
        self._io.set_direction(port, pin, False)


class PCAL6416A_GPIOInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "gpio"

    def __init__(
        self,
        pinmap: Optional[Dict[str, PCAL6416AAvailablePins]] = None,
        address: int = 0x21,
        cache_reg: bool = True,
    ) -> None:
        self._pinmap = pinmap
        self._address = address
        self._cache_reg = cache_reg

    def build(self) -> PCAL6416A_GPIOInterface:
        return PCAL6416A_GPIOInterface(self._pinmap, self._address, self._cache_reg)
