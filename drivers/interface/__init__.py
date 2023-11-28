"""
About the Interface module:

This module is used to manage the hardware interface of the device, for spliting
hardware implementation from the software driver.

Usage:

from drivers.interface import register_interface

# First, register the interface depends on the hardware

register_interface("CP2112", "i2c")

# Then, use your driver

from drivers.aht20 import AHT20

aht20 = AHT20()
aht20.init()
aht20.measure()
...

Usage for writing a new driver:

from drivers.interface import request_interface

class MyNewDriver:

    def __init__(self, addr: int) -> None:
        self._i2c = request_interface("i2c", "DriverName", address=addr)
        ...
"""
from .errors import (  # noqa: F401
    InterfacefIOError,
    InterfaceInitError,
    InterfaceIOTimeout,
    InterfaceNotFoundError,
)
from .helper import register_interface, request_interface  # noqa: F401
from .manager import InterfaceManager  # noqa: F401
