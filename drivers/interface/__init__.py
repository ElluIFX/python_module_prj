"""
# Universal interface module for hardware drivers

Spliting hardware implementation from the software logical driver.

## Usage for users:

### First, register the interface depends on the hardware

```python
from drivers.interface import register_interface

# For example, you are using a CP2112 USB-I2C bridge:
# For all i2c driver:
register_interface("CP2112", "i2c", clock=400000)
# Or, just for one driver:
register_interface("CP2112", "i2c", clock=400000, specific_module=["aht20",])

```

### Then, use your driver

```python
from drivers.aht20 import AHT20

aht20 = AHT20()
aht20.init()
aht20.measure()
```python

## Usage for writing a new driver:

```python
from drivers.interface import request_interface

class MyNewDriver:

    def __init__(self, addr: int = 0x23):
        self._i2c = request_interface("i2c", "DriverName", address=addr)
        ...

    def read(self, register: int, length: int) -> bytes:
        return self._i2c.read_reg_data(register, length)

    ...
```

"""
from .errortype import (  # noqa: F401
    InterfacefIOError,
    InterfaceInitError,
    InterfaceIOTimeout,
    InterfaceNotFoundError,
)
from .helper import register_interface, request_interface  # noqa: F401
from .manager import InterfaceManager  # noqa: F401
