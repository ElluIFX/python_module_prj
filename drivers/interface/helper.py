from typing import TYPE_CHECKING, Dict, Literal, Optional, Union, overload

from .manager import InterfaceManager
from .templates import (
    GPIOInterfaceTemplate,
    I2CInterfaceTemplate,
    SPIInterfaceTemplate,
    UARTInterfaceTemplate,
)

if TYPE_CHECKING:
    from .if_ch347 import (
        CH347_GPIOInterfaceBuilder,
        CH347_I2CInterfaceBuilder,
        CH347_SPIInterfaceBuilder,
        CH347_UARTInterfaceBuilder,
    )
    from .if_cp2112 import (
        CP2112_GPIOInterfaceBuilder,
        CP2112_I2CInterfaceBuilder,
        CP2112AvailablePins,
    )
    from .if_periphery import (
        Periphery_GPIOInterfaceBuilder,
        Periphery_I2CInterfaceBuilder,
        Periphery_SPIInterfaceBuilder,
        Periphery_UARTInterfaceBuilder,
    )
    from .if_pyserial import PySerial_UARTInterfaceBuilder
    from .if_smbus2 import SMBus2_I2CInterfaceBuilder
    from .if_spidev import Spidev_SPIInterfaceBuilder
    from .templates import GpioModes_T


@overload
def register_interface(
    driver_name: Literal["cp2112"],
    driver_type: Literal["i2c"],
    clock=400000,
    retry=3,
    txrx_leds=True,
    add_lock=True,
    **xargs,
) -> "CP2112_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["cp2112"],
    driver_type: Literal["gpio"],
    pinmap: Optional[Dict[str, "CP2112AvailablePins"]] = None,
    add_lock=True,
    **xargs,
) -> "CP2112_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    driver_type: Literal["i2c"],
    clock: Literal[20000, 50000, 100000, 200000, 400000, 750000, 1000000] = 400000,
    add_lock=True,
    **xargs,
) -> "CH347_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    driver_type: Literal["uart"],
    uart_index: int,
    add_lock=True,
    **xargs,
) -> "CH347_UARTInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    driver_type: Literal["spi"],
    auto_cs: bool = False,
    cs: Literal[1, 2] = 1,
    cs_polarity: bool = False,
    auto_reset: bool = False,
    add_lock=True,
    **xargs,
) -> "CH347_SPIInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    driver_type: Literal["gpio"],
    pinmap: Optional[Dict[str, str]] = None,
    add_lock=True,
    **xargs,
) -> "CH347_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["i2c"],
    devpath: str,
    keep_alive: bool = False,
    **xargs,
) -> "Periphery_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["spi"],
    devpath: str,
    **xargs,
) -> "Periphery_SPIInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["uart"],
    devpath: str,
    **xargs,
) -> "Periphery_UARTInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["gpio"],
    pinmap: Optional[Dict[str, str]] = None,
    modemap: Optional[Dict[str, "GpioModes_T"]] = None,
    **xargs,
) -> "Periphery_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["pyserial"],
    driver_type: Literal["uart"],
    port: str,
    **xargs,
) -> "PySerial_UARTInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["smbus2"],
    driver_type: Literal["i2c"],
    bus: Union[int, str],
    **xargs,
) -> "SMBus2_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["spidev"],
    driver_type: Literal["spi"],
    channel: int,
    port: int,
    **xargs,
) -> "Spidev_SPIInterfaceBuilder":
    ...


def register_interface(
    driver_name,
    driver_type,
    *args,
    **xargs,
):
    module_name = xargs.pop("module_name", None)
    if driver_name == "cp2112":
        from .if_cp2112 import CP2112_GPIOInterfaceBuilder, CP2112_I2CInterfaceBuilder

        if driver_type == "i2c":
            return CP2112_I2CInterfaceBuilder(*args, **xargs).register(module_name)
        elif driver_type == "gpio":
            return CP2112_GPIOInterfaceBuilder(*args, **xargs).register(module_name)
    elif driver_name == "ch347":
        from .if_ch347 import (
            CH347_GPIOInterfaceBuilder,
            CH347_I2CInterfaceBuilder,
            CH347_SPIInterfaceBuilder,
            CH347_UARTInterfaceBuilder,
        )

        if driver_type == "i2c":
            return CH347_I2CInterfaceBuilder(*args, **xargs).register(module_name)
        elif driver_type == "uart":
            return CH347_UARTInterfaceBuilder(*args, **xargs).register(module_name)
        elif driver_type == "spi":
            return CH347_SPIInterfaceBuilder(*args, **xargs).register(module_name)
        elif driver_type == "gpio":
            return CH347_GPIOInterfaceBuilder(*args, **xargs).register(module_name)
    elif driver_name == "periphery":
        from .if_periphery import (
            Periphery_GPIOInterfaceBuilder,
            Periphery_I2CInterfaceBuilder,
            Periphery_SPIInterfaceBuilder,
            Periphery_UARTInterfaceBuilder,
        )

        if driver_type == "i2c":
            return Periphery_I2CInterfaceBuilder(*args, **xargs).register(module_name)
        elif driver_type == "spi":
            return Periphery_SPIInterfaceBuilder(*args, **xargs).register(module_name)
        elif driver_type == "uart":
            return Periphery_UARTInterfaceBuilder(*args, **xargs).register(module_name)
        elif driver_type == "gpio":
            return Periphery_GPIOInterfaceBuilder(*args, **xargs).register(module_name)
    elif driver_name == "pyserial":
        from .if_pyserial import PySerial_UARTInterfaceBuilder

        if driver_type == "uart":
            return PySerial_UARTInterfaceBuilder(*args, **xargs).register(module_name)
    elif driver_name == "smbus2":
        from .if_smbus2 import SMBus2_I2CInterfaceBuilder

        if driver_type == "i2c":
            return SMBus2_I2CInterfaceBuilder(*args, **xargs).register(module_name)
    elif driver_name == "spidev":
        from .if_spidev import Spidev_SPIInterfaceBuilder

        if driver_type == "spi":
            return Spidev_SPIInterfaceBuilder(*args, **xargs).register(module_name)
    raise ValueError(
        f"Unknown combination of driver '{driver_name}' and type '{driver_type}'"
    )


@overload
def request_interface(
    interface_type: Literal["i2c"],
    module_name: str,
    address: int,
) -> I2CInterfaceTemplate:
    ...


@overload
def request_interface(
    interface_type: Literal["gpio"],
    module_name: str,
) -> GPIOInterfaceTemplate:
    ...


@overload
def request_interface(
    interface_type: Literal["spi"], module_name: str, mode: int, speed_hz: int
) -> SPIInterfaceTemplate:
    ...


@overload
def request_interface(
    interface_type: Literal["uart"], module_name: str, baudrate: int
) -> UARTInterfaceTemplate:
    ...


def request_interface(
    interface_type,
    module_name,
    *args,
    **xargs,
):
    if interface_type == "i2c":
        return InterfaceManager.request_interface("i2c", module_name, *args, **xargs)
    elif interface_type == "gpio":
        return InterfaceManager.request_interface("gpio", module_name, *args, **xargs)
    elif interface_type == "spi":
        return InterfaceManager.request_interface("spi", module_name, *args, **xargs)
    elif interface_type == "uart":
        return InterfaceManager.request_interface("uart", module_name, *args, **xargs)
    else:
        raise ValueError(f"Unsupported interface type '{interface_type}'")
