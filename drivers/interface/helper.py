from typing import TYPE_CHECKING, Dict, List, Literal, Optional, Union, overload

from .manager import InterfaceManager
from .template import (
    GPIOInterfaceTemplate,
    I2CInterfaceTemplate,
    SPIInterfaceTemplate,
    UARTInterfaceTemplate,
)

if TYPE_CHECKING:
    from .interfaces._ch347 import (
        CH347_GPIOInterfaceBuilder,
        CH347_I2CInterfaceBuilder,
        CH347_SPIInterfaceBuilder,
        CH347_UARTInterfaceBuilder,
        CH347AvailablePins,
    )
    from .interfaces._cp2112 import (
        CP2112_GPIOInterfaceBuilder,
        CP2112_I2CInterfaceBuilder,
        CP2112AvailablePins,
    )
    from .interfaces._pcal6416a import (
        PCAL6416A_GPIOInterfaceBuilder,
        PCAL6416AAvailablePins,
    )
    from .interfaces._periphery import (
        Periphery_GPIOInterfaceBuilder,
        Periphery_I2CInterfaceBuilder,
        Periphery_SPIInterfaceBuilder,
        Periphery_UARTInterfaceBuilder,
    )
    from .interfaces._pyserial import PySerial_UARTInterfaceBuilder
    from .interfaces._smbus2 import SMBus2_I2CInterfaceBuilder
    from .interfaces._spidev import Spidev_SPIInterfaceBuilder
    from .template import GpioModes_T


@overload
def register_interface(
    driver_name: Literal["cp2112"],
    interface_type: Literal["i2c"],
    clock: int = 400000,
    retry: int = 3,
    txrx_leds: bool = True,
    specific_module: Union[None, str, List[str]] = None,
) -> "CP2112_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["cp2112"],
    interface_type: Literal["gpio"],
    pinmap: Optional[Dict[str, "CP2112AvailablePins"]] = None,
    specific_module: Union[None, str, List[str]] = None,
) -> "CP2112_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    interface_type: Literal["i2c"],
    clock: Literal[20000, 50000, 100000, 200000, 400000, 750000, 1000000] = 400000,
    specific_module: Union[None, str, List[str]] = None,
) -> "CH347_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    interface_type: Literal["uart"],
    uart_index: int = 0,
    specific_module: Union[None, str, List[str]] = None,
) -> "CH347_UARTInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    interface_type: Literal["spi"],
    enable_cs: bool = True,
    cs: Literal[0, 1] = 0,
    cs_high: bool = False,
    auto_reset: bool = False,
    specific_module: Union[None, str, List[str]] = None,
) -> "CH347_SPIInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["ch347"],
    interface_type: Literal["gpio"],
    pinmap: Optional[Dict[str, "CH347AvailablePins"]] = None,
    specific_module: Union[None, str, List[str]] = None,
) -> "CH347_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    interface_type: Literal["i2c"],
    devpath: str,
    keep_alive: bool = False,
    specific_module: Union[None, str, List[str]] = None,
) -> "Periphery_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    interface_type: Literal["spi"],
    devpath: str,
    specific_module: Union[None, str, List[str]] = None,
) -> "Periphery_SPIInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    interface_type: Literal["uart"],
    devpath: str,
    specific_module: Union[None, str, List[str]] = None,
) -> "Periphery_UARTInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    interface_type: Literal["gpio"],
    pinmap: Optional[Dict[str, str]] = None,
    modemap: Optional[Dict[str, "GpioModes_T"]] = None,
    specific_module: Union[None, str, List[str]] = None,
) -> "Periphery_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["pyserial"],
    interface_type: Literal["uart"],
    port: str,
    specific_module: Union[None, str, List[str]] = None,
) -> "PySerial_UARTInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["smbus2"],
    interface_type: Literal["i2c"],
    bus: Union[int, str],
    specific_module: Union[None, str, List[str]] = None,
) -> "SMBus2_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["spidev"],
    interface_type: Literal["spi"],
    channel: int,
    port: int,
    specific_module: Union[None, str, List[str]] = None,
) -> "Spidev_SPIInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["pca6416a"],
    interface_type: Literal["gpio"],
    pinmap: Optional[Dict[str, "PCAL6416AAvailablePins"]] = None,
    address: int = 0x21,
    cache_reg: bool = True,
) -> "PCAL6416A_GPIOInterfaceBuilder":
    ...


def register_interface(
    driver_name,
    interface_type,
    *args,
    **xargs,
):
    """
    Register an interface to the interface manager

    Args:
        driver_name: Hardware driver, e.g. cp2112, ch347, periphery, pyserial, etc.
        interface_type: Type of interface, e.g. i2c, spi, uart, gpio, etc.

    Returns:
        Corresponding interface builder
    """
    mod = xargs.pop("specific_module", None)
    if driver_name == "cp2112":
        from .interfaces._cp2112 import (
            CP2112_GPIOInterfaceBuilder,
            CP2112_I2CInterfaceBuilder,
        )

        if interface_type == "i2c":
            return CP2112_I2CInterfaceBuilder(*args, **xargs).register(mod)
        elif interface_type == "gpio":
            return CP2112_GPIOInterfaceBuilder(*args, **xargs).register(mod)
    elif driver_name == "ch347":
        from .interfaces._ch347 import (
            CH347_GPIOInterfaceBuilder,
            CH347_I2CInterfaceBuilder,
            CH347_SPIInterfaceBuilder,
            CH347_UARTInterfaceBuilder,
        )

        if interface_type == "i2c":
            return CH347_I2CInterfaceBuilder(*args, **xargs).register(mod)
        elif interface_type == "uart":
            return CH347_UARTInterfaceBuilder(*args, **xargs).register(mod)
        elif interface_type == "spi":
            return CH347_SPIInterfaceBuilder(*args, **xargs).register(mod)
        elif interface_type == "gpio":
            return CH347_GPIOInterfaceBuilder(*args, **xargs).register(mod)
    elif driver_name == "periphery":
        from .interfaces._periphery import (
            Periphery_GPIOInterfaceBuilder,
            Periphery_I2CInterfaceBuilder,
            Periphery_SPIInterfaceBuilder,
            Periphery_UARTInterfaceBuilder,
        )

        if interface_type == "i2c":
            return Periphery_I2CInterfaceBuilder(*args, **xargs).register(mod)
        elif interface_type == "spi":
            return Periphery_SPIInterfaceBuilder(*args, **xargs).register(mod)
        elif interface_type == "uart":
            return Periphery_UARTInterfaceBuilder(*args, **xargs).register(mod)
        elif interface_type == "gpio":
            return Periphery_GPIOInterfaceBuilder(*args, **xargs).register(mod)
    elif driver_name == "pyserial":
        from .interfaces._pyserial import PySerial_UARTInterfaceBuilder

        if interface_type == "uart":
            return PySerial_UARTInterfaceBuilder(*args, **xargs).register(mod)
    elif driver_name == "smbus2":
        from .interfaces._smbus2 import SMBus2_I2CInterfaceBuilder

        if interface_type == "i2c":
            return SMBus2_I2CInterfaceBuilder(*args, **xargs).register(mod)
    elif driver_name == "spidev":
        from .interfaces._spidev import Spidev_SPIInterfaceBuilder

        if interface_type == "spi":
            return Spidev_SPIInterfaceBuilder(*args, **xargs).register(mod)
    elif driver_name == "pca6416a":
        from .interfaces._pcal6416a import PCAL6416A_GPIOInterfaceBuilder

        if interface_type == "gpio":
            return PCAL6416A_GPIOInterfaceBuilder(*args, **xargs).register(mod)
    raise ValueError(
        f"Unknown combination of driver '{driver_name}' and type '{interface_type}'"
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
    """
    Request an interface from the interface manager

    Args:
        interface_type: Type of interface, e.g. i2c, spi, uart, gpio, etc.
        module_name: who am I, should same as the module class name
    """
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
