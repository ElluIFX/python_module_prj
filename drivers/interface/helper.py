from typing import TYPE_CHECKING, Dict, Literal, Optional, Union, overload

from .manager import (
    GPIOInterfaceTemplate,
    I2CInterfaceTemplate,
    InterfaceManager,
    SPIInterfaceTemplate,
    UartInterfaceTemplate,
)

if TYPE_CHECKING:
    from .if_cp2112 import (
        CP2112_GPIOInterface,
        CP2112_GPIOInterfaceBuilder,
        CP2112_I2CInterfaceBuilder,
    )
    from .if_periphery import (
        Periphery_GPIOInterfaceBuilder,
        Periphery_I2CInterfaceBuilder,
        Periphery_SPIInterfaceBuilder,
        Periphery_UartInterfaceBuilder,
    )
    from .if_pyserial import PySerial_UartInterfaceBuilder
    from .if_smbus2 import SMBus2_I2CInterfaceBuilder


@overload
def register_interface(
    driver_name: Literal["cp2112"],
    driver_type: Literal["i2c"],
    clock=400000,
    retry=3,
    txrx_leds=True,
    add_lock=True,
) -> "CP2112_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["cp2112"],
    driver_type: Literal["gpio"],
    pinmap: Optional[Dict[str, "CP2112_GPIOInterface.AvailableGPIOs"]] = None,
) -> "CP2112_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["i2c"],
    devpath: str,
    keep_alive: bool = False,
) -> "Periphery_I2CInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["spi"],
    devpath: str,
) -> "Periphery_SPIInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["uart"],
    devpath: str,
) -> "Periphery_UartInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["periphery"],
    driver_type: Literal["gpio"],
    pinmap: Optional[Dict[str, str]] = None,
) -> "Periphery_GPIOInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["pyserial"],
    driver_type: Literal["uart"],
    port: str,
) -> "PySerial_UartInterfaceBuilder":
    ...


@overload
def register_interface(
    driver_name: Literal["smbus2"],
    driver_type: Literal["i2c"],
    bus: Union[int, str],
) -> "SMBus2_I2CInterfaceBuilder":
    ...


def register_interface(
    driver_name: Literal["cp2112", "periphery", "pyserial", "smbus2"],
    driver_type: Literal["i2c", "gpio", "spi", "uart"],
    *args,
    **xargs,
) -> Union[
    "CP2112_I2CInterfaceBuilder",
    "CP2112_GPIOInterfaceBuilder",
    "Periphery_I2CInterfaceBuilder",
    "Periphery_SPIInterfaceBuilder",
    "Periphery_UartInterfaceBuilder",
    "Periphery_GPIOInterfaceBuilder",
    "PySerial_UartInterfaceBuilder",
    "SMBus2_I2CInterfaceBuilder",
]:
    if driver_name == "cp2112":
        from .if_cp2112 import CP2112_GPIOInterfaceBuilder, CP2112_I2CInterfaceBuilder

        if driver_type == "i2c":
            return CP2112_I2CInterfaceBuilder(*args, **xargs).register()
        elif driver_type == "gpio":
            return CP2112_GPIOInterfaceBuilder(*args, **xargs).register()
        else:
            raise ValueError("Invalid driver type")
    elif driver_name == "periphery":
        from .if_periphery import (
            Periphery_GPIOInterfaceBuilder,
            Periphery_I2CInterfaceBuilder,
            Periphery_SPIInterfaceBuilder,
            Periphery_UartInterfaceBuilder,
        )

        if driver_type == "i2c":
            return Periphery_I2CInterfaceBuilder(*args, **xargs).register()
        elif driver_type == "spi":
            return Periphery_SPIInterfaceBuilder(*args, **xargs).register()
        elif driver_type == "uart":
            return Periphery_UartInterfaceBuilder(*args, **xargs).register()
        elif driver_type == "gpio":
            return Periphery_GPIOInterfaceBuilder(*args, **xargs).register()
        else:
            raise ValueError("Invalid driver type")
    elif driver_name == "pyserial":
        from .if_pyserial import PySerial_UartInterfaceBuilder

        if driver_type == "uart":
            return PySerial_UartInterfaceBuilder(*args, **xargs).register()
        else:
            raise ValueError("Invalid driver type")
    elif driver_name == "smbus2":
        from .if_smbus2 import SMBus2_I2CInterfaceBuilder

        if driver_type == "i2c":
            return SMBus2_I2CInterfaceBuilder(*args, **xargs).register()
        else:
            raise ValueError("Invalid driver type")
    else:
        raise ValueError("Unknown driver name")


@overload
def request_interface(
    interface_type: Literal["i2c"],
    module_name: str,
    addr: int,
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
) -> UartInterfaceTemplate:
    ...


def request_interface(
    interface_type: Literal["i2c", "gpio", "spi", "uart"],
    module_name: str,
    *args,
    **xargs,
) -> Union[
    I2CInterfaceTemplate,
    GPIOInterfaceTemplate,
    SPIInterfaceTemplate,
    UartInterfaceTemplate,
]:
    if interface_type == "i2c":
        return InterfaceManager.request_i2c_interface(module_name, *args, **xargs)
    elif interface_type == "gpio":
        return InterfaceManager.request_gpio_interface(module_name, *args, **xargs)
    elif interface_type == "spi":
        return InterfaceManager.request_spi_interface(module_name, *args, **xargs)
    elif interface_type == "uart":
        return InterfaceManager.request_uart_interface(module_name, *args, **xargs)
    else:
        raise ValueError("Invalid driver type")
