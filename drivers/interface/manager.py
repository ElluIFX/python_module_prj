from functools import cached_property
from typing import Any, Dict, List, Literal, Optional, Tuple, Union

from loguru import logger


class I2CMessageTemplate:
    @staticmethod
    def write(data: Union[bytes, List[int]]) -> "I2CMessageTemplate":
        """
        Write data to the I2C bus
        """
        raise NotImplementedError()

    @staticmethod
    def read(length: int) -> "I2CMessageTemplate":
        """
        Read data from the I2C bus
        """
        raise NotImplementedError()

    @staticmethod
    def write_addr(addr: int, data: Union[bytes, List[int]]) -> "I2CMessageTemplate":
        """
        Write data to the I2C bus with specified address
        """
        raise NotImplementedError()

    @staticmethod
    def read_addr(addr: int, length: int) -> "I2CMessageTemplate":
        """
        Read data from the I2C bus with specified address
        """
        raise NotImplementedError()

    def __len__(self) -> int:
        """
        Return the length of the message
        """
        raise NotImplementedError()

    def __bytes__(self) -> bytes:
        """
        Return the message as bytes
        """
        raise NotImplementedError()

    def __repr__(self) -> str:
        """
        Return the message as string
        """
        raise NotImplementedError()

    def __iter__(self):
        """
        Return an iterator over the data
        """
        raise NotImplementedError()


class I2CInterfaceTemplate:
    @property
    def address(self) -> int:
        """
        Return the address of communication target

        Note: 7-bit address, r/w bit will be added automatically
        """
        raise NotImplementedError()

    @address.setter
    def address(self, address: int):
        """
        Set the address of communication target

        Note: 7-bit address, r/w bit will be added automatically
        """
        raise NotImplementedError()

    def write_raw_byte(self, value: int):
        """
        Write a byte to the I2C bus
        """
        raise NotImplementedError()

    def read_raw_byte(self) -> int:
        """
        Read a byte from the I2C bus
        """
        raise NotImplementedError()

    def write_reg_byte(self, register: int, value: int):
        """
        Write a byte to specified register
        """
        raise NotImplementedError()

    def read_reg_byte(self, register: int) -> int:
        """
        Read a byte from specified register
        """
        raise NotImplementedError()

    def write_reg_data(self, register: int, data: Union[bytes, List[int]]):
        """
        Write data to specified register
        """
        raise NotImplementedError()

    def read_reg_data(self, register: int, length: int) -> bytes:
        """
        Read data from specified register
        """
        raise NotImplementedError()

    @property
    def new_msg(self) -> type[I2CMessageTemplate]:
        """
        Create a new I2C message
        """
        raise NotImplementedError()

    @cached_property
    def max_transfer_size(self) -> int:
        """
        Return the max transfer size of the I2C Driver

        Note: size = reg byte + data bytes, not include address
        """
        return 4095

    def transfer_msg(self, msgs: list[I2CMessageTemplate]):
        """
        Excute a series of I2C messages
        """
        raise NotImplementedError()

    def check_address(self) -> bool:
        """
        Check if the device is connected
        """
        raise NotImplementedError()

    def close(self):
        """
        Close the I2C bus
        """
        ...

    def reopen(self):
        """
        Reopen the I2C bus
        """
        ...


class SPIInterfaceTemplate:
    @property
    def speed_hz(self) -> float:
        """
        Return the speed of the SPI bus
        """
        raise NotImplementedError()

    @property
    def mode(self) -> int:
        """
        Return the mode of the SPI bus
        """
        raise NotImplementedError()

    @speed_hz.setter
    def speed_hz(self, speed_hz: float):
        """
        Set the speed of the SPI bus
        """
        raise NotImplementedError()

    @mode.setter
    def mode(self, mode: int):
        """
        Set the mode of the SPI bus
        """
        raise NotImplementedError()

    def write(self, data: bytes):
        """
        Write data to the SPI bus
        """
        raise NotImplementedError()

    def read(self, length: int) -> bytes:
        """
        Read data from the SPI bus
        """
        raise NotImplementedError()

    def transfer(self, data: bytes) -> bytes:
        """
        Write and read data from the SPI bus
        """
        raise NotImplementedError()

    def close(self):
        """
        Close the SPI bus
        """
        ...

    def reopen(self):
        """
        Reopen the SPI bus
        """
        ...

    def set_auto_cs(self, enable: bool, polarity: bool):
        """
        Set the auto chip select of the SPI bus

        plority: True for active high, False for active low

        Note: If this function raises NotImplementedError, \
              user should fallback to using GPIO interface. \
              For drivers, if the device can auto manage the \
              CS pin, this function should be overrided with \
              empty body.
        """
        raise NotImplementedError()

    def set_cs(self, level: bool):
        """
        Set the chip select of the SPI bus
        """
        raise NotImplementedError()


class UartInterfaceTemplate:
    @property
    def baudrate(self) -> int:
        """
        Return the baudrate of the UART bus
        """
        raise NotImplementedError()

    @baudrate.setter
    def baudrate(self, baudrate: int):
        """
        Set the baudrate of the UART bus
        """
        raise NotImplementedError()

    @property
    def data_bits(self) -> int:
        """
        Return the data bits of the UART bus
        """
        raise NotImplementedError()

    @data_bits.setter
    def data_bits(self, data_bits: int):
        """
        Set the data bits of the UART bus
        """
        raise NotImplementedError()

    @property
    def parity(self) -> str:
        """
        Return the parity of the UART bus
        """
        raise NotImplementedError()

    @parity.setter
    def parity(self, parity: str):
        """
        Set the parity of the UART bus
        """
        raise NotImplementedError()

    @property
    def stop_bits(self) -> int:
        """
        Return the stop bits of the UART bus
        """
        raise NotImplementedError()

    @stop_bits.setter
    def stop_bits(self, stop_bits: int):
        """
        Set the stop bits of the UART bus
        """
        raise NotImplementedError()

    def write(self, data: bytes):
        """
        Write data to the UART bus
        """
        raise NotImplementedError()

    def flush(self):
        """
        Flush the UART bus buffer
        """
        raise NotImplementedError()

    def read(self, length: int, timeout: Optional[float] = None) -> bytes:
        """
        Read data from the UART bus
        """
        raise NotImplementedError()

    def close(self):
        """
        Close the UART bus
        """
        ...

    def reopen(self):
        """
        Reopen the UART bus
        """
        ...

    @property
    def in_waiting(self) -> int:
        """
        Return the number of bytes in the input buffer
        """
        raise NotImplementedError()


class GPIOInterfaceTemplate:
    GPIOModes = Literal[
        "input_no_pull",
        "input_pull_up",
        "input_pull_down",
        "output_push_pull",
        "output_open_drain",
        "output_pwm",
        "input_analog",
        "output_analog",
        "special_func",
    ]

    def set_mode(self, pin_name: str, mode: GPIOModes):
        """
        Set the mode of a GPIO pin

        Note: For most drivers, you must call this function to \
              initialize the GPIO pin before using it
        """
        raise NotImplementedError()

    def free(self, pin_name: str):
        """
        Free a GPIO pin

        Note: Not necessary for most cases, use if multiple \
              instances are using the same GPIO pins and you \
              want to manage the pins share. Wont raise error \
              even if driver does not support this
        """
        ...

    def write(self, pin_name: str, level: bool):
        """
        Write digital value to the GPIO pin
        """
        raise NotImplementedError()

    def read(self, pin_name: str) -> bool:
        """
        Read digital value from the GPIO pin
        """
        raise NotImplementedError()

    def read_analog(self, pin_name: str) -> float:
        """
        Read analog value from the GPIO pin
        Value range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def write_analog(self, pin_name: str, value: float):
        """
        Write analog value to the GPIO pin
        Value range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def write_pwm_freq(self, pin_name: str, freq: int):
        """
        Write PWM frequency to the GPIO pin
        """
        raise NotImplementedError()

    def write_pwm_duty(self, pin_name: str, duty: float):
        """
        Write PWM duty to the GPIO pin
        Duty range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def write_pwm(self, pin_name: str, freq: int, duty: float):
        """
        Write PWM to the GPIO pin
        Duty range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def get_available_pins(self) -> List[Tuple[str, List[GPIOModes]]]:
        """
        Return all available pins with their available modes
        """
        raise NotImplementedError()

    def get_pin(self, pin_name: str):
        """
        Return a GPIO instance
        """

        class GPIOInstance:
            def __init__(self, pin_name: str, interface: GPIOInterfaceTemplate) -> None:
                self._name = pin_name
                self._interface = interface

            def __getattr__(self, name: str):
                return lambda *args, **kwargs: getattr(self._interface, name)(
                    self._name, *args, **kwargs
                )

        return GPIOInstance(pin_name, self)

    def close(self):
        """
        Close the GPIO bus
        """
        ...


AvailableTemplates = Union[
    I2CInterfaceTemplate,
    SPIInterfaceTemplate,
    UartInterfaceTemplate,
    GPIOInterfaceTemplate,
]

AvailableDevTypes = Literal["i2c", "spi", "uart", "gpio"]


class InterfaceBuilderTemplate:
    """
    Interface builder template
    """

    dev_type: Optional[AvailableDevTypes] = None

    def __init__(self, *args, **kwargs) -> None:
        """
        Initialize the interface builder

        Pass parameters that are globally shared
        """
        raise NotImplementedError()

    def build(self, *args, **kwargs) -> AvailableTemplates:
        """
        Build the interface

        Pass parameters that from module who requests the interface
        """
        raise NotImplementedError()

    def register(self, module_name: Union[None, str, List[str]] = None):
        """
        Register the interface to the interface manager

        module_name: offer name if this interface is specific to a module
        """
        assert self.dev_type is not None, "Invalid interface builder"
        if module_name is None:
            InterfaceManager.register_global_interface(self.dev_type, self)
        else:
            InterfaceManager.register_specific_interface(
                self.dev_type, module_name, self
            )
        self._module_name = module_name
        return self

    def unregister(self):
        """
        Unregister the interface from the interface manager
        """
        assert self.dev_type is not None, "Invalid interface builder"
        if self._module_name is None:
            InterfaceManager.unregister_global_interface(self.dev_type)
        else:
            InterfaceManager.unregister_specific_interface(
                self.dev_type, self._module_name
            )
        return self


interface_dict: Dict[AvailableDevTypes, Optional[InterfaceBuilderTemplate]] = {
    "i2c": None,
    "spi": None,
    "uart": None,
    "gpio": None,
}

specific_interface_dict: Dict[str, InterfaceBuilderTemplate] = {}


class InterfaceManager:
    """
    Interface manager for the whole system
    """

    @staticmethod
    def register_global_interface(
        dev_type: AvailableDevTypes, interface: InterfaceBuilderTemplate
    ):
        """
        Register a global interface
        """
        assert dev_type in interface_dict, f"Interface {dev_type} not supported"
        assert (
            interface_dict[dev_type] is None
        ), f"Interface {dev_type} already registered"
        interface_dict[dev_type] = interface
        logger.info(f"Registered a global {dev_type.upper()} interface")

    @staticmethod
    def register_specific_interface(
        dev_type: AvailableDevTypes,
        module_name: Union[str, List[str]],
        interface: InterfaceBuilderTemplate,
    ):
        """
        Register a specific interface
        """

        def register(dev_type, module_name, interface):
            module_name = module_name.lower()
            fullname = f"{dev_type}_{module_name}"
            assert (
                fullname not in specific_interface_dict
            ), f"Interface {fullname} already registered"
            specific_interface_dict[fullname] = interface
            logger.info(
                f"Registered a specific {dev_type.upper()} interface for {module_name.upper()}"
            )

        if isinstance(module_name, str):
            register(dev_type, module_name, interface)
        elif isinstance(module_name, list):
            for name in module_name:
                register(dev_type, name, interface)
        else:
            raise ValueError("Invalid module name")

    @staticmethod
    def unregister_global_interface(dev_type: AvailableDevTypes):
        """
        Unregister a global interface
        """
        assert dev_type in interface_dict, f"Interface {dev_type} not supported"
        interface_dict[dev_type] = None
        logger.info(f"Global {dev_type.upper()} interface unregistered")

    @staticmethod
    def unregister_specific_interface(
        dev_type: AvailableDevTypes,
        module_name: Union[str, List[str]],
    ):
        """
        Unregister a specific interface
        """

        def unregister(dev_type, module_name):
            module_name = module_name.lower()
            fullname = f"{dev_type}_{module_name}"
            if fullname in specific_interface_dict:
                specific_interface_dict.pop(fullname)
                logger.info(
                    f"Specific {dev_type.upper()} interface for {module_name.upper()} unregistered"
                )

        if isinstance(module_name, str):
            unregister(dev_type, module_name)
        elif isinstance(module_name, list):
            for name in module_name:
                unregister(dev_type, name)

    @staticmethod
    def _internal_get_dev(
        dev_type: Any, module_name: str, *args, **kwargs
    ) -> AvailableTemplates:
        module_name = module_name.lower()
        logger.info(
            f"Module {module_name.upper()} requested a {dev_type.upper()} interface"
        )
        _fullname = f"{dev_type}_{module_name}"
        if _fullname in specific_interface_dict:
            dev = specific_interface_dict[_fullname]
        else:
            dev = interface_dict.get(dev_type, None)
        assert (
            dev is not None
        ), f"{dev_type.upper()} interface not registered, register it first"
        return dev.build(*args, **kwargs)

    @staticmethod
    def request_i2c_interface(module_name: str, address: int) -> I2CInterfaceTemplate:
        return InterfaceManager._internal_get_dev("i2c", module_name, address=address)  # type: ignore

    @staticmethod
    def request_spi_interface(
        module_name: str, mode: int, speed_hz: int
    ) -> SPIInterfaceTemplate:
        return InterfaceManager._internal_get_dev(
            "spi", module_name, mode=mode, speed_hz=speed_hz
        )  # type: ignore

    @staticmethod
    def request_uart_interface(
        module_name: str, baudrate: int
    ) -> UartInterfaceTemplate:
        return InterfaceManager._internal_get_dev(
            "uart", module_name, baudrate=baudrate
        )  # type: ignore

    @staticmethod
    def request_gpio_interface(module_name: str) -> GPIOInterfaceTemplate:
        return InterfaceManager._internal_get_dev("gpio", module_name)  # type: ignore


__all__ = [
    "I2CMessageTemplate",
    "I2CInterfaceTemplate",
    "SPIInterfaceTemplate",
    "UartInterfaceTemplate",
    "GPIOInterfaceTemplate",
    "InterfaceBuilderTemplate",
    "AvailableTemplates",
    "AvailableDevTypes",
    "InterfaceManager",
]
