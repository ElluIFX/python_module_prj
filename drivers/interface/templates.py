from functools import cached_property
from typing import Callable, Dict, List, Literal, Optional, Union, final

from loguru import logger

from .errors import InterfaceNotFound

__all__ = [
    "I2CMessageTemplate",
    "I2CInterfaceTemplate",
    "SPIInterfaceTemplate",
    "UARTInterfaceTemplate",
    "GPIOInterfaceTemplate",
    "FAKE_GPIO_NAME",
    "GpioModes_T",
    "IntModes_T",
    "GpioModes",
    "IntModes",
    "GPIOPinInstance",
    "BaseInterfaceTemplate",
]


class BaseInterfaceTemplate:
    _destroyed = True

    def __init__(self) -> None:
        self._destroyed = False  # prevent double destroy
        self._on_destroy: list[Callable[[BaseInterfaceTemplate], None]] = []
        logger.debug(f"Interface {self.__class__.__name__} initialized")

    def close(self):
        """
        Close the interface

        Can be reopened by calling reopen()
        """
        ...

    def reopen(self):
        """
        Reopen the interface
        """
        ...

    @final
    def destroy(self):
        """
        Completely destroy the interface
        Can not be reopened

        Note: Though this function can be automatically called \
              when the instance is garbage collected, but module \
              should better call this function manually as soon \
              as possible to free the hardware resource
        """
        if self._destroyed:
            return
        self.close()
        for func in self._on_destroy:
            func(self)
        self._destroyed = True
        logger.debug(f"Interface {self.__class__.__name__} destroyed")

    def __del__(self):
        self.destroy()


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


class I2CInterfaceTemplate(BaseInterfaceTemplate):
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

    def exchange_msgs(self, msgs: list[I2CMessageTemplate]):
        """
        Exchange a series of I2C messages on bus
        """
        raise NotImplementedError()

    def check_address(self) -> bool:
        """
        Check if the device is connected
        """
        raise NotImplementedError()


class SPIInterfaceTemplate(BaseInterfaceTemplate):
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

    @property
    def byteorder(self) -> Literal["lsb", "msb"]:
        """
        Return the byte order of the SPI bus
        """
        raise NotImplementedError()

    @byteorder.setter
    def byteorder(self, byteorder: Literal["lsb", "msb"]):
        """
        Set the byte order of the SPI bus
        """
        raise NotImplementedError()

    @property
    def bits_per_word(self) -> int:
        """
        Return the bits per word of the SPI bus
        """
        raise NotImplementedError()

    @bits_per_word.setter
    def bits_per_word(self, bits_per_word: int):
        """
        Set the bits per word of the SPI bus
        """
        raise NotImplementedError()

    def write(self, data: Union[bytes, List[int]]):
        """
        Write data to the SPI bus
        """
        raise NotImplementedError()

    def read(self, length: int) -> bytes:
        """
        Read data from the SPI bus
        """
        raise NotImplementedError()

    def transfer(self, data: Union[bytes, List[int]]) -> bytes:
        """
        Write and read data from the SPI bus
        """
        raise NotImplementedError()


class UARTInterfaceTemplate(BaseInterfaceTemplate):
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
    def data_bits(self) -> Literal[5, 6, 7, 8, 16]:
        """
        Return the data bits of the UART bus
        """
        raise NotImplementedError()

    @data_bits.setter
    def data_bits(self, data_bits: Literal[5, 6, 7, 8, 16]):
        """
        Set the data bits of the UART bus
        """
        raise NotImplementedError()

    @property
    def parity(self) -> Literal["N", "E", "O", "M", "S"]:
        """
        Return the parity of the UART bus
        """
        raise NotImplementedError()

    @parity.setter
    def parity(self, parity: Literal["N", "E", "O", "M", "S"]):
        """
        Set the parity of the UART bus
        """
        raise NotImplementedError()

    @property
    def stop_bits(self) -> Literal["1", "1.5", "2"]:
        """
        Return the stop bits of the UART bus
        """
        raise NotImplementedError()

    @stop_bits.setter
    def stop_bits(self, stop_bits: Literal["1", "1.5", "2"]):
        """
        Set the stop bits of the UART bus
        """
        raise NotImplementedError()

    @property
    def timeout(self) -> Optional[float]:
        """
        Return the timeout of the UART bus

        None: blocking
        """
        raise NotImplementedError()

    @timeout.setter
    def timeout(self, timeout: Optional[float]):
        """
        Set the timeout of the UART bus

        None: blocking
        """
        raise NotImplementedError()

    def write(self, data: Union[bytes, List[int]]):
        """
        Write data to the UART bus
        """
        raise NotImplementedError()

    def flush(self):
        """
        Flush the UART bus buffer
        """
        raise NotImplementedError()

    def read(self, length: int) -> bytes:
        """
        Read data from the UART bus
        """
        raise NotImplementedError()

    @property
    def in_waiting(self) -> int:
        """
        Return the number of bytes in the input buffer
        """
        raise NotImplementedError()


FAKE_GPIO_NAME = "__FAKE_GPIO__"  # all operations will be ignored

GpioModes = [
    "input_no_pull",
    "input_pull_up",
    "input_pull_down",
    "output_push_pull",
    "output_open_drain",
    "output_open_source",
    "pwm_output",
    "analog_input",
    "analog_output",
    "interrupt_no_pull",
    "interrupt_pull_up",
    "interrupt_pull_down",
    "special_func",
]

IntModes = [
    "none",
    "rising_edge",
    "falling_edge",
    "both_edge",
    "low_level",
    "high_level",
]

GpioModes_T = Literal[
    "none",
    "input_no_pull",
    "input_pull_up",
    "input_pull_down",
    "output_push_pull",
    "output_open_drain",
    "output_open_source",
    "pwm_output",
    "analog_input",
    "analog_output",
    "interrupt_no_pull",
    "interrupt_pull_up",
    "interrupt_pull_down",
    "special_func",
]

IntModes_T = Literal[
    "none",
    "rising_edge",
    "falling_edge",
    "both_edge",
    "low_level",
    "high_level",
]


class GPIOInterfaceTemplate(BaseInterfaceTemplate):
    def set_mode(self, pin_name: str, mode: GpioModes_T):
        """
        Set the mode of a pin

        Note: For most drivers, you must call this function to \
              initialize the pin before using it.
              For interrupt mode, call set_interrupt() instead.
              Better check get_available_pinmode() before setting.
        """
        raise NotImplementedError()

    def free(self, pin_name: str):
        """
        Free a pin

        Note: Not necessary for most cases, use if multiple \
              instances are using the same pins and you \
              want to manage the pins share. Wont raise error \
              even if driver does not support this
        """
        ...

    def write(self, pin_name: str, level: bool):
        """
        Write digital value to the pin
        """
        raise NotImplementedError()

    def read(self, pin_name: str) -> bool:
        """
        Read digital value from the pin
        """
        raise NotImplementedError()

    def read_analog(self, pin_name: str) -> float:
        """
        Read analog value from the pin
        Value range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def write_analog(self, pin_name: str, value: float):
        """
        Write analog value to the pin
        Value range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def write_pwm_freq(self, pin_name: str, freq: float):
        """
        Write PWM frequency to the pin
        """
        raise NotImplementedError()

    def write_pwm_duty(self, pin_name: str, duty: float):
        """
        Write PWM duty to the pin
        Duty range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def write_pwm(
        self, pin_name: str, freq: float, duty: float, polarity: bool = False
    ):
        """
        Set PWM output to the pin
        Duty range: 0.0 ~ 1.0
        """
        raise NotImplementedError()

    def get_available_pins(self) -> Dict[str, List[GpioModes_T]]:
        """
        Return all available pins with their available modes
        """
        raise NotImplementedError()

    def get_available_pinmode(self, pin_name: str) -> List[GpioModes_T]:
        """
        Return all available modes of a pin
        """
        return self.get_available_pins()[pin_name]

    def set_interrupt(
        self,
        pin_name: str,
        intmode: IntModes_T,
        pinmode: GpioModes_T,
        callback: Optional[Callable[[], None]] = None,
    ):
        """
        Set the interrupt mode of a pin

        pinmode: the mode of the pin, must be interrupt_xxx
        callback: will be called when interrupt triggered, pass None to unregister
        """
        raise NotImplementedError()

    def poll_interrupt(self, pin_name: str, timeout: Optional[float] = None) -> bool:
        """
        Poll the interrupt of a pin

        return: True if interrupt triggered, False if timeout
        """
        raise NotImplementedError()

    def read_interrupt(self, pin_name: str) -> bool:
        """
        Read the interrupt state of a pin
        """
        raise NotImplementedError()

    @final
    def get_pin(self, pin_name: str) -> "GPIOPinInstance":
        """
        Return a GPIO instance
        """
        if pin_name not in self.get_available_pins():
            raise InterfaceNotFound(f"Pin {pin_name} not available")

        class _GPIOPinWrapper:
            def __init__(
                self, pin_name: str, interface: "GPIOInterfaceTemplate"
            ) -> None:
                self._pinname = pin_name
                self._interface = interface

            def __getattr__(self, name: str):
                return lambda *args, **kwargs: getattr(self._interface, name)(
                    self._pinname, *args, **kwargs
                )

        return _GPIOPinWrapper(pin_name, self)  # type: ignore


class GPIOPinInstance:
    def set_mode(self, mode: GpioModes_T):
        """
        Set the mode of this pin
        """
        ...

    def free(self):
        """
        Free this pin
        """
        ...

    def write(self, level: bool):
        """
        Write digital value to this pin
        """
        ...

    def read(self) -> bool:
        """
        Read digital value from this pin
        """
        ...

    def read_analog(self) -> float:
        """
        Read analog value from this pin
        Value range: 0.0 ~ 1.0
        """
        ...

    def write_analog(self, value: float):
        """
        Write analog value to this pin
        Value range: 0.0 ~ 1.0
        """
        ...

    def write_pwm_freq(self, freq: float):
        """
        Write PWM frequency to this pin
        """
        ...

    def write_pwm_duty(self, duty: float):
        """
        Write PWM duty to this pin
        Duty range: 0.0 ~ 1.0
        """
        ...

    def write_pwm(self, freq: float, duty: float, polarity: bool = False):
        """
        Set PWM output to this pin
        Duty range: 0.0 ~ 1.0
        """
        ...

    def get_available_pinmode(self) -> List[GpioModes_T]:
        """
        Return all available modes of this pin
        """
        ...

    def set_interrupt(
        self,
        intmode: IntModes_T,
        pinmode: GpioModes_T,
        callback: Optional[Callable[[], None]] = None,
    ):
        """
        Set the interrupt mode of this pin

        pinmode: the mode of the pin, must be interrupt_xxx
        callback: will be called when interrupt triggered, pass None to unregister
        """
        ...

    def read_interrupt(self) -> bool:
        """
        Read the interrupt state of this pin
        """
        ...

    def poll_interrupt(self, timeout: Optional[float] = None) -> bool:
        """
        Poll the interrupt of this pin

        return: True if interrupt triggered, False if timeout
        """
        ...
