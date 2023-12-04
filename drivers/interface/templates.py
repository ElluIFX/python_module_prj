from functools import cached_property
from io import BufferedRWPair, TextIOWrapper
from typing import Callable, Dict, List, Literal, Optional, Union, final

from loguru import logger

from .errors import InterfaceNotFoundError

__all__ = [
    "I2CMessageTemplate",
    "I2CInterfaceTemplate",
    "SPIInterfaceTemplate",
    "UARTInterfaceTemplate",
    "GPIOInterfaceTemplate",
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
        raise NotImplementedError("Interface can not be reopened")

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

    def read(self, length: int = 1) -> bytes:
        """
        Read data from the UART bus
        """
        raise NotImplementedError()

    def readline(self, length: int = -1) -> bytes:
        """
        Read a line from the UART bus
        """
        buf = bytearray()
        while True:
            buf.extend(self.read(1))
            if (length > 0 and len(buf) >= length) or (len(buf) > 0 and buf[-1] == 10):
                break
        return bytes(buf)

    def flush(self):
        """
        Flush the UART bus buffer
        """
        ...

    def readinto(self, buf: bytearray) -> int:
        """
        Read data from the UART bus into buffer
        """
        data = self.read(len(buf))
        buf[: len(data)] = data
        return len(data)

    @final
    def textio_wrapper(self, encoding: Optional[str] = None) -> TextIOWrapper:
        """
        Return a TextIOWrapper of the UART bus
        """
        self.closed = False
        self.readable = lambda: True
        self.writable = lambda: True
        self.seekable = lambda: False
        return TextIOWrapper(
            BufferedRWPair(self, self),  # type: ignore
            write_through=True,
            line_buffering=True,
            encoding=encoding,
        )

    @property
    def in_waiting(self) -> int:
        """
        Return the number of bytes in the input buffer
        """
        raise NotImplementedError()


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

    def get_mode(self, pin_name: str) -> GpioModes_T:
        """
        Return the mode of a pin
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
            raise InterfaceNotFoundError(f"Pin {pin_name} not available")
        return GPIOPinInstance(self, pin_name)


class GPIOPinInstance:
    def __init__(self, io: GPIOInterfaceTemplate, pin_name: str) -> None:
        self._io = io
        self._pin_name = pin_name

    def set_mode(self, mode: GpioModes_T):
        """
        Set the mode of this pin
        """
        self._io.set_mode(self._pin_name, mode)

    def get_mode(self) -> GpioModes_T:
        """
        Return the mode of this pin
        """
        return self._io.get_mode(self._pin_name)

    mode = property(get_mode, set_mode)

    def free(self):
        """
        Free this pin
        """
        self._io.free(self._pin_name)

    def write(self, level: bool):
        """
        Write digital value to this pin
        """
        self._io.write(self._pin_name, level)

    def read(self) -> bool:
        """
        Read digital value from this pin
        """
        return self._io.read(self._pin_name)

    value = property(read, write)
    level = property(read, write)

    def read_analog(self) -> float:
        """
        Read analog value from this pin
        Value range: 0.0 ~ 1.0
        """
        return self._io.read_analog(self._pin_name)

    def write_analog(self, value: float):
        """
        Write analog value to this pin
        Value range: 0.0 ~ 1.0
        """
        self._io.write_analog(self._pin_name, value)

    analog_value = property(read_analog, write_analog)

    def write_pwm_freq(self, freq: float):
        """
        Write PWM frequency to this pin
        """
        self._io.write_pwm_freq(self._pin_name, freq)

    freq = property(None, write_pwm_freq)

    def write_pwm_duty(self, duty: float):
        """
        Write PWM duty to this pin
        Duty range: 0.0 ~ 1.0
        """
        self._io.write_pwm_duty(self._pin_name, duty)

    duty = property(None, write_pwm_duty)

    def write_pwm(self, freq: float, duty: float, polarity: bool = False):
        """
        Set PWM output to this pin
        Duty range: 0.0 ~ 1.0
        """
        self._io.write_pwm(self._pin_name, freq, duty, polarity)

    def get_available_pinmode(self) -> List[GpioModes_T]:
        """
        Return all available modes of this pin
        """
        return self._io.get_available_pinmode(self._pin_name)

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
        self._io.set_interrupt(self._pin_name, intmode, pinmode, callback)

    def read_interrupt(self) -> bool:
        """
        Read the interrupt state of this pin
        """
        return self._io.read_interrupt(self._pin_name)

    int_state = property(read_interrupt)

    def poll_interrupt(self, timeout: Optional[float] = None) -> bool:
        """
        Poll the interrupt of this pin

        return: True if interrupt triggered, False if timeout
        """
        return self._io.poll_interrupt(self._pin_name, timeout)
