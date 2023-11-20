from typing import Optional

from serial import Serial

from .manager import InterfaceBuilderTemplate, UartInterfaceTemplate


class Pyserial_UartInterface(UartInterfaceTemplate):
    def __init__(self, port: str, baudrate: int) -> None:
        self._ser = Serial(port, baudrate, timeout=0, write_timeout=0)
        self._port = port
        self._baudrate = baudrate
        self._timeout = 0

    def write(self, data: bytes):
        self._ser.write(data)

    def read(self, length: int, timeout: Optional[float] = None) -> bytes:
        if timeout is not None and timeout != self._timeout:
            self._ser.timeout = timeout
            self._timeout = timeout
        return self._ser.read(length)

    @property
    def in_waiting(self) -> int:
        return self._ser.in_waiting

    def close(self):
        self._ser.close()

    def reopen(self):
        self._ser.open()

    def flush(self):
        self._ser.flush()

    def set_baudrate(self, baudrate: int):
        self._ser.baudrate = baudrate

    def set_option(self, data_bits: int, parity: str, stop_bits: int):
        self._ser.bytesize = data_bits
        self._ser.parity = parity
        self._ser.stopbits = stop_bits


class Pyserial_UartInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, port: str) -> None:
        self._port = port
        self.dev_type = "uart"

    def build(self, baudrate: int) -> Pyserial_UartInterface:
        return Pyserial_UartInterface(self._port, baudrate)
