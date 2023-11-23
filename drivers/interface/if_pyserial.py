from typing import Optional

from serial import Serial

from .manager import InterfaceBuilderTemplate, UartInterfaceTemplate


class PySerial_UartInterface(UartInterfaceTemplate):
    def __init__(self, port: str, baudrate: int) -> None:
        self._ser = Serial(port, baudrate, timeout=0, write_timeout=0)
        self._port = port
        self._baudrate = baudrate
        self._timeout = 0
        try:
            self._ser.set_buffer_size(rx_size=1024 * 1024)  # 1MB
        except Exception:
            pass

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

    @property
    def baudrate(self) -> int:
        return self._ser.baudrate

    @baudrate.setter
    def baudrate(self, baudrate: int) -> None:
        self._ser.baudrate = baudrate

    @property
    def data_bits(self) -> int:
        return self._ser.bytesize

    @data_bits.setter
    def data_bits(self, data_bits: int) -> None:
        self._ser.bytesize = data_bits

    @property
    def stop_bits(self) -> int:
        return int(self._ser.stopbits)

    @stop_bits.setter
    def stop_bits(self, stop_bits: int) -> None:
        self._ser.stopbits = stop_bits

    @property
    def parity(self) -> str:
        return self._ser.parity

    @parity.setter
    def parity(self, parity: str) -> None:
        self._ser.parity = parity


class PySerial_UartInterfaceBuilder(InterfaceBuilderTemplate):
    def __init__(self, port: str) -> None:
        self._port = port
        self.dev_type = "uart"

    def build(self, baudrate: int) -> PySerial_UartInterface:
        return PySerial_UartInterface(self._port, baudrate)
