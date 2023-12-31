from typing import Optional

from serial import Serial

from ..manager import BaseInterfaceBuilder
from ..template import UARTInterfaceTemplate
from ..utils import get_permission


class PySerial_UARTInterface(UARTInterfaceTemplate):
    def __init__(self, port: str, baudrate: int) -> None:
        try:
            self._ser = Serial(port, baudrate, timeout=0, write_timeout=0)
        except IOError as e:
            if e.errno == 13:
                get_permission(port)
                self._ser = Serial(port, baudrate, timeout=0, write_timeout=0)
            else:
                raise e

        self._port = port
        self._baudrate = baudrate
        try:
            self._ser.set_buffer_size(rx_size=1024 * 1024)  # 1MB
        except Exception:
            pass
        return super().__init__()

    @property
    def timeout(self) -> Optional[float]:
        return self._ser.timeout

    @timeout.setter
    def timeout(self, timeout: Optional[float]) -> None:
        self._ser.timeout = timeout

    def write(self, data: bytes):
        self._ser.write(data)

    def read(self, length: int = 1) -> bytes:
        return self._ser.read(length)

    def readline(self, length: int = -1) -> bytes:
        return self._ser.readline(length)

    @property
    def in_waiting(self) -> int:
        return self._ser.in_waiting

    def close(self):
        self._ser.close()

    def reopen(self):
        assert not self._destroyed, "Interface has been destroyed"
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
    def stop_bits(self) -> str:
        return str(self._ser.stopbits)

    @stop_bits.setter
    def stop_bits(self, stop_bits: str) -> None:
        self._ser.stopbits = float(stop_bits)

    @property
    def parity(self) -> str:
        return self._ser.parity

    @parity.setter
    def parity(self, parity: str) -> None:
        self._ser.parity = parity


class PySerial_UARTInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "uart"

    def __init__(self, port: str) -> None:
        self._port = port
        super().__init__()

    def build(self, baudrate: int) -> PySerial_UARTInterface:
        return PySerial_UARTInterface(self._port, baudrate)
