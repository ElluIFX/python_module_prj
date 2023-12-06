import spidev  # type: ignore

from .manager import BaseInterfaceBuilder
from .templates import SPIInterfaceTemplate


class Spidev_SPIInterface(SPIInterfaceTemplate):
    def __init__(self, channel: int, port: int, mode: int, speed_hz: int) -> None:
        self._args = [channel, port]
        self._spi = spidev.SpiDev(channel, port)
        self._spi.mode = mode
        self._spi.max_speed_hz = speed_hz
        return super().__init__()

    @property
    def mode(self) -> int:
        return self._spi.mode

    @mode.setter
    def mode(self, mode: int) -> None:
        self._spi.mode = mode

    @property
    def speed_hz(self) -> float:
        return self._spi.max_speed_hz

    @speed_hz.setter
    def speed_hz(self, speed_hz: int) -> None:
        self._spi.max_speed_hz = speed_hz

    @property
    def byteorder(self) -> str:
        return "lsb" if self._spi.lsbfirst else "msb"

    @byteorder.setter
    def byteorder(self, byteorder: str) -> None:
        self._spi.lsbfirst = byteorder == "lsb"

    @property
    def bits_per_word(self) -> int:
        return self._spi.bits_per_word

    @bits_per_word.setter
    def bits_per_word(self, bits_per_word: int) -> None:
        self._spi.bits_per_word = bits_per_word

    def write(self, data: bytes) -> None:
        self._spi.writebytes2(data)

    def read(self, length: int) -> bytes:
        return self._spi.readbytes(length)

    def transfer(self, data: bytes) -> bytes:
        return self._spi.xfer3(data)

    def close(self) -> None:
        self._spi.close()

    def reopen(self) -> None:
        assert not self._destroyed, "Interface has been destroyed"
        self._spi.open(*self._args)


class Spidev_SPIInterfaceBuilder(BaseInterfaceBuilder):
    dev_type = "spi"

    def __init__(self, channel: int, port: int) -> None:
        self._channel = channel
        self._port = port

    def build(self, mode: int, speed_hz: int) -> Spidev_SPIInterface:
        return Spidev_SPIInterface(self._channel, self._port, mode, speed_hz)
