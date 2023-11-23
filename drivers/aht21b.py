import time

from .interface import request_interface


class AHT21B:
    """
    AHT21B environmental sensor
    """

    def __init__(self) -> None:
        self._bus = request_interface("i2c","AHT21B", 0x38)
        self._temperature = 0.0
        self._humidity = 0.0
        status = self._bus.read_reg_byte(0x71)
        assert status & 0x18 == 0x18, "AHT21B not ready"

    def start_measure_no_block(self) -> None:
        """
        Start measure and return immediately
        """
        self._bus.write_reg_data(0xAC, [0x33, 0x00])

    @property
    def data_ready(self) -> bool:
        """
        Whether data is ready
        """
        status = self._bus.read_raw_byte()
        return status & 0x80 == 0

    def prepare_data(self) -> None:
        """
        Prepare data for reading
        Call this function after data is ready
        """
        rd = self._bus.new_msg.read(6)  # crc byte is ignored
        self._bus.transfer_msg([rd])
        data = bytes(rd)
        raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        raw_humidity = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4)
        self._temperature = (raw_temperature / 1048576) * 200 - 50
        self._humidity = (raw_humidity / 1048576) * 100

    def measure(self):
        """
        Measure temperature and humidity
        (blocking)
        """
        self.start_measure_no_block()
        while True:
            time.sleep(0.01)
            if self.data_ready:
                break
        self.prepare_data()

    @property
    def temperature(self) -> float:
        """
        Temperature in Celsius
        """
        return self._temperature

    @property
    def humidity(self) -> float:
        """
        Relative humidity in %
        """
        return self._humidity
