import time

from luma.core.interface import serial  # noqa: F401

from .interface import request_interface


class i2c:
    def __init__(self, address=0x3C):
        self._cmd_mode = 0x00
        self._data_mode = 0x40

        try:
            self._addr = int(str(address), 0)
        except ValueError:
            raise ValueError(f"I2C device address invalid: {address}")

        self._bus = request_interface("i2c", "luma.core", address)
        self._managed = True

    def command(self, *cmd):
        self._bus.write_reg_data(self._cmd_mode, list(cmd))

    def data(self, data):
        # block size is the maximum data payload that will be tolerated.
        # The managed i2c will transfer blocks of upto 4K (using i2c_rdwr)
        # whereas we must use the default 32 byte block size when unmanaged
        block_size = self._bus.max_transfer_size
        if len(data) > block_size:
            i = 0
            n = len(data)
            msgs = []
            while i < n:
                msgs.append(
                    self._bus.new_msg.write(
                        [self._data_mode] + list(data[i : i + block_size])
                    )
                )
                i += block_size
            self._bus.exchange_msgs(msgs)
        else:
            self._bus.write_reg_data(self._data_mode, data)

    def cleanup(self):
        self._bus.destroy()


class spi:
    def __init__(self, mode=0, speed_hz=10_000_000):
        self._spi = request_interface("spi", "luma.core", mode, speed_hz)
        self._gpio = request_interface("gpio", "luma.core")
        self._rst = self._gpio.get_pin("RST")
        self._dc = self._gpio.get_pin("DC")
        self._rst.set_mode("output_push_pull")
        self._dc.set_mode("output_push_pull")
        self._rst.write(True)
        self._dc.write(True)
        time.sleep(0.01)
        self._rst.write(False)
        time.sleep(0.05)
        self._rst.write(True)
        time.sleep(0.1)

    def command(self, data):
        self._dc.write(False)
        self._spi.write(bytes(data))
        self._dc.write(True)

    def data(self, data):
        self._spi.write(bytes(data))

    def cleanup(self):
        self._spi.destroy()
        self._gpio.destroy()
