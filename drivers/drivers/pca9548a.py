import logging
import time

from smbus2 import SMBus


class PCA9548A:
    # I2C communication driver for PCA9548A, using only smbus2

    def __init__(self, BusNum=3, Addr=0x70):
        # Initialize PCA9548A
        self.BusNum = BusNum
        self.Addr = Addr
        self._cmd_reset()
        logging.debug(f"PCA9548A: Initialized on bus {BusNum} at address {Addr:02x}")

    def _cmd_reset(self):
        # Send the command to reset
        with SMBus(self.BusNum) as i2c_bus:
            i2c_bus.write_i2c_block_data(self.Addr, 0x0, [0x0])
        time.sleep(0.01)  # Wait 10 ms after reset

    def switch(self, channel, check=True):
        # Send the command to switch to a channel
        assert channel in list(range(8))
        with SMBus(self.BusNum) as i2c_bus:
            i2c_bus.write_byte(self.Addr, 0x01 << channel)
        logging.debug(f"PCA9548A: Switched to channel {channel}")
        time.sleep(0.01)
        if check and not self.get_enabled()[channel]:
            logging.warning(f"PCA9548A: Failed to switch to channel {channel}, retry")
            time.sleep(0.1)
            self.switch(channel, check=True)

    def enable(self, channels: list):
        # Send the command to enable a channel
        with SMBus(self.BusNum) as i2c_bus:
            data = i2c_bus.read_byte(self.Addr)
        for channel in channels:
            assert channel in list(range(8))
            data |= 0x01 << channel
        time.sleep(0.01)
        with SMBus(self.BusNum) as i2c_bus:
            i2c_bus.write_byte(self.Addr, data)
        logging.debug(f"PCA9548A: Enabled channels {channels}")
        time.sleep(0.01)

    def disable(self, channels: list):
        # Send the command to disable a channel
        with SMBus(self.BusNum) as i2c_bus:
            data = i2c_bus.read_byte(self.Addr)
        for channel in channels:
            assert channel in list(range(8))
            data &= ~(0x01 << channel)
        time.sleep(0.01)
        with SMBus(self.BusNum) as i2c_bus:
            i2c_bus.write_byte(self.Addr, data)
        logging.debug(f"PCA9548A: Disabled channels {channels}")
        time.sleep(0.01)

    def get_enabled(self):
        # Get the enabled channel
        with SMBus(self.BusNum) as i2c_bus:
            status = i2c_bus.read_byte(self.Addr)
        enabled = [status >> i & 0x01 == 0x01 for i in range(8)]
        return enabled

    def disable_all(self):
        # Send the command to disable all channels
        self._cmd_reset()
        logging.debug(f"PCA9548A: Disabled all channels")
