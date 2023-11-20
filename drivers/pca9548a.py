import time

from loguru import logger

from .interface import InterfaceManager


class PCA9548A:
    def __init__(self, Addr=0x70):
        # Initialize PCA9548A
        self._dev = InterfaceManager.request_i2c_interface("PCA9548A", Addr)
        self._cmd_reset()
        logger.debug(f"PCA9548A: Initialized at address {Addr:02x}")

    def _cmd_reset(self):
        # Send the command to reset
        self._dev.write_byte(0x0, 0x0)
        time.sleep(0.01)  # Wait 10 ms after reset

    def switch(self, channel, check=True):
        # Send the command to switch to a channel
        assert channel in list(range(8))
        self._dev.write_raw_byte(0x01 << channel)
        logger.debug(f"PCA9548A: Switched to channel {channel}")
        time.sleep(0.01)
        if check and not self.get_enabled()[channel]:
            logger.warning(f"PCA9548A: Failed to switch to channel {channel}, retry")
            time.sleep(0.1)
            self.switch(channel, check=True)

    def enable(self, channels: list):
        # Send the command to enable a channel
        data = self._dev.read_raw_byte()
        for channel in channels:
            assert channel in list(range(8))
            data |= 0x01 << channel
        time.sleep(0.01)
        self._dev.write_raw_byte(data)
        logger.debug(f"PCA9548A: Enabled channels {channels}")
        time.sleep(0.01)

    def disable(self, channels: list):
        # Send the command to disable a channel
        data = self._dev.read_raw_byte()
        for channel in channels:
            assert channel in list(range(8))
            data &= ~(0x01 << channel)
        time.sleep(0.01)
        self._dev.write_raw_byte(data)
        logger.debug(f"PCA9548A: Disabled channels {channels}")
        time.sleep(0.01)

    def get_enabled(self):
        # Get the enabled channel
        status = self._dev.read_raw_byte()
        enabled = [status >> i & 0x01 == 0x01 for i in range(8)]
        return enabled

    def disable_all(self):
        # Send the command to disable all channels
        self._cmd_reset()
        logger.debug("PCA9548A: Disabled all channels")
