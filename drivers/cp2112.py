"""Implementation of interface to the CP2112 USB-to-I2C device."""
import hid  # Installed with "pip install hidapi"
from loguru import logger


class I2CError(Exception):
    """Class used for all communication errors on the I2C bus."""

    pass


# Status 1 when Status 0 is 0x03
ErrorStatus = {
    0x00: "NACK received",
    0x01: "Bus not free (SCL low)",
    0x02: "Arbitration lost",
    0x03: "Read incomplete",
    0x04: "Write incomplete",
    0x05: "Succeeded after Status 2 retries",
}

VENDOR_ID = 0x10C4
PRODUCT_ID = 0xEA90


class CP2112:
    """Class for interfacing with the CP2112 USB-to-I2C device through the HID USB driver."""

    def __init__(self, clock=400000, serial_number=None, retry=3, txrx_leds=True):
        """Constructor creates a new CP2112 object.
        Args:
            clock: I2C clock in Hz.
            serial_number: Optional serial number as a text string.
            retry: Number of retry attempts. 0 means infinite.
        Throws:
            OSError: When open fails.
        """
        self.handle = hid.device()
        self.handle.open(VENDOR_ID, PRODUCT_ID, serial_number)
        # Set SMBus Configuration
        self.handle.send_feature_report(
            [
                0x06,  # Set SMBus Configuration
                (clock >> 24) & 0xFF,
                (clock >> 16) & 0xFF,
                (clock >> 8) & 0xFF,
                clock & 0xFF,  # Clock in Hz
                0x00,  # Device address (disabled)
                0x00,  # Auto Send Read (disabled)
                0x00,
                0x0F,  # Write Timeout (No Timeout)
                0x00,
                0x0F,  # Read Timeout (No timeout)
                0x01,  # SCL Low Timeout (disabled)
                (retry >> 8) & 0xFF,
                retry & 0xFF,
            ]
        )  # Retry Time (number of retry attempts)
        if txrx_leds:
            self.set_gpio_config(0x03, 0x03, 0x06, 0)
        logger.info(
            f"CP2112 initialized (Serial Number: {self.get_serial_number()}, Version: {self.get_version()})",
        )

    def get_manufacturer(self):
        """Retrieve manufacturer name."""
        return self.handle.get_manufacturer_string()

    def get_product(self):
        """Retrieve product name."""
        return self.handle.get_product_string()

    def get_serial_number(self):
        """Retrieve serial number."""
        return self.handle.get_serial_number_string()

    def get_version(self):
        """Retrieve version. Returns a tuple with Part Number and Device Version."""
        response = self.handle.get_feature_report(0x05, 3)
        return (response[1], response[2])

    def reset(self):
        """Reset the device and re.enumerate it on the USB bus."""
        self.handle.send_feature_report(0x01, [0x01])

    def cancel(self):
        """Cancel an active transfer."""
        self.handle.write([0x17, 0x01])

    def set_gpio_config(self, dir, push_pull, special, clock_divider):
        """Configure all GPIO pins.
        Args:
            dir: Direction; Each bit indicates input (0) or output (1).
            push_pull: Each bit indicates if the pin is output is it open-drain (0) or push-pull(1).
            special: Enable special functions.
                Bit 0: GPIO7_CLK: GPIO pin (0) or Clock output (1)
                Bit 1: GPIO0_TXT: GPIO pin (0) or Tx toggle (1)
                Bit 2: GPIO1_RXT: GPIO pin (0) or Rx toggle (1)
            clock_divider: Only used when GPIO7_CLK is configured as clock output.
                0=48 MHz; Otherwise Clk=(48 MHz)/(2*clock_divider)
        """
        self.handle.send_feature_report([0x02, dir, push_pull, special, clock_divider])
        self.handle.send_feature_report(
            [0x02, dir, push_pull, special, clock_divider]
        )  # send twice to make sure it is set

    def get_gpio_config(self):
        """Return the same GPIO configuration as set with set_gpio_config().
        Returns:
            A tuple (dir, push_pull, special, clock_divider).
        """
        response = self.handle.get_feature_report(0x02, 5)
        return tuple(response[1:5])

    def set_gpio(self, value, mask):
        """Set several GPIO output pins at once.
        Args:
            value: Each bit is a value written to the output pins.
            mask: Each bit indicates if the corresponding bit in value is written to the pin (1) or ignored (0).
        """
        self.handle.send_feature_report([0x04, value, mask])

    def get_gpio(self):
        """Read all GPIO pins.
        Returns:
            Each bit indicates a pin state.
        """
        response = self.handle.get_feature_report(0x03, 2)
        return response[1]

    def set_pin_config(self, pin, dir, push_pull=True, special=False, clock=0):
        """Configure a single pin.
        Args:
            pin: GPIO pin number.
            dir: Direction; False for input, True for output.
            push_pull: False for open-drain, True for push/pull.
            special: False for GPIO pin; True for special purpose:
                GPIO0_TXT: Tx toggle.
                GPIO1_RXT: Rx toggle.
                GPIO7_CLK: Clock output.
            clock: The clock output frequency. Only used when pin=7 and special=True.
                0=48 MHz; Otherwise Clk=(48 MHz)/(2*clock_divider)
        """
        data = self.handle.get_feature_report(0x02, 5)
        mask = 1 << pin
        data[1] = data[1] | mask if dir else data[1] & ~mask
        data[2] = data[2] | mask if push_pull else data[2] & ~mask
        if pin == 0:
            data[3] = data[3] | 0x02 if special else data[3] & ~0x02
        elif pin == 1:
            data[3] = data[3] | 0x04 if special else data[3] & ~0x04
        elif pin == 7:
            data[3] = data[3] | 0x01 if special else data[3] & ~0x01
            data[4] = clock
        self.handle.send_feature_report(data)

    def set_pin(self, pin, value):
        """Set a GPIO pin output state. Only used when the pin is configured as output.
        Args:
            pin: GPIO pin number.
            value: True sets the pin high; False sets the pin low.
        """
        self.set_gpio(value << pin, 1 << pin)

    def get_pin(self, pin):
        """Read a GPIO pin. If it is an output pin, the output state is read.
        Args:
            pin: GPIO pin number.
        Returns:
            True if the pin is high; False if the pin is low.
        """
        return True if self.get_gpio() & (1 << pin) else False

    def _wait_transfer_finish(self):
        """Wait for transfer to finish."""
        try:
            while True:
                self.handle.write([0x15, 0x01])
                response = self.handle.read(7)
                if response[0] == 0x16 and response[1] in [0x02, 0x03]:
                    if response[1] == 0x03:
                        raise I2CError(ErrorStatus[response[2]])
                    return (response[5] << 8) | response[6]
        except Exception:
            self.cancel()
            raise

    def _wait_response(self, id, size):
        """Wait for a specific response.
        Args:
            id: The ID to wait for.
            size: The number of bytes to read.
        """
        while True:
            response = self.handle.read(size)
            if response[0] == id:
                return response

    def write_i2c(self, address, tx_data):
        """Write to the I2C bus.
        Args:
            address: 7-bit I2C slave address.
            tx_data: Data to write (must be convertible to `bytes`). At most 61 bytes can be written.
        """
        # CP2112 cannot write more than 61 bytes
        if not 1 <= len(tx_data) <= 61:
            raise IndexError()
        data = bytes([0x14, address << 1, len(tx_data)]) + bytes(tx_data)
        self.handle.write(data)
        # self._wait_transfer_finish()

    def read_i2c(self, address, rx_size) -> bytes:
        """Read from the I2C bus.
        Args:
            address: 7-bit I2C slave address.
            rx_size: Number of bytes to read from the I2C bus. Maximum 512 bytes can be read.
        Returns:
            list with data received.
        """
        # CP2112 cannot read more than 512 bytes
        if not 1 <= rx_size <= 512:
            raise IndexError()
        self.handle.write([0x10, address << 1, 0x00, rx_size])
        count = self._wait_transfer_finish()
        assert count == rx_size
        self.handle.write([0x12, rx_size])
        response = self._wait_response(0x13, rx_size + 3)
        assert response[2] == rx_size
        return bytes(response[3 : rx_size + 3])

    def write_read_i2c(self, address, tx_data, rx_size) -> bytes:
        """Combined write and read on the I2C bus.
        Args:
            address: 7-bit I2C slave address.
            tx_data: Data to write (must be convertible to `bytes`). At most 61 bytes can be written.
            rx_size: Number of bytes to read from the I2C bus. Maximum 512 bytes can be read.
        Returns:
            list with data received.
        """
        # CP2112 cannot write more than 16 byte and not read more than 512 bytes
        if not 1 <= len(tx_data) <= 16 or not 1 <= rx_size <= 512:
            raise IndexError()
        self.handle.write(
            bytes([0x11, address << 1, rx_size >> 8, rx_size & 0xFF, len(tx_data)])
            + bytes(tx_data)
        )
        count = self._wait_transfer_finish()
        assert count == rx_size
        self.handle.write([0x12, rx_size])
        response = self._wait_response(0x13, rx_size + 3)
        assert response[2] == rx_size
        return bytes(response[3 : rx_size + 3])

    def check_i2c_device(self, address) -> bool:
        """Check if a device is connected to the I2C bus.
        Args:
            address: 7-bit I2C slave address.
        Returns:
            True if the device is connected; False otherwise.
        """
        try:
            self.write_i2c(address, b"\x00")
            self.read_i2c(address, 1)
            return True
        except I2CError:
            return False

    def bus_scan(self):
        """Scan the bus for devices.
        Returns:
            A list of 7-bit addresses corresponding to the devices found.
        """
        addrs = [addr for addr in range(1, 128) if self.check_i2c_device(addr)]
        logger.debug(f"CP2112 bus scan: {[hex(addr) for addr in addrs]}")
        return addrs


if __name__ == "__main__":
    dev = CP2112()
    dev.bus_scan()
