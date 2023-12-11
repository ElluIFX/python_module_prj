import struct
import time
from typing import List, Union

import hid
from loguru import logger


class I2CError(Exception):
    """Class used for all communication errors on the I2C bus."""

    pass


def _reset_error_type(new_type):
    """Reset the error type for I2CError.
    Args:
        new_type: The new error type.
    Returns:
        The previous error type.
    """
    global I2CError
    I2CError = new_type


# Status 1 when Status 0 is 0x03
ErrorStatus = {
    0x00: "Timeout address NACKed (no device)",
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

    def __init__(self, clock=400000, retry=3, txrx_leds=True, serial_number=None):
        """Constructor creates a new CP2112 object.
        Args:
            clock: I2C clock in Hz.
            retry: Number of retry attempts.
            txrx_leds: Enable Tx/Rx LEDs on GPIO0 and GPIO1.
            serial_number: Optional serial number as a text string.
        Throws:
            OSError: When open fails.
        """
        self.handle = hid.device()
        self.handle.open(VENDOR_ID, PRODUCT_ID, serial_number)
        self.set_smbus_config(clock, 0x00, False, 10, 10, True, retry)
        if txrx_leds:
            self.set_gpio_config(0x03, 0x03, 0x06, 0)
        else:
            self.set_gpio_config(0x00, 0x00, 0x00, 0)
        logger.info(
            f"CP2112 initialized (Serial Number: {self.serial_number}, Version: {self.version})",
        )

    def close(self):
        """Close the device."""
        self.handle.close()

    @property
    def manufacturer(self) -> str:
        """Retrieve manufacturer name."""
        return self.handle.get_manufacturer_string()

    @property
    def product(self) -> str:
        """Retrieve product name."""
        return self.handle.get_product_string()

    @property
    def serial_number(self) -> str:
        """Retrieve serial number."""
        return self.handle.get_serial_number_string()

    @property
    def version(self) -> tuple[int, int]:
        """Retrieve version. Returns a tuple with Part Number and Device Version."""
        response = self.handle.get_feature_report(0x05, 3)
        return (int(response[1]), int(response[2]))

    def reset(self):
        """Reset the device and re.enumerate it on the USB bus."""
        self.handle.send_feature_report(0x01, [0x01])

    def cancel_transfer(self):
        """Cancel an active transfer."""
        self.handle.write([0x17, 0x01])

    def set_gpio_config(
        self, dir: int, push_pull: int, special: int, clock_divider: int
    ):
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

    def get_gpio_config(self) -> tuple[int, ...]:
        """Return the same GPIO configuration as set with set_gpio_config().
        Returns:
            A tuple (dir, push_pull, special, clock_divider).
        """
        response = self.handle.get_feature_report(0x02, 5)
        return tuple(int(x) for x in response[1:5])

    def set_gpio(self, value: int, mask: int):
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

    def get_pin(self, pin) -> bool:
        """Read a GPIO pin. If it is an output pin, the output state is read.
        Args:
            pin: GPIO pin number.
        Returns:
            True if the pin is high; False if the pin is low.
        """
        return True if self.get_gpio() & (1 << pin) else False

    def set_smbus_config(
        self,
        clock_speed: int,
        device_address: int,
        auto_send_read: bool,
        write_timeout: int,
        read_timeout: int,
        scl_low_timeout: bool,
        retry_time: int,
    ):
        """Set SMBus configuration.

        Args:
            clock_speed: SCL clock speed in Hz.
            device_address: 7-bit I2C slave address, device will ACK this address but not support any R/W operation.
            auto_send_read: Auto send read interrupt report after read transfer complete.
            write_timeout: Timeout for write transfer in milliseconds, Max 1000.
            read_timeout: Timeout for read transfer in milliseconds, Max 1000.
            scl_low_timeout: SCL keep low 25ms will issue timeout error and reset SMBus.
            retry_time: Retry attempts before auto cancel transfer, Max 1000.
        """
        data = [
            0x06,  # Set SMBus Configuration
            (clock_speed >> 24) & 0xFF,
            (clock_speed >> 16) & 0xFF,
            (clock_speed >> 8) & 0xFF,
            clock_speed & 0xFF,  # Clock in Hz, MSB first
            device_address << 1,  # Device address
            0x01 if auto_send_read else 0x00,  # Auto Send Read
            (write_timeout >> 8) & 0xFF,
            write_timeout & 0xFF,  # Write Timeout in ms
            (read_timeout >> 8) & 0xFF,
            read_timeout & 0xFF,  # Read Timeout in ms
            0x01 if scl_low_timeout else 0x00,  # SCL Low Timeout
            (retry_time >> 8) & 0xFF,
            retry_time & 0xFF,  # Retry Time
        ]
        self.handle.send_feature_report(data)

    def get_smbus_config(self):
        """Get SMBus configuration.

        Returns:
            A tuple (clock_speed, device_address, auto_send_read, write_timeout, read_timeout, scl_low_timeout, retry_time).
        """
        response = self.handle.get_feature_report(0x06, 14)
        return struct.unpack(">IB?HH?H", bytes(response[1:14]))

    def _wait_transfer_finish_response(self) -> int:
        """Wait for transfer to finish.

        Returns:
            The number of bytes received.
        """
        t0 = time.perf_counter()
        try:
            while True:
                self.handle.write([0x15, 0x01])
                response = self.handle.read(7)
                if response[0] == 0x16 and response[1] in [0x02, 0x03]:
                    if response[1] == 0x03:
                        raise I2CError(ErrorStatus[response[2]])
                    return (response[5] << 8) | response[6]
                if time.perf_counter() - t0 > 1:
                    raise I2CError("Wait transfer response timeout")
        except Exception as e:
            self.cancel_transfer()
            raise e

    def _get_transfer_response(self, size: int) -> bytes:
        """Get transfer response data.

        Args:
            size: The number of bytes to read.
        """
        t0 = time.perf_counter()
        while True:
            response = self.handle.read(size)
            if response[0] == 0x13:
                return bytes(response)
            if time.perf_counter() - t0 > 1:
                raise I2CError("Get transfer response timeout")

    def write_i2c(self, address: int, tx_data: Union[bytes, List[int]]):
        """Write to the I2C bus.
        Args:
            address: 7-bit I2C slave address.
            tx_data: Data to write (must be convertible to `bytes`). Maximum 61 bytes can be written.
        """
        # CP2112 cannot write more than 61 bytes
        if not 1 <= len(tx_data) <= 61:
            raise IndexError()
        data = bytes([0x14, address << 1, len(tx_data)]) + bytes(tx_data)
        self.handle.write(data)
        self._wait_transfer_finish_response()

    def read_i2c(
        self,
        address: int,
        rx_size: int,
        ignore_size_mismatch: bool = False,
    ) -> bytes:
        """Read from the I2C bus.
        Args:
            address: 7-bit I2C slave address.
            rx_size: Number of bytes to read from the I2C bus. Maximum 512 bytes can be read.
            ignore_size_mismatch: If True, the function will not throw an exception if the transfered data less than rx_size
        Returns:
            bytes data received.
        """
        # CP2112 cannot read more than 512 bytes
        if not 1 <= rx_size <= 512:
            raise IndexError()
        self.handle.write([0x10, address << 1, (rx_size >> 8) & 0xFF, rx_size & 0xFF])
        count = self._wait_transfer_finish_response()
        if not ignore_size_mismatch and count != rx_size:
            raise I2CError(f"Transfer size mismatch: {count} != {rx_size}")
        self.handle.write([0x12, rx_size])
        response = self._get_transfer_response(count + 3)
        if ((response[1] << 8) | response[2]) != count:
            raise I2CError(f"Response size mismatch: {response[2]} != {count}")
        return bytes(response[3 : count + 3])

    def write_read_i2c(
        self,
        address: int,
        tx_data: Union[bytes, List[int]],
        rx_size: int,
        ignore_size_mismatch: bool = False,
    ) -> bytes:
        """Combined write and read on the I2C bus.
        Args:
            address: 7-bit I2C slave address.
            tx_data: Data to write (must be convertible to `bytes`). Maximum 16 bytes can be written.
            rx_size: Number of bytes to read from the I2C bus. Maximum 512 bytes can be read.
            ignore_size_mismatch: If True, the function will not throw an exception if the transfered data less than rx_size
        Returns:
            bytes data received.
        """
        if not 1 <= len(tx_data) <= 16 or not 1 <= rx_size <= 512:
            raise ValueError(
                "write_read_i2c() only supports 1-16 bytes write and 1-512 bytes read"
            )
        self.handle.write(
            bytes([0x11, address << 1, rx_size >> 8, rx_size & 0xFF, len(tx_data)])
            + bytes(tx_data)
        )
        count = self._wait_transfer_finish_response()
        if not ignore_size_mismatch and count != rx_size:
            raise I2CError(f"Transfer size mismatch: {count} != {rx_size}")
        self.handle.write([0x12, rx_size])
        response = self._get_transfer_response(count + 3)
        if ((response[1] << 8) | response[2]) != count:
            raise I2CError(f"Response size mismatch: {response[2]} != {count}")
        return bytes(response[3 : count + 3])

    def check_i2c_device(self, address) -> bool:
        """Check if a device is connected to the I2C bus.
        Args:
            address: 7-bit I2C slave address.
        Returns:
            True if the device is connected; False otherwise.
        """
        try:
            try:
                self.write_i2c(address, b"\x00")
            except I2CError:
                pass
            self.read_i2c(address, 1)
            return True
        except I2CError:
            return False

    def bus_scan(self) -> list[int]:
        """Scan the bus for devices.
        Returns:
            A list of 7-bit addresses corresponding to the devices found.
        """
        addrs = [addr for addr in range(1, 128) if self.check_i2c_device(addr)]
        logger.debug(f"CP2112 bus scan result: {[hex(addr) for addr in addrs]}")
        return addrs


if __name__ == "__main__":
    dev = CP2112()
    dev.bus_scan()
