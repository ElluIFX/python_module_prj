from .. import request_interface


class PCAL6416A:
    """
    PCAL6416A 16-bit I2C-bus I/O expander
    """

    def __init__(self, address: int = 0x21, cache_reg: bool = True) -> None:
        """
        Initialize PCAL6416A.

        Args:
            address: Chip I2C Address.
            cache_reg: Enable data caching when modifying for non-input-sensitive registers.
        """
        self._bus = request_interface("i2c", "PCAL6416A", address)
        self._cache_reg = cache_reg
        self._cache = [0] * 24
        if self._cache_reg:
            self._init_cache()

    def _init_cache(self) -> None:
        self._cache = []
        for reg in range(0x00, 0x08):
            self._cache.append(self._bus.read_reg_byte(reg))
        for reg in range(0x40, 0x50):
            if reg == 0x4E:  # not exist
                self._cache.append(0)
            else:
                self._cache.append(self._bus.read_reg_byte(reg))
        assert len(self._cache) == 24, "Cache initialization failed"

    def _remap(self, reg: int) -> int:
        if reg < 0x08:
            return reg
        elif reg >= 0x40:
            return reg + 0x08 - 0x40
        else:
            raise ValueError("Invalid register address")

    def _read_reg(self, reg: int) -> int:
        if self._cache_reg:
            return self._cache[self._remap(reg)]
        return self._bus.read_reg_byte(reg)

    def _write_reg(self, reg: int, value: int) -> None:
        if self._cache_reg:
            self._cache[self._remap(reg)] = value
        self._bus.write_reg_byte(reg, value)

    def _modify_reg(self, reg: int, mask: int, value: int) -> None:
        self._write_reg(reg, (self._read_reg(reg) & ((~mask) & 0xFF)) | value)

    @property
    def cache_reg(self) -> bool:
        """
        Get register caching status.
        """
        return self._cache_reg

    @cache_reg.setter
    def cache_reg(self, value: bool) -> None:
        """
        Set register caching status.
        """
        self._cache_reg = value
        if self._cache_reg:
            self._init_cache()

    def read_port(self, port: int) -> int:
        """
        Read input value of all pins on a port.

        Args:
            port: Port number (0/1)

        Returns:
            Pin0-7 in a byte. 1 for high, 0 for low.
        """
        return self._bus.read_reg_byte(0x00 + port)

    def read(self, port: int, pin: int) -> bool:
        """
        Read input value of a pin.

        Args:
            port: Port number (0/1)
            pin: pin number.

        Returns:
            Pin level.
        """
        return self.read_port(port) & (1 << pin) != 0

    def write_port(self, port: int, value: int) -> None:
        """
        Write output value of all pins on a port.
        After-reset: 0xFF

        Args:
            port: Port number (0/1)
            value: Pin0-7 in a byte. 1 for high, 0 for low.
        """
        self._write_reg(0x02 + port, value)

    def write(self, port: int, pin: int, level: bool) -> None:
        """
        Write output value of a pin.
        After-reset: True (high level)

        Args:
            port: Port number (0/1)
            pin: Pin number.
            level: Pin level.
        """
        self._modify_reg(0x02 + port, 1 << pin, int(level) << pin)

    def set_polarity_inversion_port(self, port: int, value: int) -> None:
        """
        Set polarity inversion of all pins on a port.
        Only effects input registers.
        After-reset: 0x00

        Args:
            port: Port number (0/1)
            value: Pin0-7 in a byte. 1 for inverted, 0 for normal.
        """
        self._write_reg(0x04 + port, value)

    def set_polarity_inversion(self, port: int, pin: int, enable: bool) -> None:
        """
        Set polarity inversion of a pin.
        Only effects input registers.
        After-reset: False

        Args:
            port: Port number (0/1)
            pin: Pin number.
            enable: Enable polarity inversion.
        """
        self._modify_reg(0x04 + port, 1 << pin, int(enable) << pin)

    def set_direction_port(self, port: int, value: int) -> None:
        """
        Set direction of all pins on a port.
        After-reset: 0xFF

        Args:
            port: Port number (0/1)
            value: Pin0-7 in a byte. 1 for hi-z input, 0 for output.
        """
        self._write_reg(0x06 + port, value)

    def set_direction(self, port: int, pin: int, as_input: bool) -> None:
        """
        Set direction of a pin.
        After-reset: True (hi-z input)

        Args:
            port: Port number (0/1)
            pin: Pin number.
            as_input: Set as a hi-z input.
        """
        self._modify_reg(0x06 + port, 1 << pin, int(as_input) << pin)

    def set_output_drive_strength_port(self, port: int, value: int) -> None:
        """
        Set output drive strength of all pins on a port.
        After-reset: 0xFFFF

        Args:
            port: Port number (0/1)
            value: Pin0-7 in a 16-bit word. 0 for 0.25x, 1 for 0.5x, 2 for 0.75x, 3 for 1x.
        """
        self._write_reg(0x40 + port * 2, value & 0xFF)
        self._write_reg(0x41 + port * 2, (value >> 8) & 0xFF)

    def set_output_drive_strength(self, port: int, pin: int, strength: int) -> None:
        """
        Set output drive strength of a pin.
        After-reset: 3 (1x)

        Args:
            port: Port number (0/1)
            pin: Pin number.
            strength: 0 for 0.25x, 1 for 0.5x, 2 for 0.75x, 3 for 1x.
        """
        assert 0 <= strength <= 3, "strength must be in range [0, 3]"
        self._modify_reg(
            0x40 + port * 2 + pin // 4,
            0x03 << ((pin % 4) * 2),
            strength << ((pin % 4) * 2),
        )

    def set_input_interrupt_latch_port(self, port: int, value: int) -> None:
        """
        Set input interrupt latch of all pins on a port.
        After-reset: 0x00

        Args:
            port: Port number (0/1)
            value: Pin0-7 in a byte. 1 for enable, 0 for disable.
        """
        self._write_reg(0x44 + port, value)

    def set_input_interrupt_latch(self, port: int, pin: int, enable: bool) -> None:
        """
        Set input interrupt latch of a pin.
        Latch menas the interrupt source register will keep 1 even if the pin is back to original level.
        In latch mode, the input level register will keep level which it was when interrupt occured until it is read.
        After-reset: False

        Args:
            port: Port number (0/1)
            pin: Pin number.
            enable: Enable input interrupt latch.
        """
        self._modify_reg(0x44 + port, 1 << pin, int(enable) << pin)

    def set_pull_mode_port(self, port: int, en_value: int, sel_value: int) -> None:
        """
        Set pull mode of all pins on a port.
        After-reset: 0x00 0x00 (disable pullup/pulldown)

        Args:
            port: Port number (0/1)
            en_value: Pin0-7 in a byte. 1 for enable, 0 for disable.
            sel_value: Pin0-7 in a byte. 1 for pullup, 0 for pulldown.
        """
        self._write_reg(0x46 + port, en_value)
        self._write_reg(0x48 + port, sel_value)

    def set_pull_mode(self, port: int, pin: int, enable: bool, pullup: bool) -> None:
        """
        Set pull mode of a pin.
        After-reset: False False (disable pullup/pulldown)

        Args:
            port: Port number (0/1)
            pin: Pin number.
            enable: 1 for enable, 0 for disable.
            pullup: 1 for pullup, 0 for pulldown.
        """
        self._modify_reg(0x46 + port, 1 << pin, int(enable) << pin)
        self._modify_reg(0x48 + port, 1 << pin, int(pullup) << pin)

    def set_input_interrupt_enable_port(self, port: int, value: int) -> None:
        """
        Set input interrupt enable of all pins on a port.
        After-reset: 0x00 (disable all interrupt)

        Args:
            port: Port number (0/1)
            value: Pin0-7 in a byte. 1 for enable, 0 for disable.
        """
        self._write_reg(
            0x4A + port, (~value) & 0xFF
        )  # reg is 'mask', so 0 means enable

    def set_input_interrupt_enable(self, port: int, pin: int, enable: bool) -> None:
        """
        Set input interrupt enable of a pin.
        After-reset: False (disable interrupt)

        Args:
            port: Port number (0/1)
            pin: Pin number.
            enable: Enable input interrupt.
        """
        self._modify_reg(0x4A + port, 1 << pin, int(not enable) << pin)

    def get_input_interrupt_source(self) -> list[int]:
        """
        Get input interrupt source.

        Returns:
            List of pin numbers which triggered interrupt.
            (0-7: Port0, 8-15: Port1)
        """
        data = self._bus.read_reg_data(0x4C, 2)
        value = data[0] | (data[1] << 8)
        ret = []
        for pin in range(16):
            if value & (1 << pin):
                ret.append(pin)
        return ret

    def set_output_opendrain(self, port: int, is_opendrain: bool) -> None:
        """
        Set output mode of a port to opendrain.
        All pins on the same port share the same setting.
        After-reset: False (push-pull)

        Args:
            port: Port number (0/1)
            is_opendrain: True for opendrain, False for push-pull.
        """
        self._modify_reg(0x4F, 1 << port, int(is_opendrain) << port)
