from typing import List, Literal, Tuple

from .interface import InterfaceManager


class MCP3421:
    """
    MCP3421 18bit ADC driver
    """

    def __init__(self, address=0x68) -> None:
        self._bus = InterfaceManager.request_i2c_interface("MCP3421", address)
        self._calibration_factor = 1
        self._calibration_offset = 0
        self._set_configuration(1, 1, 3, 0)  # continuous conversion, 18bit, PGA=1x

    def calc_calibration_factor(self, voltage_pairs: List[Tuple[float, float]]):
        """
        Calculate calibration factor using polynomial regression
        voltage_pairs: [(voltage_read, voltage_true), ...]
        """
        import numpy as np
        from scipy.optimize import curve_fit

        x = np.array([v[0] for v in voltage_pairs])
        y = np.array([v[1] for v in voltage_pairs])

        def func(x, a, b):
            return a * x + b

        popt, pcov = curve_fit(func, x, y)
        self._calibration_factor = popt[0]
        self._calibration_offset = popt[1]
        return (self._calibration_factor, self._calibration_offset)

    def set_calibration_factor(self, factor: float, offset: float):
        """
        Set calibration factor
        Vtrue = Vread * factor + offset
        """
        self._calibration_factor = factor
        self._calibration_offset = offset

    def _set_configuration(self, RDY: int, OC: int, SMP: int, PGA: int):
        """
        Set configuration
        """
        assert 0 <= RDY <= 1
        assert 0 <= OC <= 1
        assert 0 <= SMP <= 3
        assert 0 <= PGA <= 3
        self._bus.write_raw_byte((RDY << 7) | (OC << 4) | (SMP << 2) | (PGA << 0))
        self._oc = OC
        self._smp = SMP
        self._pga = PGA

    def set_resolution(self, resolution: Literal[12, 14, 16, 18]):
        """
        Set resolution
        12bit: 240SPS
        14bit: 60SPS
        16bit: 15SPS
        18bit: 3.75SPS
        """
        smp = {12: 0, 14: 1, 16: 2, 18: 3}
        self._set_configuration(0, self._oc, smp[resolution], self._pga)

    def set_pga(self, pga_gain: Literal[1, 2, 4, 8]):
        """
        Set PGA Gain
        Max Input = 2.048V * pga_gain
        """
        pga = {1: 0, 2: 1, 4: 2, 8: 3}
        self._set_configuration(0, self._oc, self._smp, pga[pga_gain])

    def set_mode(self, mode: Literal["continuous", "single"]):
        """
        Set mode
        In single mode, call trigger_single_conversion() to start conversion
        """
        if mode == "continuous":
            self._set_configuration(0, 1, self._smp, self._pga)
        else:
            self._set_configuration(0, 0, self._smp, self._pga)

    def trigger_single_conversion(self):
        """
        Trigger single conversion
        """
        self._set_configuration(1, self._oc, self._smp, self._pga)

    @property
    def standardized(self) -> float:
        """
        Read standardized value in -1.0 - 1.0
        value depends on resolution
        """
        if self._smp == 3:
            msg = self._bus.new_msg().read(3)
        else:
            msg = self._bus.new_msg().read(2)
        self._bus.transfer_msg([msg])
        data = bytes(msg)
        msb = data[0] >> 7
        if self._smp == 3:
            lsb = ((data[0] & 0x01) << 16) | (data[1] << 8) | data[2]
            lsb_range = 2**17
        elif self._smp == 2:
            lsb = ((data[0] & 0x7F) << 8) | data[1]
            lsb_range = 2**15
        elif self._smp == 1:
            lsb = ((data[0] & 0x1F) << 8) | data[1]
            lsb_range = 2**13
        else:
            lsb = ((data[0] & 0x07) << 8) | data[1]
            lsb_range = 2**11
        if msb:  # take 2's complement
            lsb -= lsb_range
        return lsb / lsb_range

    @property
    def voltage(self) -> float:
        """
        Read voltage (considering: PGA gain, Resolution, Calibration)
        """
        _pga_vref = {0: 2.048, 1: 1.024, 2: 0.512, 3: 0.256}
        return (
            self._calibration_factor * self.standardized * _pga_vref[self._pga]
            + self._calibration_offset
        )

    def global_reset(self):
        """
        Reset all devices on the bus
        """
        addr = self._bus.address
        self._bus.address = 0x00
        self._bus.write_raw_byte(0x06)
        self._bus.address = addr

    def global_conversion(self):
        """
        Trigger conversion on all devices on the bus
        """
        addr = self._bus.address
        self._bus.address = 0x00
        self._bus.write_raw_byte(0x08)
        self._bus.address = addr
