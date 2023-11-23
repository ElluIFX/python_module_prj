import time
from struct import unpack

from .interface import request_interface

# BMP280 default address.
BMP280_I2CADDR = 0x77

# Operating Modes
BMP280_ULTRALOWPOWER = 0
BMP280_STANDARD = 1
BMP280_HIGHRES = 2
BMP280_ULTRAHIGHRES = 3

# BMP280 Temperature Registers
BMP280_REGISTER_DIG_T1 = 0x88
BMP280_REGISTER_DIG_T2 = 0x8A
BMP280_REGISTER_DIG_T3 = 0x8C
# BMP280 Pressure Registers
BMP280_REGISTER_DIG_P1 = 0x8E
BMP280_REGISTER_DIG_P2 = 0x90
BMP280_REGISTER_DIG_P3 = 0x92
BMP280_REGISTER_DIG_P4 = 0x94
BMP280_REGISTER_DIG_P5 = 0x96
BMP280_REGISTER_DIG_P6 = 0x98
BMP280_REGISTER_DIG_P7 = 0x9A
BMP280_REGISTER_DIG_P8 = 0x9C
BMP280_REGISTER_DIG_P9 = 0x9E

BMP280_REGISTER_CONTROL = 0xF4
# Pressure measurments
BMP280_REGISTER_PRESSUREDATA_MSB = 0xF7
BMP280_REGISTER_PRESSUREDATA_LSB = 0xF8
BMP280_REGISTER_PRESSUREDATA_XLSB = 0xF9
# Temperature measurments
BMP280_REGISTER_TEMPDATA_MSB = 0xFA
BMP280_REGISTER_TEMPDATA_LSB = 0xFB
BMP280_REGISTER_TEMPDATA_XLSB = 0xFC

# Commands
BMP280_READCMD = 0x3F


class BMP280(object):
    def __init__(self, mode=BMP280_STANDARD, address=BMP280_I2CADDR, **kwargs):
        # Check that mode is valid.
        if mode not in [
            BMP280_ULTRALOWPOWER,
            BMP280_STANDARD,
            BMP280_HIGHRES,
            BMP280_ULTRAHIGHRES,
        ]:
            raise ValueError(
                "Unexpected mode value {0}.  Set mode to one of BMP280_ULTRALOWPOWER, BMP280_STANDARD, BMP280_HIGHRES, or BMP280_ULTRAHIGHRES".format(
                    mode
                )
            )
        self._mode = mode
        # Create I2C device.
        self._bus = request_interface("i2c", "BMP280", address)
        # Load calibration values.
        self._load_calibration()
        self._tfine = 0

    def _readU16LE(self, address):
        data = self._bus.read_reg_data(address, 2)
        return unpack("<H", data)[0]

    def _readS16LE(self, address):
        data = self._bus.read_reg_data(address, 2)
        return unpack("<h", data)[0]

    def _readU8(self, address):
        data = self._bus.read_reg_data(address, 1)
        return unpack("<B", data)[0]

    def _writeU8(self, address, val):
        self._bus.write_reg_byte(address, val)

    # reading two bytes of data from each address as signed or unsigned, based on the Bosch docs
    def _load_calibration(self):
        self._cal_REGISTER_DIG_T1 = self._readU16LE(BMP280_REGISTER_DIG_T1)  # UINT16
        self._cal_REGISTER_DIG_T2 = self._readS16LE(BMP280_REGISTER_DIG_T2)  # INT16
        self._cal_REGISTER_DIG_T3 = self._readS16LE(BMP280_REGISTER_DIG_T3)  # INT16
        self._cal_REGISTER_DIG_P1 = self._readU16LE(BMP280_REGISTER_DIG_P1)  # UINT16
        self._cal_REGISTER_DIG_P2 = self._readS16LE(BMP280_REGISTER_DIG_P2)  # INT16
        self._cal_REGISTER_DIG_P3 = self._readS16LE(BMP280_REGISTER_DIG_P3)  # INT16
        self._cal_REGISTER_DIG_P4 = self._readS16LE(BMP280_REGISTER_DIG_P4)  # INT16
        self._cal_REGISTER_DIG_P5 = self._readS16LE(BMP280_REGISTER_DIG_P5)  # INT16
        self._cal_REGISTER_DIG_P6 = self._readS16LE(BMP280_REGISTER_DIG_P6)  # INT16
        self._cal_REGISTER_DIG_P7 = self._readS16LE(BMP280_REGISTER_DIG_P7)  # INT16
        self._cal_REGISTER_DIG_P8 = self._readS16LE(BMP280_REGISTER_DIG_P8)  # INT16
        self._cal_REGISTER_DIG_P9 = self._readS16LE(BMP280_REGISTER_DIG_P9)  # INT16

        # logger.debug("T1 = {0:6d}".format(self._cal_REGISTER_DIG_T1))
        # logger.debug("T2 = {0:6d}".format(self._cal_REGISTER_DIG_T2))
        # logger.debug("T3 = {0:6d}".format(self._cal_REGISTER_DIG_T3))
        # logger.debug("P1 = {0:6d}".format(self._cal_REGISTER_DIG_P1))
        # logger.debug("P2 = {0:6d}".format(self._cal_REGISTER_DIG_P2))
        # logger.debug("P3 = {0:6d}".format(self._cal_REGISTER_DIG_P3))
        # logger.debug("P4 = {0:6d}".format(self._cal_REGISTER_DIG_P4))
        # logger.debug("P5 = {0:6d}".format(self._cal_REGISTER_DIG_P5))
        # logger.debug("P6 = {0:6d}".format(self._cal_REGISTER_DIG_P6))
        # logger.debug("P7 = {0:6d}".format(self._cal_REGISTER_DIG_P7))
        # logger.debug("P8 = {0:6d}".format(self._cal_REGISTER_DIG_P8))
        # logger.debug("P9 = {0:6d}".format(self._cal_REGISTER_DIG_P9))

    # data from the datasheet example, useful for debug
    def _load_datasheet_calibration(self):
        self._cal_REGISTER_DIG_T1 = 27504
        self._cal_REGISTER_DIG_T2 = 26435
        self._cal_REGISTER_DIG_T3 = -1000
        self._cal_REGISTER_DIG_P1 = 36477
        self._cal_REGISTER_DIG_P2 = -10685
        self._cal_REGISTER_DIG_P3 = 3024
        self._cal_REGISTER_DIG_P4 = 2855
        self._cal_REGISTER_DIG_P5 = 140
        self._cal_REGISTER_DIG_P6 = -7
        self._cal_REGISTER_DIG_P7 = 15500
        self._cal_REGISTER_DIG_P8 = -14600
        self._cal_REGISTER_DIG_P9 = 6000

    # reading raw data from registers, and combining into one raw measurment
    def read_raw_temp(self) -> int:
        """Reads the raw (uncompensated) temperature from the sensor."""
        self._writeU8(BMP280_REGISTER_CONTROL, BMP280_READCMD + (self._mode << 6))
        if self._mode == BMP280_ULTRALOWPOWER:
            time.sleep(0.005)
        elif self._mode == BMP280_HIGHRES:
            time.sleep(0.014)
        elif self._mode == BMP280_ULTRAHIGHRES:
            time.sleep(0.026)
        else:
            time.sleep(0.008)
        msb = self._readU8(BMP280_REGISTER_TEMPDATA_MSB)
        lsb = self._readU8(BMP280_REGISTER_TEMPDATA_LSB)
        xlsb = self._readU8(BMP280_REGISTER_TEMPDATA_XLSB)
        raw = ((msb << 8 | lsb) << 8 | xlsb) >> 4
        # logger.debug("Raw temperature 0x{0:04X} ({1})".format(raw & 0xFFFF, raw))
        return raw

    # reading raw data from registers, and combining into one raw measurment
    def read_raw_pressure(self) -> int:
        """Reads the raw (uncompensated) pressure level from the sensor."""
        self._writeU8(BMP280_REGISTER_CONTROL, BMP280_READCMD + (self._mode << 6))
        if self._mode == BMP280_ULTRALOWPOWER:
            time.sleep(0.005)
        elif self._mode == BMP280_HIGHRES:
            time.sleep(0.014)
        elif self._mode == BMP280_ULTRAHIGHRES:
            time.sleep(0.026)
        else:
            time.sleep(0.008)
        msb = self._readU8(BMP280_REGISTER_PRESSUREDATA_MSB)
        lsb = self._readU8(BMP280_REGISTER_PRESSUREDATA_LSB)
        xlsb = self._readU8(BMP280_REGISTER_PRESSUREDATA_XLSB)
        raw = ((msb << 8 | lsb) << 8 | xlsb) >> 4
        # logger.debug("Raw pressure 0x{0:04X} ({1})".format(raw & 0xFFFF, raw))
        return raw

    # applying calibration data to the raw reading
    def read_temperature(self) -> float:
        """Gets the compensated temperature in degrees celsius."""
        adc_T = self.read_raw_temp()
        TMP_PART1 = (
            ((adc_T >> 3) - (self._cal_REGISTER_DIG_T1 << 1))
            * self._cal_REGISTER_DIG_T2
        ) >> 11
        TMP_PART2 = (
            (
                (
                    ((adc_T >> 4) - (self._cal_REGISTER_DIG_T1))
                    * ((adc_T >> 4) - (self._cal_REGISTER_DIG_T1))
                )
                >> 12
            )
            * (self._cal_REGISTER_DIG_T3)
        ) >> 14
        TMP_FINE = TMP_PART1 + TMP_PART2
        self._tfine = TMP_FINE
        temp = ((TMP_FINE * 5 + 128) >> 8) / 100.0
        # logger.debug("Calibrated temperature {0} C".format(temp))
        return temp

    # applying calibration data to the raw reading
    def read_pressure(self) -> float:
        """Gets the compensated pressure in Pascals."""
        # for pressure calculation we need a temperature, checking if we have one, and reading data if not
        if self._tfine == 0:
            self.read_temperature()

        adc_P = self.read_raw_pressure()
        var1 = self._tfine - 128000
        var2 = var1 * var1 * self._cal_REGISTER_DIG_P6
        var2 = var2 + ((var1 * self._cal_REGISTER_DIG_P5) << 17)
        var2 = var2 + ((self._cal_REGISTER_DIG_P4) << 35)
        var1 = ((var1 * var1 * self._cal_REGISTER_DIG_P3) >> 8) + (
            (var1 * self._cal_REGISTER_DIG_P2) << 12
        )
        var1 = (((1) << 47) + var1) * (self._cal_REGISTER_DIG_P1) >> 33
        if var1 == 0:
            return 0

        p = 1048576 - adc_P
        p = int((((p << 31) - var2) * 3125) / var1)
        var1 = ((self._cal_REGISTER_DIG_P9) * (p >> 13) * (p >> 13)) >> 25
        var2 = ((self._cal_REGISTER_DIG_P8) * p) >> 19
        p = ((p + var1 + var2) >> 8) + ((self._cal_REGISTER_DIG_P7) << 4)
        return p / 256.0

    def read_altitude(self, sealevel_pa=101325.0) -> float:
        """Calculates the altitude in meters."""
        pressure = float(self.read_pressure())
        altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0 / 5.255)))
        # logger.debug("Altitude {0} m".format(altitude))
        return altitude

    def read_sealevel_pressure(self, altitude_m=0.0) -> float:
        """Calculates the pressure at sealevel when given a known altitude in
        meters. Returns a value in Pascals."""
        pressure = float(self.read_pressure())
        p0 = pressure / pow(1.0 - altitude_m / 44330.0, 5.255)
        # logger.debug("Sealevel pressure {0} Pa".format(p0))
        return p0
