import time
from struct import pack, unpack

from .interface import request_interface

REF_REG_0 = 0xC0
REF_REG_0_VAL = 0xEE
REF_REG_1 = 0xC1
REF_REG_1_VAL = 0xAA
REF_REG_2 = 0xC2
REF_REG_2_VAL = 0x10
REF_REG_3 = 0x51
REF_REG_3_VAL = 0x0099
REF_REG_4 = 0x61
REF_REG_4_VAL = 0x0000

VL53L0X_GPIO_FUNC_OFF = 0
VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_LOW = 1
VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_HIGH = 2
VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_OUT = 3
VL53L0X_GPIO_FUNC_NEW_MEASURE_READY = 4


SYSRANGE_START = 0x00
SYSTEM_THRESH_HIGH = 0x0C
SYSTEM_THRESH_LOW = 0x0E
SYSTEM_SEQUENCE_CONFIG = 0x01
SYSTEM_RANGE_CONFIG = 0x09
SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
GPIO_HV_MUX_ACTIVE_HIGH = 0x84
SYSTEM_INTERRUPT_CLEAR = 0x0B
RESULT_INTERRUPT_STATUS = 0x13
RESULT_RANGE_STATUS = 0x14
RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC
RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0
RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0
RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4
RESULT_PEAK_SIGNAL_RATE_REF = 0xB6
ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28
I2C_SLAVE_DEVICE_ADDRESS = 0x8A
MSRC_CONFIG_CONTROL = 0x60
PRE_RANGE_CONFIG_MIN_SNR = 0x27
PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56
PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57
PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64
FINAL_RANGE_CONFIG_MIN_SNR = 0x67
FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47
FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48
FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0
PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61
PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62
PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52
SYSTEM_HISTOGRAM_BIN = 0x81
HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33
HISTOGRAM_CONFIG_READOUT_CTRL = 0x55
FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72
CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20
MSRC_CONFIG_TIMEOUT_MACROP = 0x46
SOFT_RESET_GO2_SOFT_RESET_N = 0xBF
IDENTIFICATION_MODEL_ID = 0xC0
IDENTIFICATION_REVISION_ID = 0xC2
OSC_CALIBRATE_VAL = 0xF8
GLOBAL_CONFIG_VCSEL_WIDTH = 0x32
GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1
GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2
GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3
GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4
GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5
GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80
VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89
ALGO_PHASECAL_LIM = 0x30
ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30

RANGE_SEQUENCE_STEP_TCC = 0x10
RANGE_SEQUENCE_STEP_MSRC = 0x04
RANGE_SEQUENCE_STEP_DSS = 0x28
RANGE_SEQUENCE_STEP_PRE_RANGE = 0x40
RANGE_SEQUENCE_STEP_FINAL_RANGE = 0x80
VL53L0X_DEFAULT_MAX_LOOP = 100
VL53L0X_REG_SYSRANGE_MODE_MASK = 0x0F
VL53L0X_REG_SYSRANGE_MODE_START_STOP = 0x01
VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT = 0x00
VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK = 0x02
VL53L0X_REG_SYSRANGE_MODE_TIMED = 0x04
VL53L0X_REG_SYSRANGE_MODE_HISTOGRAM = 0x08

SPAD_TYPE_APERTURE = 0x01
SPAD_START_SELECT = 0xB4
SPAD_MAX_COUNT = 44
SPAD_MAP_ROW_COUNT = 6
SPAD_ROW_SIZE = 8

SPAD_APERTURE_START_INDEX = 12

CALIBRATION_TYPE_VHV = 0
CALIBRATION_TYPE_PHASE = 1

VL53L0X_SINGLE_RANGING = 0
VL53L0X_CONTINUOUS_RANGING = 1
VL53L0X_SINGLE_RANGING_NO_POLLING = 2


class VL53L0XError(Exception):
    pass


class VL53L0X:
    def __init__(self, addr: int = 0x29):
        self._dev = request_interface("i2c", "VL53L0X", addr)
        self._stop_variable = 0
        self._gpio_func = VL53L0X_GPIO_FUNC_NEW_MEASURE_READY
        self._soft_reset()
        self._check_i2c_com()
        self._data_init()
        self._static_init()
        self._perform_ref_calibration()

    def _read_byte(self, reg: int):
        return self._dev.read_reg_byte(reg)

    def _read_word(self, reg: int):
        temp = self._dev.read_reg_data(reg, 2)
        return unpack(">H", temp)[0]

    def _read_dword(self, reg: int):
        temp = self._dev.read_reg_data(reg, 4)
        return unpack(">L", temp)[0]

    def _write_byte(self, reg: int, val: int):
        self._dev.write_reg_byte(reg, val)

    def _write_word(self, reg: int, val: int):
        self._dev.write_reg_data(reg, pack(">H", val))

    def _write_dword(self, reg: int, val: int):
        self._dev.write_reg_data(reg, pack(">L", val))

    def _read_multi(self, reg: int, count: int):
        return self._dev.read_reg_data(reg, count)

    def _write_multi(self, reg: int, data: list, count: int):
        self._dev.write_reg_data(reg, bytes(data[:count]))

    def _read_strobe(self):
        strobe = 0
        timeout_cycles = 0
        self._write_byte(0x83, 0x00)
        while strobe == 0:
            strobe = self._read_byte(0x83)
            if strobe != 0:
                break
            time.sleep(0.02)
            if timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP:
                raise VL53L0XError("read_strobe timeout")
            timeout_cycles += 1
        self._write_byte(0x83, 0x01)
        return True

    def _get_spad_info_from_nvm(self):
        self._write_byte(0x80, 0x01)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._write_byte(0xFF, 0x06)
        self._write_byte(0x83, self._read_byte(0x83) | 0x04)
        self._write_byte(0xFF, 0x07)
        self._write_byte(0x81, 0x01)
        self._write_byte(0x80, 0x01)

        self._write_byte(0x94, 0x6B)

        self._read_strobe()

        tmp_data32 = self._read_dword(0x83)
        spad_count = (tmp_data32 >> 8) & 0x7F
        spad_type = (tmp_data32 >> 15) & 0x01

        self._write_byte(0x81, 0x00)
        self._write_byte(0xFF, 0x06)
        self._write_byte(0x83, self._read_byte(0x83) & 0xFB)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x00)

        good_spad_map = self._read_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6)

        return spad_count, spad_type, list(good_spad_map)

    def _set_spads_from_nvm(self):
        spad_count, spad_type, good_spad_map = self._get_spad_info_from_nvm()
        # total_val = sum(good_spad_map)

        self._write_byte(0xFF, 0x01)
        self._write_byte(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self._write_byte(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self._write_byte(0xFF, 0x00)
        self._write_byte(GLOBAL_CONFIG_REF_EN_START_SELECT, SPAD_START_SELECT)

        offset = SPAD_APERTURE_START_INDEX if spad_type == SPAD_TYPE_APERTURE else 0
        spads_enabled_count = 0
        spads_to_enable_count = 0
        spad_map = [0] * SPAD_MAP_ROW_COUNT
        for row in range(SPAD_MAP_ROW_COUNT):
            for col in range(SPAD_ROW_SIZE):
                index = row * SPAD_ROW_SIZE + col
                assert index < SPAD_MAX_COUNT, "SPAD index out of range"
                if spads_enabled_count == spads_to_enable_count:
                    break
                if index < offset:
                    continue
                if (good_spad_map[row] >> col) & 0x01:
                    spad_map[row] |= 1 << col
                    spads_enabled_count += 1
                if spads_enabled_count == spads_to_enable_count:
                    break
        assert spads_enabled_count == spads_to_enable_count, "Not all SPADs enabled"
        self._write_multi(
            GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, SPAD_MAP_ROW_COUNT
        )

    def _check_i2c_com(self):
        assert self._read_byte(REF_REG_0) == REF_REG_0_VAL
        assert self._read_byte(REF_REG_1) == REF_REG_1_VAL
        assert self._read_byte(REF_REG_2) == REF_REG_2_VAL
        assert self._read_word(REF_REG_3) == REF_REG_3_VAL
        assert self._read_word(REF_REG_4) == REF_REG_4_VAL

    def _data_init(self):
        self._write_byte(
            VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
            (self._read_byte(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) & 0xFE) | 0x01,
        )

        self._write_byte(0x88, 0x00)

        self._write_byte(0x80, 0x01)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._stop_variable = self._read_byte(0x91)
        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x00)

    def _set_sequence_steps_enabled(self, sequence_step):
        self._write_byte(SYSTEM_SEQUENCE_CONFIG, sequence_step)

    def _perform_single_ref_calibration(self, calib_type):
        interrupt_status = 0
        timeout_cycles = 0
        if calib_type == CALIBRATION_TYPE_VHV:
            sequence_config = 0x01
            sysrange_start = 0x41
        elif calib_type == CALIBRATION_TYPE_PHASE:
            sequence_config = 0x02
            sysrange_start = 0x01
        else:
            raise VL53L0XError("Invalid calibration type")
        self._write_byte(SYSTEM_SEQUENCE_CONFIG, sequence_config)
        self._write_byte(SYSRANGE_START, sysrange_start)
        while True:
            if timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP:
                raise VL53L0XError("Timeout waiting for calibration")
            timeout_cycles += 1
            interrupt_status = self._read_byte(RESULT_INTERRUPT_STATUS)
            if interrupt_status & 0x07 != 0:
                break
        self._write_byte(SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write_byte(SYSRANGE_START, 0x00)

    def _perform_ref_calibration(self):
        self._perform_single_ref_calibration(CALIBRATION_TYPE_VHV)
        self._perform_single_ref_calibration(CALIBRATION_TYPE_PHASE)
        self._set_sequence_steps_enabled(
            RANGE_SEQUENCE_STEP_DSS
            + RANGE_SEQUENCE_STEP_PRE_RANGE
            + RANGE_SEQUENCE_STEP_FINAL_RANGE
        )

    def _static_init(self):
        self._set_spads_from_nvm()
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x09, 0x00)
        self._write_byte(0x10, 0x00)
        self._write_byte(0x11, 0x00)
        self._write_byte(0x24, 0x01)
        self._write_byte(0x25, 0xFF)
        self._write_byte(0x75, 0x00)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x4E, 0x2C)
        self._write_byte(0x48, 0x00)
        self._write_byte(0x30, 0x20)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x30, 0x09)
        self._write_byte(0x54, 0x00)
        self._write_byte(0x31, 0x04)
        self._write_byte(0x32, 0x03)
        self._write_byte(0x40, 0x83)
        self._write_byte(0x46, 0x25)
        self._write_byte(0x60, 0x00)
        self._write_byte(0x27, 0x00)
        self._write_byte(0x50, 0x06)
        self._write_byte(0x51, 0x00)
        self._write_byte(0x52, 0x96)
        self._write_byte(0x56, 0x08)
        self._write_byte(0x57, 0x30)
        self._write_byte(0x61, 0x00)
        self._write_byte(0x62, 0x00)
        self._write_byte(0x64, 0x00)
        self._write_byte(0x65, 0x00)
        self._write_byte(0x66, 0xA0)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x22, 0x32)
        self._write_byte(0x47, 0x14)
        self._write_byte(0x49, 0xFF)
        self._write_byte(0x4A, 0x00)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x7A, 0x0A)
        self._write_byte(0x7B, 0x00)
        self._write_byte(0x78, 0x21)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x23, 0x34)
        self._write_byte(0x42, 0x00)
        self._write_byte(0x44, 0xFF)
        self._write_byte(0x45, 0x26)
        self._write_byte(0x46, 0x05)
        self._write_byte(0x40, 0x40)
        self._write_byte(0x0E, 0x06)
        self._write_byte(0x20, 0x1A)
        self._write_byte(0x43, 0x40)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x34, 0x03)
        self._write_byte(0x35, 0x44)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x31, 0x04)
        self._write_byte(0x4B, 0x09)
        self._write_byte(0x4C, 0x05)
        self._write_byte(0x4D, 0x04)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x44, 0x00)
        self._write_byte(0x45, 0x20)
        self._write_byte(0x47, 0x08)
        self._write_byte(0x48, 0x28)
        self._write_byte(0x67, 0x00)
        self._write_byte(0x70, 0x04)
        self._write_byte(0x71, 0x01)
        self._write_byte(0x72, 0xFE)
        self._write_byte(0x76, 0x00)
        self._write_byte(0x77, 0x00)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x0D, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x01)
        self._write_byte(0x01, 0xF8)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x8E, 0x01)
        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x00)

        self._write_byte(
            SYSTEM_INTERRUPT_CONFIG_GPIO, VL53L0X_GPIO_FUNC_NEW_MEASURE_READY
        )
        self._write_byte(
            GPIO_HV_MUX_ACTIVE_HIGH, self._read_byte(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10
        )
        self._write_byte(SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._gpio_func = VL53L0X_GPIO_FUNC_NEW_MEASURE_READY

        self._set_sequence_steps_enabled(
            RANGE_SEQUENCE_STEP_DSS
            + RANGE_SEQUENCE_STEP_PRE_RANGE
            + RANGE_SEQUENCE_STEP_FINAL_RANGE
        )

    def _soft_reset(self):
        cnt = 0
        self._write_byte(SOFT_RESET_GO2_SOFT_RESET_N, 0x00)
        while self._read_byte(IDENTIFICATION_MODEL_ID) != 0 and cnt < 100:
            time.sleep(0.01)
            cnt += 1
        time.sleep(0.03)
        self._write_byte(SOFT_RESET_GO2_SOFT_RESET_N, 0x01)
        cnt = 0
        while self._read_byte(IDENTIFICATION_MODEL_ID) == 0 and cnt < 100:
            time.sleep(0.01)
            cnt += 1
        time.sleep(0.03)

    def start_measurement(self, mode):
        sysrange_start = 0
        interrupt_status = 0
        timeout_cycles = 0
        self._write_byte(0x80, 0x01)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._write_byte(0x91, self._stop_variable)
        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x00)
        if mode == VL53L0X_SINGLE_RANGING:
            self._write_byte(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP)
            while True:
                if timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP:
                    raise VL53L0XError("Timeout waiting for measurement")
                timeout_cycles += 1
                sysrange_start = self._read_byte(SYSRANGE_START)
                if (sysrange_start & 0x01) == 0:
                    break
            timeout_cycles = 0
            if self._gpio_func == VL53L0X_GPIO_FUNC_OFF:
                while True:
                    if timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP:
                        raise VL53L0XError("Timeout waiting for measurement")
                    timeout_cycles += 1
                    interrupt_status = self._read_byte(RESULT_RANGE_STATUS)
                    if interrupt_status & 0x01:
                        break
            timeout_cycles = 0
            if self._gpio_func == VL53L0X_GPIO_FUNC_NEW_MEASURE_READY:
                while True:
                    if timeout_cycles >= VL53L0X_DEFAULT_MAX_LOOP:
                        raise VL53L0XError("Timeout waiting for measurement")
                    timeout_cycles += 1
                    interrupt_status = self._read_byte(RESULT_INTERRUPT_STATUS)
                    if interrupt_status & 0x07:
                        break
        elif mode == VL53L0X_CONTINUOUS_RANGING:
            self._write_byte(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK)
        elif mode == VL53L0X_SINGLE_RANGING_NO_POLLING:
            self._write_byte(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP)
        else:
            raise VL53L0XError("Invalid mode")

    def stop_measurement(self):
        self._write_byte(SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._write_byte(0x91, 0x00)
        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)

    def activate_gpio_interrupt(self, int_type):
        assert int_type in (
            VL53L0X_GPIO_FUNC_OFF,
            VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_LOW,
            VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_HIGH,
            VL53L0X_GPIO_FUNC_THRESHOLD_CROSSED_OUT,
            VL53L0X_GPIO_FUNC_NEW_MEASURE_READY,
        ), "Invalid interrupt type"
        self._write_byte(SYSTEM_INTERRUPT_CONFIG_GPIO, int_type)
        self._write_byte(
            GPIO_HV_MUX_ACTIVE_HIGH, self._read_byte(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10
        )
        self._write_byte(SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._gpio_func = int_type

    def deactivate_gpio_interrupt(self):
        self._write_byte(SYSTEM_INTERRUPT_CONFIG_GPIO, VL53L0X_GPIO_FUNC_OFF)
        self._gpio_func = VL53L0X_GPIO_FUNC_OFF

    def clear_gpio_interrupt_flag(self):
        byte = 0xFF
        cycles = 0
        while (byte & 0x07) != 0x00:
            self._write_byte(SYSTEM_INTERRUPT_CLEAR, 0x01)
            self._write_byte(SYSTEM_INTERRUPT_CLEAR, 0x00)
            byte = self._read_byte(RESULT_INTERRUPT_STATUS)
            if cycles >= VL53L0X_DEFAULT_MAX_LOOP:
                raise VL53L0XError("Timeout waiting for interrupt clear")
            cycles += 1

    def read_in_oneshot_mode(self) -> int:
        self.start_measurement(VL53L0X_SINGLE_RANGING)
        range = self._read_word(RESULT_RANGE_STATUS + 10)
        self.clear_gpio_interrupt_flag()
        return range

    def start_continuous_mode(self):
        self.start_measurement(VL53L0X_CONTINUOUS_RANGING)

    def start_single_mode(self):
        self.start_measurement(VL53L0X_SINGLE_RANGING_NO_POLLING)

    def read_range(self) -> int:
        return self._read_word(RESULT_RANGE_STATUS + 10)

    def check_ranging_finished(self) -> bool:
        ret = True
        if self._gpio_func == VL53L0X_GPIO_FUNC_NEW_MEASURE_READY:
            ret = self._read_byte(RESULT_INTERRUPT_STATUS) & 0x07 != 0
        return ret and (self._read_byte(RESULT_RANGE_STATUS) & 0x01 != 0)
