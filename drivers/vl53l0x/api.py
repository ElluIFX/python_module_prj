#
# api.py contains a set of core functions
#

import logging
import time

import smbus2

from .register import *


def make_uint16(lsb, msb):
    return (msb << 8) + lsb


class VL53L0X(object):
    def __init__(self, address=VL53L0X_DEFAULT_ADDRESS, bus=3, init=True):
        # i2c device address
        self.address = address

        # smbus object
        self.bus = smbus2.SMBus(bus)

        # static sequence config
        self.static_seq_config = 0

        # measurement data, in millimeter
        self.measurement = 0

        if init:
            self.setup()

    def setup(self):
        self._data_init()
        self._static_init()
        self._perform_ref_calibration()
        self._perform_ref_spad_management()

    def measure(self):
        self._perform_ref_signal_measurement()

        return self.measurement

    def _data_init(self):
        # set i2c standard mode
        self._write_byte(0x88, 0x00)

        # read whoami
        self._read_byte(0xC0)

        # use internal default settings
        self._write_byte(0x80, 0x01)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)

        self._read_byte(0x91)

        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x00)

        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)

    def _static_init(self):
        self._write_byte(0xFF, 0x01)
        self._read_byte(0x84)
        self._write_byte(0xFF, 0x00)

        # read the sequence config and save it
        self.static_seq_config = self._read_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG)

    def _perform_ref_calibration(self):
        self._perform_vhv_calibration()
        self._perform_phase_calibration()

        # restore static sequence config
        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def _perform_vhv_calibration(self):
        # run vhv
        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01)

        self._perform_single_ref_calibration(0x40)

        # read vhv from device
        self._ref_calibration_io(0xCB)

        # restore static sequence config
        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def _perform_phase_calibration(self):
        # run phase cal
        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02)

        self._perform_single_ref_calibration(0x0)

        # read phase cal from device
        self._ref_calibration_io(0xEE)

        # restore static sequence config
        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def _perform_single_ref_calibration(self, byte):
        self._write_byte(
            VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP | byte
        )
        self._write_byte(VL53L0X_REG_SYSRANGE_START, 0x00)

    def _ref_calibration_io(self, byte):
        # read vhv from device
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._write_byte(0xFF, 0x00)

        self._read_byte(byte)

        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._write_byte(0xFF, 0x00)

    def _perform_ref_spad_management(self):
        self._write_byte(0xFF, 0x01)
        self._write_byte(VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self._write_byte(VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self._write_byte(0xFF, 0x00)
        self._write_byte(VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)
        self._write_byte(VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0)

        self._perform_ref_calibration()
        self._perform_ref_signal_measurement()

    def _perform_ref_signal_measurement(self):
        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xC0)

        self._perform_single_ranging_measurement()

        self._write_byte(0xFF, 0x01)
        self._write_byte(0xFF, 0x00)

        # restore static sequence config
        self._write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def _perform_single_ranging_measurement(self):
        self._perform_single_measurement()
        self._get_ranging_measurement_data()

    def _perform_single_measurement(self):
        self._start_measurement()

    def _start_measurement(self):
        self._write_byte(0x80, 0x01)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)

        self._read_byte(0x91)

        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x00)

        # device mode single ranging
        self._write_byte(VL53L0X_REG_SYSRANGE_START, 0x01)

    def _get_ranging_measurement_data(self):
        raw_data = self._read_block(0x14)

        range_millimeter = make_uint16(raw_data[11], raw_data[10])
        if 0:  # for debug
            signal_rate = make_uint16(raw_data[7], raw_data[6])
            ambient_rate = make_uint16(raw_data[9], raw_data[8])
            effective_spad_rtn_count = make_uint16(raw_data[3], raw_data[2])
            device_range_status = raw_data[0]

            logging.debug(
                f"range: {range_millimeter}\tsignal rate: {signal_rate}\tambient rate: {ambient_rate}\tspad count: {effective_spad_rtn_count}\trange status: {device_range_status}"
            )

        # update measurement
        self.measurement = range_millimeter

    def _write_byte(self, reg, data):
        self.bus.write_byte_data(self.address, reg, data)

    def _read_byte(self, reg):
        read = self.bus.read_byte_data(self.address, reg)
        return read

    def _read_block(self, reg):
        read = self.bus.read_i2c_block_data(self.address, reg, 16)
        return read
