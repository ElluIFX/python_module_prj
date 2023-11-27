from typing import List, Literal, Optional, Tuple

from .interface import request_interface

FT6236_ADDR = 0x38  # I2C address
FT6236_G_FT5201ID = 0xA8  # FocalTech's panel ID
FT6236_REG_NUMTOUCHES = 0x02  # Number of touch points
FT6236_NUM_X = 0x33  # Touch X position
FT6236_NUM_Y = 0x34  # Touch Y position
FT6236_REG_MODE = 0x00  # Device mode, either WORKING or FACTORY
FT6236_REG_CALIBRATE = 0x02  # Calibrate mode
FT6236_REG_WORKMODE = 0x00  # Work mode
FT6236_REG_FACTORYMODE = 0x40  # Factory mode
FT6236_REG_THRESHHOLD = 0x80  # Threshold for touch detection
FT6236_REG_POINTRATE = 0x88  # Point rate
FT6236_REG_FIRMVERS = 0xA6  # Firmware version
FT6236_REG_CHIPID = 0xA3  # Chip selecting
FT6236_REG_VENDID = 0xA8  # FocalTech's panel ID
FT6236_VENDID = 0x11  # FocalTech's panel ID
FT6206_CHIPID = 0x06  # FT6206 ID
FT6236_CHIPID = 0x36  # FT6236 ID
FT6236U_CHIPID = 0x64  # FT6236U ID


class FT6236:
    def __init__(self, address: int = FT6236_ADDR, threshold: int = 20) -> None:
        self._bus = request_interface("i2c", "FT6236", address)
        assert self._bus.read_reg_byte(FT6236_REG_VENDID) == FT6236_VENDID
        assert self._bus.read_reg_byte(FT6236_REG_CHIPID) in (
            FT6206_CHIPID,
            FT6236_CHIPID,
            FT6236U_CHIPID,
        )
        self._bus.write_reg_byte(FT6236_REG_THRESHHOLD, threshold)

    @property
    def threshold(self) -> int:
        return self._bus.read_reg_byte(FT6236_REG_THRESHHOLD)

    @threshold.setter
    def threshold(self, value: int):
        self._bus.write_reg_byte(FT6236_REG_THRESHHOLD, value)

    @property
    def touched(self) -> int:
        ret = self._bus.read_reg_byte(FT6236_REG_NUMTOUCHES)
        return ret if ret <= 2 else 0

    def get_point(self, point: Literal[0, 1]) -> Tuple[int, int, int]:
        addr = 0x03 + point * 6
        data = self._bus.read_reg_data(addr, 6)
        x = (data[0] & 0x0F) << 8 | data[1]
        y = (data[2] & 0x0F) << 8 | data[3]
        tid = data[2] >> 4
        return x, y, tid

    @property
    def point1(self) -> Tuple[int, int, int]:
        return self.get_point(0)

    @property
    def point2(self) -> Tuple[int, int, int]:
        return self.get_point(1)

    def read_all(self) -> Optional[List[Tuple[int, int, int]]]:
        data = self._bus.read_reg_data(0x02, 13)
        num = data[0] if data[0] <= 2 else 0
        if num == 0:
            return None
        ret = []
        for i in range(num):
            x = (data[1 + i * 6] & 0x0F) << 8 | data[2 + i * 6]
            y = (data[3 + i * 6] & 0x0F) << 8 | data[4 + i * 6]
            tid = data[3 + i * 6] >> 4
            ret.append((x, y, tid))
        ret.sort(key=lambda x: x[2])
        return ret
