from typing import Callable, List, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from .utils import BaseStruct, ByteVar


class IMUDataStruct(BaseStruct):  # IMU传感器数据包
    report_id = ByteVar("u16", int)  # 数据包ID
    acc_x = ByteVar("float", float)  # X轴加速度 / m/s^2
    acc_y = ByteVar("float", float)  # Y轴加速度 / m/s^2
    acc_z = ByteVar("float", float)  # Z轴加速度 / m/s^2
    rot_r = ByteVar("float", float)  # 四元数R
    rot_i = ByteVar("float", float)  # 四元数I
    rot_j = ByteVar("float", float)  # 四元数J
    rot_k = ByteVar("float", float)  # 四元数K

    roll = ByteVar("float", float)  # 横滚角 / deg
    pitch = ByteVar("float", float)  # 俯仰角 / deg
    yaw = ByteVar("float", float)  # 偏航角 / deg

    pressure = ByteVar("float", float)  # 气压 / Pa
    temperature = ByteVar("float", float)  # 温度 / °C
    humidity = ByteVar("float", float)  # 湿度 / %
    shake_stable = ByteVar("u16", int)  # 震动检测 8b | 稳定检测 8b
    tap_detect = ByteVar("u8", int)  # 点击检测 8b

    # 0x01: acc, 0x02: rot, 0x04: pressure, 0x08: temp,
    # 0x10: humidity, 0x20: shake, 0x40: stable, 0x80: tap
    update_flag = ByteVar("u8", int)  # 更新标志位
    PACK = [
        report_id,
        acc_x,
        acc_y,
        acc_z,
        rot_r,
        rot_i,
        rot_j,
        rot_k,
        pressure,
        temperature,
        humidity,
        shake_stable,
        tap_detect,
        update_flag,
    ]


class BNOIMU(object):
    class UpdateFlag:
        ACCEL = 0x01
        ROTATION = 0x02
        PRESSURE = 0x04
        TEMPERATURE = 0x08
        HUMIDITY = 0x10
        SHAKE = 0x20
        STABLE = 0x40
        TAP = 0x80

    def __init__(self) -> None:
        self.raw_data = IMUDataStruct()
        self._callbacks: List[Tuple[Callable[["BNOIMU"], None], int]] = []

    def update(self, data: bytes) -> None:
        """
        更新IMU数据
        """
        try:
            self.raw_data.update_from_bytes(data)
        except ValueError:
            return
        r, p, y = self.eular_rotation
        self.raw_data.roll.value = r
        self.raw_data.pitch.value = p
        self.raw_data.yaw.value = y
        for callback, flag in self._callbacks:
            if self.raw_data.update_flag.value & flag:
                callback(self)

    def _get_readble_vars(self):
        return [
            self.raw_data.acc_x,
            self.raw_data.acc_y,
            self.raw_data.acc_z,
            self.raw_data.roll,
            self.raw_data.pitch,
            self.raw_data.yaw,
            self.raw_data.pressure,
            self.raw_data.temperature,
            self.raw_data.humidity,
            self.raw_data.report_id,
        ]

    def register_callback(
        self, callback: Callable[["BNOIMU"], None], flag: int
    ) -> None:
        """
        注册回调函数
        """
        self._callbacks.append((callback, flag))

    @property
    def rotation(self) -> np.ndarray:
        """
        旋转四元数(x, y, z, w)
        """
        return np.array(
            [
                self.raw_data.rot_i.value,
                self.raw_data.rot_j.value,
                self.raw_data.rot_k.value,
                self.raw_data.rot_r.value,
            ]
        )

    @property
    def eular_rotation(self) -> np.ndarray:
        """
        获取欧拉角姿态
        返回值: roll, pitch, yaw / deg
        """
        try:
            return Rotation.from_quat(self.rotation).as_euler("yxz", degrees=True)
        except ValueError:
            return np.array([0.0, 0.0, 0.0])

    @property
    def accel(self) -> np.ndarray:
        """
        获取加速度
        返回值: x, y, z / m/s^2
        """
        return np.array(
            [
                self.raw_data.acc_x.value,
                self.raw_data.acc_y.value,
                self.raw_data.acc_z.value,
            ]
        )

    @property
    def pressure(self) -> float:
        """
        获取气压
        返回值: pa
        """
        return self.raw_data.pressure.value

    @property
    def temperature(self) -> float:
        """
        获取温度
        返回值: °C
        """
        return self.raw_data.temperature.value

    @property
    def humidity(self) -> float:
        """
        获取湿度
        返回值: %
        """
        return self.raw_data.humidity.value

    @property
    def stability_classification(self) -> int:
        """
        获取稳定分类
        """
        return self.raw_data.shake_stable.value & 0xFF

    @property
    def stability_detection(self) -> int:
        """
        获取稳定检测
        """
        return self.raw_data.shake_stable.value >> 8

    @property
    def tap_detection(self) -> dict[str, bool]:
        """
        获取点击检测
        """
        TAPDET_X = 1
        TAPDET_X_POS = 2
        TAPDET_Y = 4
        TAPDET_Y_POS = 8
        TAPDET_Z = 16
        TAPDET_Z_POS = 32
        TAPDET_DOUBLE = 64
        data = self.raw_data.tap_detect.value
        return {
            "x": bool(data & TAPDET_X),
            "x_pos": bool(data & TAPDET_X_POS),
            "y": bool(data & TAPDET_Y),
            "y_pos": bool(data & TAPDET_Y_POS),
            "z": bool(data & TAPDET_Z),
            "z_pos": bool(data & TAPDET_Z_POS),
            "double": bool(data & TAPDET_DOUBLE),
        }
