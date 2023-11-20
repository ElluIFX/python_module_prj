import logging
import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .BMI160 import Driver

_acc_scale_divs = {
    3: 8192.0 * 2,  # 2G max
    5: 4096.0 * 2,  # 4G max
    8: 2048.0 * 2,  # 8G max
    12: 1024.0 * 2,  # 16G max
}
_gyr_scale_divs = {
    0: 16.4,  # 2000dps
    1: 32.8,  # 1000dps
    2: 65.6,  # 500dps
    3: 131.2,  # 250dps
    4: 262.4,  # 125dps
}
_tap_thresholds = {
    3: lambda x: max(0, round((x - 0.03125) / 0.0625)),  # 2G max
    5: lambda x: max(0, round((x - 0.0625) / 0.125)),  # 4G max
    8: lambda x: max(0, round((x - 0.125) / 0.25)),  # 8G max
    12: lambda x: max(0, round((x - 0.25) / 0.5)),  # 16G max
}
_shock_thresholds = {
    3: lambda x: max(0, round((x - 0.00391) / 0.00781)),  # 2G max
    5: lambda x: max(0, round((x - 0.00781) / 0.01563)),  # 4G max
    8: lambda x: max(0, round((x - 0.01563) / 0.03125)),  # 8G max
    12: lambda x: max(0, round((x - 0.03125) / 0.0625)),  # 16G max
}
_motion_thresholds = {
    3: lambda x: max(0, round((x / 0.00391))),  # 2G max
    5: lambda x: max(0, round((x / 0.00781))),  # 4G max
    8: lambda x: max(0, round((x / 0.01563))),  # 8G max
    12: lambda x: max(0, round((x / 0.03125))),  # 16G max
}


@dataclass
class KalmanFilter(object):
    C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]], dtype=np.float64)
    P = np.eye(4)
    Q = np.eye(4)
    R = np.eye(2)

    state_estimate = np.array([[0], [0], [0], [0]], dtype=np.float64)
    theta_hat = 0.0
    phi_hat = 0.0
    time: Optional[float] = None
    r_estimated = 0.0


@dataclass()
class Accel(object):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass()
class Gyro(object):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass()
class AccelGyro(object):
    accel: Accel = Accel()
    gyro: Gyro = Gyro()


@dataclass
class DectectedResult(object):
    XPos: int = 0
    XNeg: int = 0
    YPos: int = 0
    YNeg: int = 0
    ZPos: int = 0
    ZNeg: int = 0
    X: int = 0
    Y: int = 0
    Z: int = 0
    _XPos_updating: bool = False
    _XNeg_updating: bool = False
    _YPos_updating: bool = False
    _YNeg_updating: bool = False
    _ZPos_updating: bool = False
    _ZNeg_updating: bool = False

    def update(self, dir, state):
        if state:
            if not getattr(self, f"_{dir}_updating"):
                setattr(self, f"_{dir}_updating", True)
        else:
            if getattr(self, f"_{dir}_updating"):
                setattr(self, dir, getattr(self, dir) + 1)
                setattr(self, dir[0], getattr(self, dir[0]) + 1)
                setattr(self, f"_{dir}_updating", False)

    def __repr__(self) -> str:
        return f"(X:{self.X}/{self.XPos}+/{self.XNeg}-, Y:{self.Y}/{self.YPos}+/{self.YNeg}-, Z:{self.Z}/{self.ZPos}+/{self.ZNeg}-)"


class IMU(object):
    def __init__(self, addr=0x69):
        self.sensor = Driver(addr)
        self.set_acc_range(5)
        self.set_gyr_range(3)
        self.motion6 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # gx, gy, gz, ax, ay, az
        logging.info("IMU Initialized")

    def set_acc_range(self, range):
        """Set the accelerometer range.
        Available ranges:
        3: 2G max
        5: 4G max
        8: 8G max
        12: 16G max
        """
        assert range in [3, 5, 8, 12]
        self.sensor.setFullScaleAccelRange(range, None)
        self._acc_scale_div = _acc_scale_divs[range]
        self._tap_threshold = _tap_thresholds[range]
        self._shock_threshold = _shock_thresholds[range]
        self._motion_threshold = _motion_thresholds[range]
        logging.debug(f"New IMU Accel Range: {range}, Div={self._acc_scale_div}")

    def set_gyr_range(self, range):
        """Set the gyroscope range.
        Available ranges:
        0: 2000dps
        1: 1000dps
        2: 500dps
        3: 250dps
        4: 125dps
        """
        assert range in [0, 1, 2, 3, 4]
        self.sensor.setFullScaleGyroRange(range, None)
        self._gyr_scale_div = _gyr_scale_divs[range]
        logging.debug(f"New IMU Gyro Range: {range}, Div={self._gyr_scale_div}")

    def set_dlpf_mode(self, acc=2, gyro=2):
        """
        Set the digital low pass filter mode.
        mode:
        2: Normal mode
        1: OSR2 -> bandwidths are approximately half of normal mode
        0: OSR4 -> bandwidths are approximately quarter of normal mode
        """
        assert acc in [0, 1, 2]
        assert gyro in [0, 1, 2]
        self.sensor.setAccelDLPFMode(acc)
        self.sensor.set_gyro_dlpf_mod(gyro)

    def get_gyro_bias(self, N=100):
        # rad/s
        bx = 0.0
        by = 0.0
        bz = 0.0

        for i in range(N):
            gx, gy, gz = self.get_gyro()
            bx += gx
            by += gy
            bz += gz
            time.sleep(0.01)
        bx /= float(N)
        by /= float(N)
        bz /= float(N)
        logging.debug(f"IMU Gyro Bias (rad/s): {bx:.3e} {by:.3e} {bz:.3e}")
        return (bx, by, bz)

    def get_acc_angle_bias(self, N=100):
        # ret: phi_offset, theta_offset
        phi_offset = 0.0
        theta_offset = 0.0

        for i in range(N):
            phi_acc, theta_acc = self.get_acc_angles()
            phi_offset += phi_acc
            theta_offset += theta_acc
            time.sleep(0.01)
        phi_offset /= float(N)
        theta_offset /= float(N)
        logging.debug(f"IMU Acc Angle Bias (rad): {phi_offset:.3e} {theta_offset:.3e}")
        return (phi_offset, theta_offset)

    def get_gyro(self):
        # rad/s
        gx, gy, gz = self.sensor.getRotation()  # deg/s
        self.motion6[0] = gx / self._gyr_scale_div * math.pi / 180.0
        self.motion6[1] = gy / self._gyr_scale_div * math.pi / 180.0
        self.motion6[2] = gz / self._gyr_scale_div * math.pi / 180.0
        return tuple(self.motion6[0:3])

    @property
    def Gyro(self):
        return Gyro(self.motion6[0], self.motion6[1], self.motion6[2])

    def get_acc(self):
        # m/s^2
        ax, ay, az = self.sensor.getAcceleration()
        self.motion6[3] = ax / self._acc_scale_div
        self.motion6[4] = ay / self._acc_scale_div
        self.motion6[5] = az / self._acc_scale_div
        return tuple(self.motion6[3:6])

    @property
    def Accel(self):
        return Accel(self.motion6[3], self.motion6[4], self.motion6[5])

    def get_all(self):
        # rad/s, m/s^2
        gx, gy, gz, ax, ay, az = self.sensor.getMotion6()
        self.motion6[0] = gx / self._gyr_scale_div * math.pi / 180.0
        self.motion6[1] = gy / self._gyr_scale_div * math.pi / 180.0
        self.motion6[2] = gz / self._gyr_scale_div * math.pi / 180.0
        self.motion6[3] = ax / self._acc_scale_div
        self.motion6[4] = ay / self._acc_scale_div
        self.motion6[5] = az / self._acc_scale_div
        return tuple(self.motion6)

    @property
    def AccelGyro(self):
        return AccelGyro(
            Accel(self.motion6[3], self.motion6[4], self.motion6[5]),
            Gyro(self.motion6[0], self.motion6[1], self.motion6[2]),
        )

    def get_temprature(self):
        return self.sensor.getTemperature()

    def get_acc_angles(self) -> tuple[float, float]:
        # rad
        ax, ay, az = self.get_acc()
        phi = math.atan2(ay, math.sqrt(ax**2.0 + az**2.0))
        theta = math.atan2(-ax, math.sqrt(ay**2.0 + az**2.0))
        return (phi, theta)

    def calibrate(self, acc=True, gyro=True, acc_direction=(0, 0, 1)):
        """Calibrate the accelerometer and gyroscope.
        acc_direction: (x, y, z) direction of gravity when calibrating accelerometer
        """
        logging.info("Calibrating IMU")
        self.sensor.setAccelOffsetEnabled(False)
        self.sensor.setGyroOffsetEnabled(False)
        if acc:
            self.sensor.autoCalibrateXAccelOffset(acc_direction[0])
            self.sensor.autoCalibrateYAccelOffset(acc_direction[1])
            self.sensor.autoCalibrateZAccelOffset(acc_direction[2])
            self.sensor.setAccelOffsetEnabled(True)
            logging.debug(
                f"Calibrated ACC offsets (g): {self.sensor.getXAccelOffset()*0.0039:.3e} {self.sensor.getYAccelOffset()*0.0039:.3e} {self.sensor.getZAccelOffset()*0.0039:.3e}"
            )
        if gyro:
            self.sensor.autoCalibrateGyroOffset()
            self.sensor.setGyroOffsetEnabled(True)
            logging.debug(
                f"Calibrated GYR offsets (dps): {self.sensor.getXGyroOffset()*0.061:.3e} {self.sensor.getYGyroOffset()*0.061:.3e} {self.sensor.getZGyroOffset()*0.061:.3e}"
            )
        logging.info("IMU Calibration Complete")

    def kalman_calibrate(self, N=100):
        """
        Calibrate the Kalman Filter
        N: Number of samples to use for calibration
        """
        logging.info(f"Calibrating Kalman Filter with {N} samples")
        self._phi_offset, self._theta_offset = self.get_acc_angle_bias(N)
        self._p_offset, self._q_offset, self._r_offset = self.get_gyro_bias(N)
        logging.info("Kalman Filter Calibration Complete")

    def kalman_init(self, filter_k=0.6, calibrate=False, calibrate_N=100):
        """
        Initialise the Kalman Filter
        filter_k: Result filter coefficient, 0.0-1.0, 1.0 = no filter
        calibrate: If True, calibrate the filter before initialising
        calibrate_N: Number of samples to use for calibration
        """
        if calibrate:
            self.kalman_calibrate(calibrate_N)
        else:
            self._phi_offset, self._theta_offset = 0, 0
            self._p_offset, self._q_offset, self._r_offset = 0, 0, 0

        self.karman = KalmanFilter()
        self.karman_roll = 0.0
        self.karman_pitch = 0.0
        self.karman_yaw = 0.0
        self.karman_filter_k = filter_k

    def kalman_estimate(
        self, data: Optional[tuple] = None
    ) -> tuple[float, float, float]:
        """
        Estimate the attitude using a Kalman filter.
        data: gx, gy, gz, ax, ay, az
        (get from imu.get_all() is recommended, or leave blank to auto-read)
        returns: (pitch, roll, yaw) in radians
        """
        # Get current time
        if data is None:
            data = self.get_all()
        time_now = time.perf_counter()
        if self.karman.time is None:
            self.karman.time = time_now
            return (0.0, 0.0, 0.0)
        dt = time_now - self.karman.time
        self.karman.time = time_now

        # Get measurements
        p, q, r, ax, ay, az = data
        p -= self._p_offset
        q -= self._q_offset
        r -= self._r_offset
        phi_acc = math.atan2(ay, math.sqrt(ax**2.0 + az**2.0))
        theta_acc = math.atan2(-ax, math.sqrt(ay**2.0 + az**2.0))
        phi_acc -= self._phi_offset
        theta_acc -= self._theta_offset

        self.karman.r_estimated += dt * r
        if self.karman.r_estimated > math.pi:
            self.karman.r_estimated -= 2 * math.pi
        elif self.karman.r_estimated < -math.pi:
            self.karman.r_estimated += 2 * math.pi
        # Predict
        phi_dot = (
            p
            + q * math.sin(self.karman.phi_hat) * math.tan(self.karman.theta_hat)
            + r * math.cos(self.karman.phi_hat) * math.tan(self.karman.theta_hat)
        )
        theta_dot = q * math.cos(self.karman.phi_hat) - r * math.sin(
            self.karman.phi_hat
        )
        A = np.array([[1, -dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -dt], [0, 0, 0, 1]])
        B = np.array([[dt, 0], [0, 0], [0, dt], [0, 0]])
        gyro_input = np.array([[phi_dot], [theta_dot]])
        self.karman.state_estimate = A.dot(self.karman.state_estimate) + B.dot(
            gyro_input
        )
        self.karman.P = A.dot(self.karman.P.dot(np.transpose(A))) + self.karman.Q
        measurement = np.array([[phi_acc], [theta_acc]])
        y_tilde = measurement - self.karman.C.dot(self.karman.state_estimate)
        S = self.karman.R + self.karman.C.dot(
            self.karman.P.dot(np.transpose(self.karman.C))
        )
        K = self.karman.P.dot(np.transpose(self.karman.C).dot(np.linalg.inv(S)))
        self.karman.state_estimate = self.karman.state_estimate + K.dot(y_tilde)
        self.karman.P = (np.eye(4) - K.dot(self.karman.C)).dot(self.karman.P)
        self.karman.phi_hat = self.karman.state_estimate[0]
        self.karman.theta_hat = self.karman.state_estimate[2]

        self.karman_roll += (
            float(self.karman.phi_hat) - self.karman_roll
        ) * self.karman_filter_k
        self.karman_pitch += (
            float(self.karman.theta_hat) - self.karman_pitch
        ) * self.karman_filter_k
        self.karman_yaw += (
            float(self.karman.r_estimated) - self.karman_yaw
        ) * self.karman_filter_k
        return (self.karman_pitch, self.karman_roll, self.karman_yaw)

    def set_tap_detection(self, enabled: bool, threshold: float = 1.0):
        """
        Enable/disable tap detection
        threshold: G-force
        """
        if enabled:
            self.sensor.setIntTapEnabled(False)
            self.sensor.setTapDetectionThreshold(self._tap_threshold(threshold))
            self.tap_detected = DectectedResult()
        self.sensor.setIntTapEnabled(enabled)

    def update_tap_detection(self) -> DectectedResult:
        """
        Update tap detection status
        """
        assert hasattr(self, "tap_detected"), "Tap detection not enabled"
        for dir in ("XPos", "XNeg", "YPos", "YNeg", "ZPos", "ZNeg"):
            state = getattr(self.sensor, f"get{dir}TapDetected")()
            self.tap_detected.update(dir, state)
        return self.tap_detected

    def set_shock_detection(
        self, enabled: bool, threshold: float = 3.0, duration: float = 10.0
    ):
        """
        Enable/disable shock detection
        threshold: G-force
        duration: milliseconds (resolution: 2.5ms)
        """
        if enabled:
            self.sensor.setIntShockEnabled(False)
            self.sensor.setShockDetectionThreshold(self._shock_threshold(threshold))
            dur_data = max(0, round((duration - 2.5) / 2.5))
            self.sensor.setShockDetectionDuration(dur_data)
            self.shock_detected = DectectedResult()
        self.sensor.setIntShockEnabled(enabled)

    def update_shock_detection(self) -> DectectedResult:
        """
        Update shock detection status
        """
        assert hasattr(self, "shock_detected"), "Shock detection not enabled"
        for dir in ("XPos", "XNeg", "YPos", "YNeg", "ZPos", "ZNeg"):
            state = getattr(self.sensor, f"get{dir}ShockDetected")()
            self.shock_detected.update(dir, state)
        return self.shock_detected

    def set_motion_detection(
        self, enabled: bool, threshold: float = 0.3, duration: int = 4
    ):
        """
        Enable/disable motion detection
        threshold: G-force
        duration: sapmles higher than threshold to trigger motion detection
        """
        if enabled:
            self.sensor.setIntMotionEnabled(False)
            self.sensor.setMotionDetectionThreshold(self._motion_threshold(threshold))
            self.sensor.setMotionDetectionDuration(duration)
            self.motion_detected = DectectedResult()
        self.sensor.setIntMotionEnabled(enabled)

    def update_motion_detection(self) -> DectectedResult:
        """
        Update motion detection status
        """
        assert hasattr(self, "motion_detected"), "Motion detection not enabled"
        for dir in ("XPos", "XNeg", "YPos", "YNeg", "ZPos", "ZNeg"):
            state = getattr(self.sensor, f"get{dir}MotionDetected")()
            self.motion_detected.update(dir, state)
        return self.motion_detected

    def set_idle_detection(
        self, enabled: bool, threshold: float = 0.1, duration: int = 1
    ):
        """
        Enable/disable idle detection
        threshold: G-force
        duration: (x+1) * 1.28s and x<15
        """
        if enabled:
            self.sensor.setIntZeroMotionEnabled(False)
            self.sensor.setZeroMotionDetectionThreshold(
                self._motion_threshold(threshold)
            )
            duration = min(15, max(0, duration - 1))
            self.sensor.setZeroMotionDetectionDuration(duration)
            self.idle_count = 0
            self._idle_detected = False
        self.sensor.setIntZeroMotionEnabled(enabled)

    def update_idle_detection(self) -> int:
        """
        Get idle detection count
        """
        state = self.sensor.getIntZeroMotionStatus()
        if state and not self._idle_detected:
            self.idle_count += 1
            self._idle_detected = True
        elif not state:
            self._idle_detected = False
        return self.idle_count

    def set_freefall_detection(
        self, enabled: bool, threshold: float = 0.1, duration: float = 2.5
    ):
        """
        Enable/disable freefall detection
        threshold: G-force
        duration: milliseconds (resolution: 2.5ms)
        """
        if enabled:
            self.sensor.setIntFreefallEnabled(False)
            thr = max(0, round((threshold - 0.00391) / 0.00781))
            self.sensor.setFreefallDetectionThreshold(thr)
            dur = max(0, round((duration - 2.5) / 2.5))
            self.sensor.setFreefallDetectionDuration(dur)
            self.freefall_count = 0
            self._freefall_detected = False
        self.sensor.setIntFreefallEnabled(enabled)

    def update_freefall_detection(self) -> int:
        """
        Get freefall detection count
        """
        state = self.sensor.getIntFreefallStatus()
        if state and not self._freefall_detected:
            self.freefall_count += 1
            self._freefall_detected = True
        elif not state:
            self._freefall_detected = False
        return self.freefall_count

    def set_step_counter(self, enabled: bool, mode=0):
        """
        Enable/disable step counter
        mode:
        0 = Normal Mode
        1 = Sensitive Mode
        2 = Robust Mode
        3 = Unkown Mode
        """
        if enabled:
            self.sensor.setStepCountEnabled(False)
            self.sensor.setStepDetectionMode(mode)
        self.sensor.setStepCountEnabled(enabled)

    def get_step_counter(self) -> int:
        """
        Get step counter
        """
        return self.sensor.getStepCount()

    def reset_step_counter(self):
        """
        Reset step counter
        """
        self.sensor.resetStepCount()
