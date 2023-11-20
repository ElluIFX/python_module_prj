import math
import time

from loguru import logger

from drivers.imu import IMU
from drivers.interface.if_pyserial import Pyserial_UartInterfaceBuilder

Pyserial_UartInterfaceBuilder("COM29").register()

imu = IMU()
imu.kalman_init()
imu.kalman_calibrate()
while True:
    p, r, y = imu.kalman_estimate()
    logger.info(
        f"roll: {p/math.pi*180:.2f}, pitch: {r/math.pi*180:.2f}, yaw: {y/math.pi*180:.2f}"
    )
