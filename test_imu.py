import math
import time

from loguru import logger

from drivers.imu import IMU
from drivers.interface.if_cp2112 import CP2112_I2CInterfaceBuilder

CP2112_I2CInterfaceBuilder(clock=400000).register()

imu = IMU()
imu.kalman_init()
imu.kalman_calibrate()
while True:
    p, r, y = imu.kalman_estimate()
    logger.info(
        f"roll: {p/math.pi*180:7.2f}, pitch: {r/math.pi*180:7.2f}, yaw: {y/math.pi*180:7.2f}"
    )
