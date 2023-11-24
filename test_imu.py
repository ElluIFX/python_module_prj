import math
import time

from loguru import logger

from drivers.imu import IMU
from drivers.interface import register_interface
from drivers.interface.utils import i2c_bus_scan

register_interface("ch347", "i2c", clock=400000)
i2c_bus_scan()
imu = IMU()
imu.kalman_init()
imu.kalman_calibrate()
while True:
    p, r, y = imu.kalman_estimate()
    logger.info(
        f"roll: {p/math.pi*180:7.2f}, pitch: {r/math.pi*180:7.2f}, yaw: {y/math.pi*180:7.2f}"
    )
