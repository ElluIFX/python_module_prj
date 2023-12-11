import time

from loguru import logger

from drivers.filters import kalman
from drivers.interface import register_interface, request_interface
from drivers.interface.utils import i2c_bus_scan
from drivers.mpu9250 import MPU9250

register_interface("ch347", "i2c")
i2c_bus_scan()

imu = MPU9250()
if 0:
    logger.info("Calibrating accelerometer, follow the instructions")
    imu.caliberateAccelerometer()
    logger.info("Accelerometer calibrated")
    logger.info("Calibrating magnetometer, rotate the sensor in all directions")
    imu.caliberateMagPrecise()
    logger.info("Magnetometer calibrated")
    time.sleep(1)
    imu.saveCalibDataToFile("./mpu9250_cali.json")
    logger.info("Calibration data saved to mpu9250_cali.json")
else:
    imu.loadCalibDataFromFile("./mpu9250_cali.json")
    logger.info("Calibration data loaded from mpu9250_cali.json")
est = kalman.Kalman()
last_time = time.perf_counter()
while True:
    imu.readSensor()
    time_now = time.perf_counter()
    dt = time_now - last_time
    last_time = time_now
    est.computeAndUpdateRollPitchYaw(
        imu.AccelVals[0],
        imu.AccelVals[1],
        imu.AccelVals[2],
        imu.GyroVals[0],
        imu.GyroVals[1],
        imu.GyroVals[2],
        imu.MagVals[0],
        imu.MagVals[1],
        imu.MagVals[2],
        dt,
    )
    print(
        f"roll: {est.roll:>8.3f} pitch: {est.pitch:>8.3f} yaw: {est.yaw:>8.3f}"
        + " " * 20,
        end="\r",
    )
    time.sleep(0.1)
