import math
import time

import ADD_PATH  # noqa: F401

from drivers.bmi160 import BMI160
from drivers.ina226 import INA226, INA226Monitor
from drivers.interface import register_interface
from drivers.interface.utils import i2c_bus_scan

if __name__ == "__main__":
    register_interface("ch347", "i2c", clock=400000)
    m = INA226Monitor().start()
    time.sleep(2)
    m.add_marker("init")
    imu = BMI160()
    imu.kalman_init()
    # imu.kalman_calibrate()
    start_time = time.perf_counter()
    sample_count = 0
    try:
        while True:
            p, r, y = imu.kalman_estimate()
            sample_count += 1
            sps = sample_count / (time.perf_counter() - start_time)
            print(
                f"roll: {p/math.pi*180:7.2f}, pitch: {r/math.pi*180:7.2f}, yaw: {y/math.pi*180:7.2f}, sps: {sps:.2f}             ",
                end="\r",
            )
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
