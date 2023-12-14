import time

import ADD_PATH  # noqa: F401

from drivers.ina226 import INA226, INA226Monitor
from drivers.interface import register_interface

if __name__ == "__main__":
    register_interface("ch347", "i2c", clock=400000)
    # ina = INA226(software_filter_N=3)
    # ina.set_current_measurement_range(0.2)
    # print(f"Conv time: {ina.conversion_time*1000:.2f} ms")

    # while True:
    #     print(
    #         f"Voltage: {ina.voltage:.6f}V Current: {ina.current:.6f}A Power: {ina.power:.6f}W"
    #         + " " * 10,
    #         end="\r",
    #     )
    #     time.sleep(0.1)

    m = INA226Monitor()
    m.start()
    while m.is_alive():
        time.sleep(1)
