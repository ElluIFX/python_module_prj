import time

from loguru import logger

from drivers.interface import register_interface
from drivers.vl53l0x import VL53L0X

register_interface("cp2112", "i2c", clock=400000)
tof = VL53L0X()
tof.measurement_timing_budget = 20000
last_time = time.perf_counter()
tof.continuous_mode = True
while True:
    while not tof.data_ready:
        time.sleep(0.001)
    now = time.perf_counter()
    dist = tof.distance
    logger.info(f"Distance: {dist:7.3f}m Cost: {now - last_time:.3f}s")
    last_time = now
