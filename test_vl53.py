import time

from loguru import logger

from drivers.interface import register_interface
from drivers.vl53l0x_st import VL53L0X

register_interface("cp2112", "i2c", clock=100000)

tof = VL53L0X()
tof.deactivate_gpio_interrupt()
while True:
    mm = tof.read_in_oneshot_mode()
    logger.info(f"Distance: {mm}mm")
    time.sleep(0.1)
