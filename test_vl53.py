import time

from loguru import logger

from drivers.interface.if_cp2112 import CP2112_I2CInterfaceBuilder
from drivers.vl53l0x import VL53L0X

CP2112_I2CInterfaceBuilder(clock=400000).register()

# tof = VL53L0X()
# # tof.deactivate_gpio_interrupt()
# while True:
#     # mm = tof.read_in_oneshot_mode()
#     mm = tof.measure()
#     logger.info(f"Distance: {mm}mm")
#     time.sleep(0.1)
