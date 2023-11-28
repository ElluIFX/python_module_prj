import time

from loguru import logger

from drivers.interface import register_interface, request_interface

register_interface("ch347", "gpio")
gpio = request_interface("gpio", "main")

gpio_dict = gpio.get_available_pins()
gpio_list = list(gpio_dict.keys())
logger.info(f"GPIO list: {gpio_list}")
for n in gpio_list:
    gpio.set_mode(n, "output_push_pull")
    gpio.write(n, False)
logger.info("GPIO init done")
time.sleep(2)
for n in gpio_list:
    logger.info(f"GPIO {n} toggle")
    gpio.write(n, True)
    time.sleep(0.8)
    gpio.write(n, False)
    time.sleep(0.2)
logger.info("GPIO test done")
for n in gpio_list:
    logger.info(f"GPIO {n} free")
    gpio.write(n, True)
    time.sleep(0.2)
    gpio.free(n)
    time.sleep(0.2)
