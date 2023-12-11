import time

from loguru import logger

from drivers.interface import register_interface, request_interface

register_interface("cp2112", "gpio")
gpio = request_interface("gpio", "main")

gpio_dict = gpio.get_available_pinmodes()
gpio_list = list(gpio_dict.keys())
logger.info(f"GPIO list: {gpio_list}")
for n in gpio_list:
    gpio.set_mode(n, "output_push_pull")
    gpio.write(n, False)
logger.info("GPIO init done")
tp = gpio.get_pin("Pin_7")

while True:
    tp.value = True
    tp.value = False
