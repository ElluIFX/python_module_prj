import time
from pprint import pprint

from loguru import logger

from drivers.interface import register_interface, request_interface

register_interface("ch347", "i2c")
register_interface("pca6416a", "gpio")
gpio = request_interface("gpio", "main")
io0 = gpio.get_pin("P06")
io1 = gpio.get_pin("P07")
io0.set_mode("output_push_pull")
io1.set_mode("output_push_pull")
while True:
    io0.write(True)
    io1.write(False)
    io0.write(False)
    io1.write(True)
