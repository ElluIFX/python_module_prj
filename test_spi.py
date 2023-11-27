import random
import time

from drivers.interface import register_interface, request_interface

register_interface("ch347", "spi", True)
register_interface("ch347", "gpio")

spi1 = request_interface("spi", "main", 3, 60_000_000)
gpio = request_interface("gpio", "main")


def generate_data(size=4000):
    return bytes(random.randint(0, 255) for _ in range(size))


for i in range(100):
    rd = spi1.transfer(data := generate_data())
    time.sleep(0.01)
    if rd != data:
        print("Test failed")
        break
else:
    print("Test passed")
