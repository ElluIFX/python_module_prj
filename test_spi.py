import random
import time

from drivers.interface import register_interface, request_interface

register_interface("ch347", "spi", True, module_name="test1")

spi1 = request_interface("spi", "test1", 3, 1_875_000)


def generate_data(size=100):
    return bytes(random.randint(0, 255) for _ in range(size))


data = generate_data()
for i in range(10):
    rd = spi1.transfer(data)
    time.sleep(0.01)
else:
    print("Test passed")