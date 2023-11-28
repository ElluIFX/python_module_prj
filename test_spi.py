import random
import time

from drivers.interface import register_interface, request_interface

register_interface("ch347", "spi")
register_interface("ch347", "i2c")
register_interface(
    "pca6416a",
    "gpio",
)

spi1 = request_interface("spi", "main", 0, 1_000_000)
io = request_interface("gpio", "main")
cs = io.get_pin("P03")
cs.set_mode("output_push_pull")

def generate_data(size=10):
    return bytes(random.randint(0, 255) for _ in range(size))


for _ in range(100):
    cs.write(False)
    rd = spi1.transfer(data := generate_data())
    cs.write(True)
    time.sleep(0.01)
