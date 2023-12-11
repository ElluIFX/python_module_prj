import time

from drivers import register_interface
from drivers.ft6236 import FT6236
from drivers.interface.utils import i2c_bus_scan

register_interface("ch347", "i2c")

i2c_bus_scan()

touch = FT6236(threshold=15)

while True:
    print(f"touch: {touch.read_all()}")
    time.sleep(0.1)
