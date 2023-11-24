import time

from luma.core.render import canvas
from luma.oled.device import ssd1306

from drivers.interface import register_interface
from drivers.luma_adapt import i2c

register_interface("cp2112", "i2c", clock=1000000)
device = ssd1306(i2c(0x3C), 128, 32)

with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white", fill="black")
    draw.text((10, 10), "Hello World", fill="white")
print("Done")
while True:
    time.sleep(1)
