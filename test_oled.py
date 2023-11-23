import time

from luma.core.render import canvas
from luma.oled.device import ssd1306

from drivers.interface.if_cp2112 import CP2112_I2CInterfaceBuilder
from drivers.luma_if import i2c

CP2112_I2CInterfaceBuilder(clock=400000).register()
device = ssd1306(i2c(0x3C), 128, 32)
with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white", fill="black")
    draw.text((10, 10), "Hello World", fill="white")
print("Done")
while True:
    time.sleep(1)
