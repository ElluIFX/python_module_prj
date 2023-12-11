import datetime
import time

from luma.core.render import canvas
from luma.oled.device import ssd1306

from drivers.interface import register_interface
from drivers.luma_adapt import i2c

register_interface("cp2112", "i2c", clock=1000000)
device = ssd1306(i2c(0x3C), 128, 32)
width, height = device.width, device.height
while True:
    with canvas(device) as draw:
        # draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.polygon([(0, 0), (height, height), (0, height)], fill="white")
        draw.polygon(
            [(width - 1, 0), (width - height - 1, 0), (width - 1, height)],
            fill="white",
        )
        draw.text(
            (width // 2 - 40, 3),
            datetime.datetime.now().strftime("%Y-%m-%d"),
            fill="white",
        )
        draw.text(
            (width // 2 - 10, 18),
            datetime.datetime.now().strftime("%H:%M:%S"),
            fill="white",
        )
    time.sleep(1)
