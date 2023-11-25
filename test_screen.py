import time

import cv2
import numpy as np

from drivers.interface import register_interface
from drivers.st7789 import ST7789

# register_interface("periphery", "spi", "/dev/spidev1.0")
register_interface("spidev", "spi", 1, 0)
register_interface(
    "periphery",
    "gpio",
    pinmap={"DC": "GPIOX_9", "RST": "GPIOC_7", "BL": "GPIOX_8"},
    modemap={"GPIOC_7": "output_open_drain", "GPIOH_8": "output_open_drain"},
)

screen = ST7789()
print("screen initialized")


def circle_test():
    width = screen._width
    height = screen._height
    test_img = np.zeros((height, width, 3), dtype=np.uint8)
    screen.display(test_img)
    circle_num = 5
    x = [width // 2 for _ in range(circle_num)]
    y = [height // 2 for _ in range(circle_num)]
    add_x = [1, 2, -2, -3, 2, -1]
    add_y = [1, -3, 1, -4, 3, -3]
    r = [60, 30, 20, 10, 5, 2]
    colors = [
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (255, 0, 255),
        (255, 255, 0),
        (0, 255, 255),
    ]
    bright = 1
    add = False
    while True:
        img = test_img.copy()
        for i in range(circle_num):
            x[i] += add_x[i]
            y[i] += add_y[i]
            if x[i] >= width - r[i] or x[i] <= r[i]:
                add_x[i] = -add_x[i]
            if y[i] >= height - r[i] or y[i] <= r[i]:
                add_y[i] = -add_y[i]
            cv2.circle(img, (int(x[i]), int(y[i])), r[i], colors[i], 2)
        t0 = time.perf_counter()
        screen.display_diff(img)
        dt = time.perf_counter() - t0
        print(f"FPS: {1/dt:.2f} dt: {dt:.6f}")
        bright *= 0.9 if not add else 1.1
        if bright < 0.1 or bright > 1:
            bright = max(0.1, min(1, bright))
            add = not add


try:
    circle_test()
except KeyboardInterrupt:
    screen.turn_off()
