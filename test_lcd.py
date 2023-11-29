import time

import cv2
import numpy as np

from drivers.interface import register_interface
from drivers.st7789 import ST7789
from drivers.st7796 import ST7796

# register_interface("periphery", "spi", "/dev/spidev1.0")
# register_interface("spidev", "spi", 1, 0)
# register_interface(
#     "periphery",
#     "gpio",
#     pinmap={"DC": "GPIOX_9", "RST": "GPIOC_7", "BLK": "GPIOX_8"},
#     modemap={"GPIOC_7": "output_open_drain", "GPIOH_8": "output_open_drain"},
# )

register_interface("ch347", "spi")
register_interface(
    "ch347",
    "gpio",
    pinmap={"DC": "GPIO4", "RST": "GPIO7", "BLK": "GPIO6"},
)

# screen = ST7789()
# screen = ST7789(width=240, height=240, offset_left=0, offset_top=0, rotation=270)
screen = ST7796(width=240, height=240, offset_left=0, offset_top=0)
print(f"screen initialized with {screen.WIDTH}x{screen.HEIGHT}")


def circle_test():
    width = screen.WIDTH
    height = screen.HEIGHT
    test_img = np.zeros((height, width, 3), dtype=np.uint8)
    screen.display(test_img)
    circle_num = 4
    x = [width // 2 for _ in range(circle_num)]
    y = [height // 2 for _ in range(circle_num)]
    add_x = [1, 2, -2, -3, 2, -1]
    add_y = [2, -3, 1, -4, 3, -3]
    r = [25, 30, 20, 10, 5, 2]
    colors = [
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (255, 0, 255),
        (255, 255, 0),
        (0, 255, 255),
    ]
    while True:
        img = test_img.copy()
        for i in range(circle_num):
            x[i] += add_x[i]
            y[i] += add_y[i]
            if x[i] >= width - r[i] or x[i] <= r[i]:
                add_x[i] = -add_x[i]
            if y[i] >= height - r[i] or y[i] <= r[i]:
                add_y[i] = -add_y[i]
            cv2.circle(img, (int(x[i]), int(y[i])), r[i], colors[i], -1)
        t0 = time.perf_counter()
        screen.display(img)
        dt = time.perf_counter() - t0
        print(f"FPS: {1/dt:.2f} dt: {dt:.6f}")


def bad_apple():
    video = cv2.VideoCapture("badapple.mp4")
    frame_num = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_cnt = 0
    while True:
        # time.sleep(0.01)
        ret, frame = video.read()
        if not ret:
            break
        frame_cnt += 1
        frame = cv2.resize(frame, (screen.WIDTH, screen.HEIGHT))
        t0 = time.perf_counter()
        screen.display(frame)
        dt = time.perf_counter() - t0
        print(f"FPS: {1/dt:.2f} dt: {dt:.6f} frame: {frame_cnt}/{frame_num}")


try:
    # circle_test()
    bad_apple()
finally:
    screen.turn_off()
