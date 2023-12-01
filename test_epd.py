import os
import time
from math import floor

import cv2

from drivers import register_interface
from drivers.epd import EPD_BW_154

os.chdir(os.path.dirname(os.path.abspath(__file__)))
register_interface("ch347", "spi")
# register_interface("ch347", "i2c")
register_interface(
    "ch347",
    "gpio",
    pinmap={
        "DC": "GPIO6",
        "RST": "GPIO4",
        "BUSY": "GPIO7",
    },
)


def vertical_text(
    img, text, pos, font, scale, color, thinkness, x_offset=None, y_offset=3
):
    x, y = pos
    text_len = len(text)
    sizes = [
        cv2.getTextSize(text[i], font, scale, thinkness)[0] for i in range(text_len)
    ]
    avg_x = sum([sizes[i][0] for i in range(text_len)]) / text_len
    for i in range(text_len):
        if x_offset is None:
            x_offset_set = floor((avg_x - sizes[i][0]) / 2)
        else:
            x_offset_set = x_offset
        y += sizes[i][1] + y_offset
        cv2.putText(img, text[i], (x + x_offset_set, y), font, scale, color, thinkness)


def test_image():
    with EPD_BW_154() as epd:
        cvimg = cv2.imread("./res/girl.jpg")
        cvimg = cv2.threshold(cvimg, 185, 255, cv2.THRESH_BINARY)[1]
        cvimg = epd.fit_cv2(cvimg)
        t0 = time.perf_counter()
        # epd.display(cvimg)
        epd.clear()
        t1 = time.perf_counter()
        print(f"Display time: {t1 - t0:.3f}s")
        t0 = time.perf_counter()
        epd.display_partial(cvimg)
        t1 = time.perf_counter()
        print(f"Display partial time: {t1 - t0:.3f}s")


test_image()
