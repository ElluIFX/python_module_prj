import time

import cv2
import numpy as np

from .interface import request_interface


class _CMD:
    NOP = 0x00
    SWRST = 0x01
    RDDID = 0x04
    RDDST = 0x09

    SLPIN = 0x10
    SLPOUT = 0x11
    PTLON = 0x12
    NORON = 0x13

    RDMODE = 0x0A
    RDMADCTL = 0x0B
    RDPIXFMT = 0x0C
    RDIMGFMT = 0x0A
    RDSELFDIAG = 0x0F

    INVOFF = 0x20
    INVON = 0x21
    DISPOFF = 0x28
    DISPON = 0x29
    CASET = 0x2A
    PASET = 0x2B
    RASET = 0x2B
    RAMWR = 0x2C
    RAMRD = 0x2E

    PTLAR = 0x30
    VSCRDEF = 0x33
    MADCTL = 0x36
    VSCRSADD = 0x37
    PIXFMT = 0x3A
    MAD_MY = 0x80
    MAD_MX = 0x40
    MAD_MV = 0x20
    MAD_ML = 0x10
    MAD_BGR = 0x08
    MAD_MH = 0x04
    MAD_RGB = 0x00
    INVOFF = 0x20
    INVON = 0x21

    WRDISBV = 0x51
    RDDISBV = 0x52
    WRCTRLD = 0x53

    FRMCTR1 = 0xB1
    FRMCTR2 = 0xB2
    FRMCTR3 = 0xB3
    INVCTR = 0xB4
    DFUNCTR = 0xB6

    PWCTR1 = 0xC0
    PWCTR2 = 0xC1
    PWCTR3 = 0xC2

    VMCTR1 = 0xC5
    VMCOFF = 0xC6

    RDID4 = 0xD3

    GMCTRP1 = 0xE0
    GMCTRN1 = 0xE1

    MADCTL_MY = 0x80
    MADCTL_MX = 0x40
    MADCTL_MV = 0x20
    MADCTL_ML = 0x10
    MADCTL_RGB = 0x00
    MADCTL_BGR = 0x08
    MADCTL_MH = 0x04


class ST7796(object):
    """Representation of an ST7796 TFT LCD."""

    def __init__(
        self,
        width=240,
        height=240,
        invert=True,
        bgr=False,
        fps=0,
        rotation=0,
        offset_left=0,
        offset_top=0,
    ):
        """
        Create an instance of the ST7796V3 display using SPI communication.
        """

        self._spi = request_interface("spi", "ST7796", 0, 10_000_000)
        self._gpio = request_interface("gpio", "ST7796")

        self._pin_dc = self._gpio.get_pin("DC")
        self._pin_rst = self._gpio.get_pin("RST")
        self._pin_dc.set_mode("output_push_pull")
        self._pin_rst.set_mode("output_push_pull")
        self._pin_dc.write(True)  # DC keeps in high
        self._pin_rst.write(True)  # RST keeps in high
        self._pin_bl = self._gpio.get_pin("BLK")
        self._bk_pwm_control = "pwm_output" in self._pin_bl.get_available_pinmode()
        if self._bk_pwm_control:
            self._pin_bl.set_mode("pwm_output")
            self._pin_bl.write_pwm(freq=1000, duty=0, polarity=True)
        else:
            self._pin_bl.set_mode("output_push_pull")
            self._pin_bl.write(False)  # BK keeps in low
        self.WIDTH = width
        self.HEIGHT = height
        self._invert = invert
        self._bgr = bgr
        self._fps = fps
        self._rotation = rotation

        self._offset_left = offset_left
        self._offset_top = offset_top

        self._last_window = (self.WIDTH, self.HEIGHT)
        self._last_img = None

        self.set_brightness(0)
        self._reset()
        self._init()
        self.clear()
        self.set_brightness(1)

    def _send(self, data):
        MAX_SEND = 4080
        if len(data) <= MAX_SEND:
            self._spi.transfer(data)
        else:
            for i in range(0, len(data), MAX_SEND):
                self._spi.transfer(data[i : i + MAX_SEND])

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.turn_off()

    def set_brightness(self, brightness: float):
        """
        Set the brightness of the backlight.
        Only works if enabled software PWM control,
        otherwise can only control on or off.
        """
        if self._bk_pwm_control:
            self._pin_bl.write_pwm_duty(brightness)
        else:
            self._pin_bl.write(brightness > 0)

    def _command(self, data):
        self._pin_dc.write(False)
        self._spi.transfer([data & 0xFF])
        self._pin_dc.write(True)

    def _data(self, data):
        self._spi.transfer([data & 0xFF])

    def _reset(self):
        self._pin_rst.write(True)
        time.sleep(0.1)
        self._pin_rst.write(False)
        time.sleep(0.1)
        self._pin_rst.write(True)
        time.sleep(0.1)

    def _init(self):
        # Initialize the display.
        self._command(0x01)  # Software reset
        time.sleep(0.15)
        self._command(0x11)  # Sleep out
        time.sleep(0.15)

        self._command(0xF0)  # Command Set control
        self._data(0xC3)  # Enable extension command 2 partI

        self._command(0xF0)  # Command Set control
        self._data(0x96)  # Enable extension command 2 partII

        self._set_param(self._bgr, self._rotation, self._invert)
        # self._command(0x36)  # Memory Data Access Control
        # self._data(0x48)  # X-Mirror, Top-Left to right-Buttom, RGB

        self._command(_CMD.PIXFMT)  # Interface Pixel Format
        self._data(0x55)  # Control interface color format set to 16

        self._command(0xB4)  # Column inversion
        self._data(0x01)  # 1-dot inversion

        self._command(0xB6)  # Display Function Control
        self._data(0x80)  # Bypass
        self._data(
            0x02
        )  # Source Output Scan from S1 to S960, Gate Output scan from G1 to G480, scan cycle=2
        self._data(0x3B)  # LCD Drive Line=8*(59+1)

        self._command(0xE8)  # Display Output Ctrl Adjust
        self._data(0x40)
        self._data(0x8A)
        self._data(0x00)
        self._data(0x00)
        self._data(0x29)  # Source eqaulizing period time= 22.5 us
        self._data(0x19)  # Timing for "Gate start"=25 (Tclk)
        self._data(0xA5)  # Timing for "Gate End"=37 (Tclk), Gate driver EQ function ON
        self._data(0x33)

        self._command(0xC1)  # Power control2
        self._data(
            0x06
        )  # VAP(GVDD)=3.85+( vcom+vcom offset), VAN(GVCL)=-3.85+( vcom+vcom offset)

        self._command(0xC2)  # Power control 3
        self._data(
            0xA7
        )  # Source driving current level=low, Gamma driving current level=High

        self._command(0xC5)  # VCOM Control
        self._data(0x18)  # VCOM=0.9

        self.set_window()
        time.sleep(0.120)

        # ST7796 Gamma Sequence
        self._command(0xE0)  # Gamma"+"
        self._data(0xF0)
        self._data(0x09)
        self._data(0x0B)
        self._data(0x06)
        self._data(0x04)
        self._data(0x15)
        self._data(0x2F)
        self._data(0x54)
        self._data(0x42)
        self._data(0x3C)
        self._data(0x17)
        self._data(0x14)
        self._data(0x18)
        self._data(0x1B)

        self._command(0xE1)  # Gamma"-"
        self._data(0xE0)
        self._data(0x09)
        self._data(0x0B)
        self._data(0x06)
        self._data(0x04)
        self._data(0x03)
        self._data(0x2B)
        self._data(0x43)
        self._data(0x42)
        self._data(0x3B)
        self._data(0x16)
        self._data(0x14)
        self._data(0x17)
        self._data(0x1B)

        time.sleep(0.120)

        self._command(0xF0)  # Command Set control
        self._data(0x3C)  # Disable extension command 2 partI

        self._command(0xF0)  # Command Set control
        self._data(0x69)  # Disable extension command 2 partII

        time.sleep(0.120)

        self._command(_CMD.DISPON)  # Display on
        time.sleep(0.120)

    def set_window(self, x0=0, y0=0, x1=None, y1=None):
        """
        Partial refresh for the given window.
        Affects will be kept until the next setting be set.
        """
        if x1 is None:
            x1 = self.WIDTH - 1
        else:
            x1 -= 1

        if y1 is None:
            y1 = self.HEIGHT - 1
        else:
            y1 -= 1

        self._last_window = (x1 - x0 + 1, y1 - y0 + 1)

        y0 += self._offset_top
        y1 += self._offset_top

        x0 += self._offset_left
        x1 += self._offset_left

        self._command(_CMD.CASET)  # Column addr set
        self._data(x0 >> 8)
        self._data(x0)  # XSTART
        self._data(x1 >> 8)
        self._data(x1)  # XEND
        self._command(_CMD.RASET)  # Row addr set
        self._data(y0 >> 8)
        self._data(y0)  # YSTART
        self._data(y1 >> 8)
        self._data(y1)  # YEND
        self._command(_CMD.RAMWR)  # write to RAM

    def _set_param(self, bgr, rotation, invert):
        data = 0x02
        # data |= 0x04  # Latch Data Order
        data |= 0x10  # Line Address Order
        if bgr:
            data |= _CMD.MADCTL_BGR
        if rotation == 90:
            data |= _CMD.MADCTL_MV | _CMD.MADCTL_MY
        elif rotation == 180:
            data |= _CMD.MADCTL_MX | _CMD.MADCTL_MY
        elif rotation == 270:
            data |= _CMD.MADCTL_MX | _CMD.MADCTL_MV
        elif rotation == 0:
            pass
        else:
            raise ValueError("Invalid rotation value: %s" % self._rotation)
        self._command(_CMD.MADCTL)
        self._data(data & 0xFF)
        if invert:
            self._command(_CMD.INVON)  # Invert display
        else:
            self._command(_CMD.INVOFF)  # Don't invert display

    def display(self, image):
        """
        Write the provided image to the hardware.
        image: OpenCV image in BGR format and same as screen resolution
        """
        data = cv2.cvtColor(image, cv2.COLOR_BGR2BGR565)  # It actually trans to RGB565
        self._send(np.array(data).astype(np.int16).flatten().tolist())

    def fit_display(self, image, keep_ratio=True, no_enlarge=True):
        """
        Write the provided image to the hardware.
        image: OpenCV image in BGR format in any resolution
        """
        image = self._fit_image(image, keep_ratio, no_enlarge)
        self._fit_window(image)
        self.display(image)

    def _fit_image(self, image, keep_ratio=True, no_enlarge=True):
        """
        Change image resolution to fit screen resolution
        """
        target_size = (self.WIDTH, self.HEIGHT)
        image_size = (image.shape[1], image.shape[0])
        if target_size == image_size:
            return image
        if not keep_ratio:
            image = cv2.resize(image, target_size)
            return image
        x_ratio = target_size[0] / image_size[0]
        y_ratio = target_size[1] / image_size[1]
        min_ratio = min(x_ratio, y_ratio)
        if min_ratio >= 1 and no_enlarge:
            pass
        elif min_ratio >= 1:
            image = cv2.resize(image, dsize=(0, 0), fx=min_ratio, fy=min_ratio)
        else:
            image = cv2.resize(image, dsize=(0, 0), fx=min_ratio, fy=min_ratio)
        return image

    def _fit_window(self, image):
        target_size = (self.WIDTH, self.HEIGHT)
        image_size = (image.shape[1], image.shape[0])
        assert (
            image_size[0] <= target_size[0] and image_size[1] <= target_size[1]
        ), "Image is too large"
        dx = target_size[0] - image_size[0]
        dy = target_size[1] - image_size[1]
        x0 = dx // 2
        y0 = dy // 2
        x1 = x0 + image_size[0]
        y1 = y0 + image_size[1]
        self.set_window(x0, y0, x1, y1)

    def _find_diff(self, img1, img2):
        non_black_pixels = np.argwhere((img1 - img2) != 0)
        if len(non_black_pixels) > 0:
            x1 = np.min(non_black_pixels[:, 1])  # type: ignore
            y1 = np.min(non_black_pixels[:, 0])  # type: ignore
            x2 = np.max(non_black_pixels[:, 1])  # type: ignore
            y2 = np.max(non_black_pixels[:, 0])  # type: ignore
            return int(x1), int(y1), int(x2) + 1, int(y2) + 1
        else:
            return None

    def display_diff(self, image):
        """
        Write only the different part between the provided image and the last image to the hardware.
        image: OpenCV image in BGR format and same as screen resolution
        """
        if self._last_img is None:
            self.display(image)
            self._last_img = image
            return
        diff = self._find_diff(self._last_img, image)
        self._last_img = image.copy()
        if diff is not None:
            self.set_window(*diff)
            self.display(image[diff[1] : diff[3], diff[0] : diff[2]])

    def clear(self):
        """
        Clear all frame buffer
        """
        self.set_window(0, 0)
        self._send(
            np.zeros((self.WIDTH, self.HEIGHT, 2)).astype(np.int16).flatten().tolist()
        )

    def clear_window(self):
        """
        Clear the current window
        """
        self._send(
            np.zeros((self._last_window[0], self._last_window[1], 2))
            .astype(np.int16)
            .flatten()
            .tolist()
        )

    def turn_off(self):
        """
        Turn off the display
        """
        self.clear()
        self._command(_CMD.DISPOFF)
        self.set_brightness(0)
