import time
from typing import Literal

import cv2
import numpy as np

from .interface import request_interface


class LCDBase(object):
    def __init__(
        self,
        width: int,
        height: int,
        invert: bool,
        bgr: bool,
        rotation: Literal[0, 90, 180, 270],
        offset_left: int = 0,
        offset_top: int = 0,
    ):
        """
        Create an instance of the ST7789V3 display using SPI communication.
        """

        self._spi = request_interface("spi", "ST7789", 0, 100_000_000)
        self._gpio = request_interface("gpio", "ST7789")

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
        self._rotation = rotation

        self._offset_left = offset_left
        self._offset_top = offset_top

        self._last_window = (self.WIDTH, self.HEIGHT)
        self._last_window_pos = (0, 0)
        self._last_img = None

        self.set_brightness(0)
        self.reset_hardware()
        self._init_cmd()
        self.clear()
        self.set_brightness(1)

    def _init_cmd(self):
        raise NotImplementedError

    def _sleep_cmd(self):
        raise NotImplementedError

    def _set_window_cmd(self, x0, y0, x1, y1):
        raise NotImplementedError

    def _cmd(self, data):
        self._pin_dc.write(False)
        self._spi.write([data & 0xFF])
        self._pin_dc.write(True)

    def _data(self, data):
        if isinstance(data, int):
            data = [data & 0xFF]
        self._spi.write(data)

    def _write(self, data):
        MAX_SEND = 4080
        if len(data) <= MAX_SEND:
            self._spi.write(data)
        else:
            for i in range(0, len(data), MAX_SEND):
                self._spi.write(data[i : i + MAX_SEND])

    def reset_hardware(self):
        self._pin_rst.write(True)
        time.sleep(0.1)
        self._pin_rst.write(False)
        time.sleep(0.1)
        self._pin_rst.write(True)
        time.sleep(0.1)

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
        self._last_window_pos = (x0, y0)

        y0 += self._offset_top
        y1 += self._offset_top

        x0 += self._offset_left
        x1 += self._offset_left

        self._set_window_cmd(x0, y0, x1, y1)

    def fit_image(self, image, keep_ratio, no_enlarge):
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

    def fit_window(self, image, set_window=True):
        """
        Change screen buffer window to fit image resolution
        """
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
        if set_window:
            self.set_window(x0, y0, x1, y1)
        return x0, y0, x1, y1

    def display_raw(self, image):
        data = cv2.cvtColor(image, cv2.COLOR_BGR2BGR565)  # It actually trans to RGB565
        self._write(np.array(data).astype(np.int16).flatten().tolist())

    def display(self, image, fit=True, keep_ratio=True, no_enlarge=True):
        """
        Write the provided image to the hardware.
        image: OpenCV image in BGR format and same as screen resolution
        fit: If True, the image will be resized to fit the screen resolution
        keep_ratio: Keep the ratio of the image when fit is True
        no_enlarge: If True, the image will not be enlarged when fit is True
        """
        if fit and image.shape[:2] != (self.WIDTH, self.HEIGHT):
            image = self.fit_image(image, keep_ratio, no_enlarge)
            self.fit_window(image)
        elif self._last_window != (self.WIDTH, self.HEIGHT):
            self.set_window(0, 0)
        self.display_raw(image)

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

    def display_diff(self, image, fit=True, keep_ratio=True, no_enlarge=True):
        """
        Write only the different part between the provided image and the last image to the hardware.
        image: OpenCV image in BGR format and same as screen resolution
        fit: If True, the image will be resized to fit the screen resolution
        keep_ratio: Keep the ratio of the image when fit is True
        no_enlarge: If True, the image will not be enlarged when fit is True
        """
        if fit and image.shape[:2] != (self.WIDTH, self.HEIGHT):
            image = self.fit_image(image, keep_ratio, no_enlarge)
            xb, yb, _, _ = self.fit_window(image)
        else:
            xb, yb = 0, 0
        if self._last_img is None:
            self.display(image)
            self._last_img = image
            return
        diff = self._find_diff(self._last_img, image)
        self._last_img = image.copy()
        if diff is not None:
            x1, y1, x2, y2 = diff
            self.set_window(x1 + xb, y1 + yb, x2 + xb, y2 + yb)
            self.display_raw(image[y1:y2, x1:x2])

    def clear(self):
        """
        Clear all frame buffer
        """
        self.set_window(0, 0)
        self._write(
            np.zeros((self.WIDTH, self.HEIGHT, 2)).astype(np.int16).flatten().tolist()
        )

    def clear_window(self):
        """
        Clear the current window
        """
        self._write(
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
        self._sleep_cmd()
        self.set_brightness(0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.turn_off()


class _ST7789V:
    # refer to ST7789V datasheet for details
    NOP = 0x00
    SWRESET = 0x01
    RDDID = 0x04
    RDDST = 0x09

    SLPIN = 0x10
    SLPOUT = 0x11
    PTLON = 0x12
    NORON = 0x13

    INVOFF = 0x20
    INVON = 0x21
    DISPOFF = 0x28
    DISPON = 0x29

    CASET = 0x2A
    RASET = 0x2B
    RAMWR = 0x2C
    RAMRD = 0x2E

    PTLAR = 0x30
    MADCTL = 0x36
    COLMOD = 0x3A

    RAMCTRL = 0xB0
    FRMCTR1 = 0xB1
    FRMCTR2 = 0xB2
    FRMCTR3 = 0xB3
    INVCTR = 0xB4
    DISSET5 = 0xB6

    GCTRL = 0xB7
    GTADJ = 0xB8
    VCOMS = 0xBB

    LCMCTRL = 0xC0
    IDSET = 0xC1
    VDVVRHEN = 0xC2
    VRHS = 0xC3
    VDVS = 0xC4
    VMCTR1 = 0xC5
    FRCTRL2 = 0xC6
    CABCCTRL = 0xC7

    RDID1 = 0xDA
    RDID2 = 0xDB
    RDID3 = 0xDC
    RDID4 = 0xDD

    GMCTRP1 = 0xE0
    GMCTRN1 = 0xE1

    PWCTR6 = 0xFC

    MADCTL_BGR = 0x08
    MADCTL_MV = 0x20
    MADCTL_MX = 0x40
    MADCTL_MY = 0x80


class ST7789V(LCDBase):
    """Representation of an ST7789 TFT LCD."""

    def _init_cmd(self):
        # Initialize the display.
        self._cmd(_ST7789V.SWRESET)  # Software reset
        time.sleep(0.15)

        self._set_param_cmd(self._bgr, self._rotation, self._invert)

        self._cmd(_ST7789V.FRMCTR2)  # Frame rate ctrl - idle mode
        self._data(0x0C)
        self._data(0x0C)
        self._data(0x00)
        self._data(0x33)
        self._data(0x33)

        self._cmd(_ST7789V.COLMOD)
        self._data(0x05)

        self._cmd(_ST7789V.GCTRL)
        self._data(0x14)

        self._cmd(_ST7789V.VCOMS)
        self._data(0x37)

        self._cmd(_ST7789V.LCMCTRL)  # Power control
        self._data(0x2C)

        self._cmd(_ST7789V.VDVVRHEN)  # Power control
        self._data(0x01)

        self._cmd(_ST7789V.VRHS)  # Power control
        self._data(0x13)

        self._cmd(_ST7789V.VDVS)  # Power control
        self._data(0x20)

        self._cmd(0xD0)
        self._data(0xA4)
        self._data(0xA1)

        self._cmd(_ST7789V.FRCTRL2)  # FPS control
        self._data(0x0F)  # 60Hz

        self._cmd(_ST7789V.GMCTRP1)  # Set Gamma
        self._data(0xD0)
        self._data(0x04)
        self._data(0x0D)
        self._data(0x11)
        self._data(0x13)
        self._data(0x2B)
        self._data(0x3F)
        self._data(0x54)
        self._data(0x4C)
        self._data(0x18)
        self._data(0x0D)
        self._data(0x0B)
        self._data(0x1F)
        self._data(0x23)

        self._cmd(_ST7789V.GMCTRN1)  # Set Gamma
        self._data(0xD0)
        self._data(0x04)
        self._data(0x0C)
        self._data(0x11)
        self._data(0x13)
        self._data(0x2C)
        self._data(0x3F)
        self._data(0x44)
        self._data(0x51)
        self._data(0x2F)
        self._data(0x1F)
        self._data(0x1F)
        self._data(0x20)
        self._data(0x23)

        self._cmd(_ST7789V.RAMCTRL)  # Set RAM Control
        self._data(0x00)
        self._data(0xC8)  # little endian and 65K RGB565

        self._cmd(_ST7789V.SLPOUT)

        self._cmd(_ST7789V.DISPON)  # Display on
        time.sleep(0.1)
        self.set_window(0, 0)

    def _set_param_cmd(self, bgr, rotation, invert):
        data = 0x02
        # data |= 0x04  # Latch Data Order
        data |= 0x10  # Line Address Order
        if bgr:
            data |= _ST7789V.MADCTL_BGR
        if rotation == 90:
            data |= _ST7789V.MADCTL_MV | _ST7789V.MADCTL_MY
        elif rotation == 180:
            data |= _ST7789V.MADCTL_MX | _ST7789V.MADCTL_MY
        elif rotation == 270:
            data |= _ST7789V.MADCTL_MX | _ST7789V.MADCTL_MV
        elif rotation == 0:
            pass
        else:
            raise ValueError("Invalid rotation value: %s" % self._rotation)
        self._cmd(_ST7789V.MADCTL)
        self._data(data & 0xFF)
        if invert:
            self._cmd(_ST7789V.INVON)  # Invert display
        else:
            self._cmd(_ST7789V.INVOFF)  # Don't invert display

    def _sleep_cmd(self):
        self._cmd(_ST7789V.SLPIN)

    def _set_window_cmd(self, x0, y0, x1, y1):
        self._cmd(_ST7789V.CASET)  # Column addr set
        self._data(x0 >> 8)
        self._data(x0)  # XSTART
        self._data(x1 >> 8)
        self._data(x1)  # XEND
        self._cmd(_ST7789V.RASET)  # Row addr set
        self._data(y0 >> 8)
        self._data(y0)  # YSTART
        self._data(y1 >> 8)
        self._data(y1)  # YEND
        self._cmd(_ST7789V.RAMWR)  # write to RAM


class _ST7796:
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


class ST7796(LCDBase):
    def _init_cmd(self):
        # Initialize the display.
        self._cmd(0x01)  # Software reset
        time.sleep(0.15)
        self._cmd(0x11)  # Sleep out
        time.sleep(0.15)

        self._cmd(0xF0)  # Command Set control
        self._data(0xC3)  # Enable extension command 2 partI

        self._cmd(0xF0)  # Command Set control
        self._data(0x96)  # Enable extension command 2 partII

        self._set_param_cmd(self._bgr, self._rotation, self._invert)
        # self._cmd(0x36)  # Memory Data Access Control
        # self._data(0x48)  # X-Mirror, Top-Left to right-Buttom, RGB

        self._cmd(_ST7796.PIXFMT)  # Interface Pixel Format
        self._data(0x55)  # Control interface color format set to 16

        self._cmd(0xB4)  # Column inversion
        self._data(0x01)  # 1-dot inversion

        self._cmd(0xB6)  # Display Function Control
        self._data(0x80)  # Bypass
        self._data(
            0x02
        )  # Source Output Scan from S1 to S960, Gate Output scan from G1 to G480, scan cycle=2
        self._data(0x3B)  # LCD Drive Line=8*(59+1)

        self._cmd(0xE8)  # Display Output Ctrl Adjust
        self._data(0x40)
        self._data(0x8A)
        self._data(0x00)
        self._data(0x00)
        self._data(0x29)  # Source eqaulizing period time= 22.5 us
        self._data(0x19)  # Timing for "Gate start"=25 (Tclk)
        self._data(0xA5)  # Timing for "Gate End"=37 (Tclk), Gate driver EQ function ON
        self._data(0x33)

        self._cmd(0xC1)  # Power control2
        self._data(
            0x06
        )  # VAP(GVDD)=3.85+( vcom+vcom offset), VAN(GVCL)=-3.85+( vcom+vcom offset)

        self._cmd(0xC2)  # Power control 3
        self._data(
            0xA7
        )  # Source driving current level=low, Gamma driving current level=High

        self._cmd(0xC5)  # VCOM Control
        self._data(0x18)  # VCOM=0.9

        self.set_window()
        time.sleep(0.120)

        # ST7796 Gamma Sequence
        self._cmd(0xE0)  # Gamma"+"
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

        self._cmd(0xE1)  # Gamma"-"
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

        self._cmd(0xF0)  # Command Set control
        self._data(0x3C)  # Disable extension command 2 partI

        self._cmd(0xF0)  # Command Set control
        self._data(0x69)  # Disable extension command 2 partII

        time.sleep(0.120)

        self._cmd(_ST7796.DISPON)  # Display on
        time.sleep(0.120)

    def _set_param_cmd(self, bgr, rotation, invert):
        data = 0x02
        # data |= 0x04  # Latch Data Order
        data |= 0x10  # Line Address Order
        if bgr:
            data |= _ST7796.MADCTL_BGR
        if rotation == 90:
            data |= _ST7796.MADCTL_MV | _ST7796.MADCTL_MY
        elif rotation == 180:
            data |= _ST7796.MADCTL_MX | _ST7796.MADCTL_MY
        elif rotation == 270:
            data |= _ST7796.MADCTL_MX | _ST7796.MADCTL_MV
        elif rotation == 0:
            pass
        else:
            raise ValueError("Invalid rotation value: %s" % self._rotation)
        self._cmd(_ST7796.MADCTL)
        self._data(data & 0xFF)
        if invert:
            self._cmd(_ST7796.INVON)  # Invert display
        else:
            self._cmd(_ST7796.INVOFF)  # Don't invert display

    def _sleep_cmd(self):
        self._cmd(_ST7796.SLPIN)

    def _set_window_cmd(self, x0, y0, x1, y1):
        self._cmd(_ST7796.CASET)  # Column addr set
        self._data(x0 >> 8)
        self._data(x0)  # XSTART
        self._data(x1 >> 8)
        self._data(x1)  # XEND
        self._cmd(_ST7796.RASET)  # Row addr set
        self._data(y0 >> 8)
        self._data(y0)  # YSTART
        self._data(y1 >> 8)
        self._data(y1)  # YEND
        self._cmd(_ST7796.RAMWR)  # write to RAM
