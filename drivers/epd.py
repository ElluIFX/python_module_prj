import atexit
import time

import cv2
import numpy as np
from loguru import logger
from PIL import Image

from drivers.interface import request_interface


class MODE:
    uninit = 0
    full = 1
    partial = 2
    grayscale = 3
    fast = 4


class EPD_Base(object):
    # Display resolution
    WIDTH: int = 0
    HEIGHT: int = 0

    def __init__(self, auto_sleep: bool = True):
        self._mode = MODE.uninit
        self._spi = request_interface("spi", "EPD", 0, 4_000_000)
        self._io = request_interface("gpio", "EPD")
        self._io.set_mode("RST", "output_push_pull")
        self._io.set_mode("DC", "output_push_pull")
        self._io.set_mode("BUSY", "input_no_pull")
        try:
            self._io.set_mode("CS", "output_push_pull")
            self._io.write("CS", True)
            self._cs = True
            logger.debug("EPD Soft-CS enabled")
        except Exception:
            self._cs = False
            logger.debug("EPD Soft-CS disabled")
        self._auto_sleep = auto_sleep
        self._LWIDTH = int(self.WIDTH / 8) + (0 if self.WIDTH % 8 == 0 else 1)
        self._empty_buf = [0xFF] * self.HEIGHT * self._LWIDTH
        atexit.register(self._exit_handler)

    def __enter__(self):
        logger.debug("EPD entered")
        self._init()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.sleep()

    def _exit_handler(self):
        if self._auto_sleep and self._mode:
            self.sleep()
            atexit.unregister(self._exit_handler)

    def _delay_ms(self, delaytime: float):
        time.sleep(delaytime / 1000.0)

    def _reset(self):
        self._io.write("DC", True)
        self._io.write("RST", True)
        self._delay_ms(10)
        self._io.write("RST", False)
        self._delay_ms(5)
        self._io.write("RST", True)
        self._delay_ms(10)
        self._wait_idle()

    def _send_command(self, command: int):
        self._io.write("DC", False)
        if self._cs:
            self._io.write("CS", False)
        self._spi.write([command])
        if self._cs:
            self._io.write("CS", True)
        self._io.write("DC", True)

    def _send_data(self, data: int):
        if self._cs:
            self._io.write("CS", False)
        self._spi.write([data])
        if self._cs:
            self._io.write("CS", True)

    def _send_data2(self, data):
        if self._cs:
            self._io.write("CS", False)
        self._spi.write(data)
        if self._cs:
            self._io.write("CS", True)

    def _wait_idle(self):
        while self._io.read("BUSY"):
            time.sleep(0.01)

    def _init_full_cmd(self):
        raise NotImplementedError

    def _init_partial_cmd(self):
        raise NotImplementedError

    def _init_grayscale_cmd(self):
        raise NotImplementedError

    def _init_fast_cmd(self):
        raise NotImplementedError

    def _sleep_cmd(self):
        self._send_command(0x10)
        self._send_data(0x01)

    def _init(self, target_mode: int = MODE.full, reset: bool = True):
        logger.debug(f"EPD init to mode {target_mode}")
        if reset:
            self._reset()
            self._wait_idle()
        {
            MODE.full: self._init_full_cmd,
            MODE.partial: self._init_partial_cmd,
            MODE.grayscale: self._init_grayscale_cmd,
            MODE.fast: self._init_fast_cmd,
        }[target_mode]()
        self._wait_idle()
        self._mode = target_mode

    def _img_to_data_bw(self, image, invert: bool = False):
        """
        转换图片为数据 (黑白屏 双色模式)
        image: PIL图片/cv2 ndarray
        invert: 数据按位取反
        """
        if not isinstance(image, Image.Image):
            image = Image.fromarray(image)
        imwidth, imheight = image.size
        if imwidth == self.WIDTH and imheight == self.HEIGHT:
            img = image.convert("1")
        elif imwidth == self.HEIGHT and imheight == self.WIDTH:
            img = image.rotate(90, expand=True).convert("1")
        else:
            raise ValueError(f"Image size error: {imwidth}x{imheight}")
        img.save("test.bmp")
        if not invert:
            return bytearray(img.tobytes("raw"))
        else:
            return bytearray((~i) & 0xFF for i in img.tobytes("raw"))

    def _img_to_data_bw_raw(self, image):
        """
        转换图片为数据 (黑白屏 双色模式)
        image: PIL图片/cv2 ndarray
        """
        if not isinstance(image, Image.Image):
            image = Image.fromarray(image)
        imwidth, imheight = image.size
        if imwidth == self.WIDTH and imheight == self.HEIGHT:
            pass
        elif imwidth == self.HEIGHT and imheight == self.WIDTH:
            image = image.rotate(90, expand=True)
            imwidth, imheight = image.size
            assert imwidth == self.WIDTH and imheight == self.HEIGHT
        else:
            raise ValueError(f"Image size error: {imwidth}x{imheight}")
        image_monocolor = image.convert("1")
        pixels = image_monocolor.load()
        buf = [0xFF] * (int(self.WIDTH / 8) * self.HEIGHT)
        for y in range(imheight):
            for x in range(imwidth):
                if pixels[x, y] == 0:
                    buf[int((x + y * self.WIDTH) / 8)] &= ~(0x80 >> (x % 8))
        return buf

    def _img_to_data_4gray(self, image):
        """
        转换图片为数据 (4灰阶屏 灰阶模式)
        image: PIL图片/cv2 ndarray
        """
        if not isinstance(image, Image.Image):
            image = Image.fromarray(image)
        imwidth, imheight = image.size
        if imwidth == self.WIDTH and imheight == self.HEIGHT:
            pass
        elif imwidth == self.HEIGHT and imheight == self.WIDTH:
            image = image.rotate(90, expand=True)
            imwidth, imheight = image.size
            assert imwidth == self.WIDTH and imheight == self.HEIGHT
        else:
            raise ValueError(f"Image size error: {imwidth}x{imheight}")
        image_monocolor = image.convert("L")
        imwidth, imheight = image_monocolor.size
        pixels = image_monocolor.load()
        buf = [0xFF] * (int(self.WIDTH / 4) * self.HEIGHT)
        i = 0
        for y in range(imheight):
            for x in range(imwidth):
                if pixels[x, y] == 0xC0:
                    pixels[x, y] = 0x80
                elif pixels[x, y] == 0x80:
                    pixels[x, y] = 0x40
                i = i + 1
                if i % 4 == 0:
                    buf[int((x + (y * self.WIDTH)) / 4)] = (
                        (pixels[x - 3, y] & 0xC0)
                        | (pixels[x - 2, y] & 0xC0) >> 2
                        | (pixels[x - 1, y] & 0xC0) >> 4
                        | (pixels[x, y] & 0xC0) >> 6
                    )
        return buf

    @property
    def idle(self) -> bool:
        """
        屏幕是否空闲
        """
        return not self._io.read("BUSY")

    def sleep(self):
        """
        使屏幕进入睡眠模式, 下次调用display时会自动唤醒
        """
        self._sleep_cmd()
        self._delay_ms(400)
        self._mode = MODE.uninit
        logger.info("EPD entered deep-sleep mode")

    def fit_cv2(self, image: np.ndarray):
        """
        保持比例转换 cv2 图片为屏幕大小的灰度图
        """

        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_size = (image.shape[1], image.shape[0])
        if img_size[0] > img_size[1]:
            target_size = (max(self.WIDTH, self.HEIGHT), min(self.WIDTH, self.HEIGHT))
        else:
            target_size = (min(self.WIDTH, self.HEIGHT), max(self.WIDTH, self.HEIGHT))
        if img_size != target_size:
            logger.debug(f"Resizing image from {img_size} to {target_size}")
            ratio = min(target_size[0] / img_size[0], target_size[1] / img_size[1])
            image = cv2.resize(
                image,
                (int(img_size[0] * ratio), int(img_size[1] * ratio)),
                interpolation=cv2.INTER_AREA,
            )
            new_size = (image.shape[1], image.shape[0])
            if new_size[0] < target_size[0]:
                left_pad = int(target_size[0] - new_size[0]) // 2
                right_pad = int(target_size[0] - new_size[0] - left_pad)
                image = np.pad(  # type: ignore
                    image,
                    ((0, 0), (left_pad, right_pad)),  # type: ignore
                    mode="constant",
                    constant_values=255,
                )
            elif new_size[1] < target_size[1]:
                top_pad = int(target_size[1] - new_size[1]) // 2
                bottom_pad = int(target_size[1] - new_size[1] - top_pad)
                image = np.pad(  # type: ignore
                    image,
                    ((top_pad, bottom_pad), (0, 0)),  # type: ignore
                    mode="constant",  # type: ignore
                    constant_values=255,
                )
            fin_size = (image.shape[1], image.shape[0])
            if fin_size != target_size:
                logger.debug(f"Image size is {fin_size}, force to {target_size}")
                image = cv2.resize(image, target_size, interpolation=cv2.INTER_AREA)
        return image


class EPD_BWR_213(EPD_Base):
    """
    Driver for WeAct Studio's 2.13 inch Black/White/Red e-paper display
    Screen Type: Unknown
    """

    WIDTH: int = 122
    HEIGHT: int = 250

    def _init_full_cmd(self):
        self._send_command(0x12)  # SWRESET
        self._wait_idle()
        self._send_command(0x01)  # Driver output control
        self._send_data(0x27)
        self._send_data(0x01)
        self._send_data(0x01)
        self._send_command(0x11)  # data entry mode
        self._send_data(0x01)
        self._send_command(0x44)  # set Ram-X address start/end position
        self._send_data(0x00)
        self._send_data(0x0F)  # 0x0F-->(15+1)*8=128

        self._send_command(0x45)  ## set Ram-Y address start/end position
        self._send_data(0xFF)  ## 0x127-->(295+1)=296
        self._send_data(0x01)
        self._send_data(0x00)
        self._send_data(0x00)

        self._send_command(0x3C)  # BorderWavefrom
        self._send_data(0x05)
        self._send_command(0x21)  # Display update control
        self._send_data(0x00)
        self._send_data(0x80)

        self._send_command(0x18)  # Read built-in temperature sensor
        self._send_data(0x80)

        self._send_command(0x4E)
        self._send_data(0x00)
        self._send_command(0x4F)
        self._send_data(0x27)
        self._send_data(0x10)

    def _trigger_display(self) -> None:
        self._send_command(0x22)
        self._send_data(0xF7)
        self._send_command(0x20)

    def _set_cursor(self, xstart: int, ystart: int):
        xstart //= 8
        ystart = 295 - ystart
        self._send_command(0x4E)  # SET_RAM_X_ADDRESS_COUNTER
        self._send_data(xstart & 0xFF)

        self._send_command(0x4F)  # SET_RAM_Y_ADDRESS_COUNTER
        self._send_data(ystart & 0xFF)
        self._send_data((ystart >> 8) & 0x01)

    def display(
        self,
        image_black=None,
        image_red=None,
        clear_none: bool = False,
        wait_idle: bool = True,
    ) -> None:
        """
        全刷显示图片
        image_b/r: PIL图片/cv2 ndarray (黑/红色)
        clear_none: 是否清除未提供的图层
        wait_idle:  是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.full:
            self._init(MODE.full)
        if image_black is None and image_red is None:
            raise ValueError("At least one image should be provided")
        if image_black is not None:
            self._set_cursor(0x00, 0x00)
            self._send_command(0x24)
            image_buf_b = self._img_to_data_bw(image_black)
            self._send_data2(image_buf_b)
        elif clear_none:
            self._set_cursor(0x00, 0x00)
            self._send_command(0x24)
            self._send_data2(self._empty_buf)
        if image_red is not None:
            self._set_cursor(0x00, 0x00)
            self._send_command(0x26)
            image_buf_r = self._img_to_data_bw(image_red, invert=True)
            self._send_data2(image_buf_r)
        elif clear_none:
            self._set_cursor(0x00, 0x00)
            self._send_command(0x26)
            buf_inv = [(~i) & 0xFF for i in self._empty_buf]
            self._send_data2(buf_inv)
        self._trigger_display()
        if wait_idle:
            self._wait_idle()

    def clear(self, wait_idle: bool = True):
        """
        清屏
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.full:
            self._init(MODE.full)
        self._set_cursor(0x00, 0x00)
        self._send_command(0x24)
        self._send_data2(self._empty_buf)
        buf_inv = [(~i) & 0xFF for i in self._empty_buf]
        self._set_cursor(0x00, 0x00)
        self._send_command(0x26)
        self._send_data2(buf_inv)
        self._trigger_display()
        if wait_idle:
            self._wait_idle()


class EPD_BW_154(EPD_Base):
    """
    Driver for WaveShare's 1.54 inch black/white e-paper display
    Support partial refresh
    """

    WIDTH: int = 200
    HEIGHT: int = 200

    # waveform full refresh
    _lut_full = [
        0x66, 0x66, 0x44, 0x66, 0xAA, 0x11, 0x80, 0x08, 0x11, 0x18, 0x81, 0x18,
        0x11, 0x88, 0x11, 0x88, 0x11, 0x88, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
        0x5F, 0xAF, 0xFF, 0xFF, 0x2F, 0x00
    ]  # fmt: skip

    # waveform partial refresh(fast)
    _lut_partial = [
        0x10, 0x18, 0x18, 0x28, 0x18, 0x18, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x11, 0x22, 0x63,
        0x11, 0x00, 0x00, 0x00, 0x00, 0x00
    ]  # fmt: skip

    def _set_lut(self, lut: list):
        self._send_command(0x32)  # WRITE_LUT_REGISTER
        for data in lut[:30]:
            self._send_data(data)

    def _set_cursor(self, Xstart, Ystart):
        self._send_command(0x4E)  # SET_RAM_X_ADDRESS_COUNTER
        self._send_data(Xstart & 0xFF)
        self._send_command(0x4F)  # SET_RAM_Y_ADDRESS_COUNTER
        self._send_data(Ystart & 0xFF)
        self._send_data((Ystart >> 8) & 0xFF)
        self._wait_idle()

    def _set_window(self, Xstart, Ystart, Xend, Yend):
        self._send_command(0x44)  # SET_RAM_X_ADDRESS_START_END_POSITION
        self._send_data((Xstart >> 3) & 0xFF)
        self._send_data((Xend >> 3) & 0xFF)
        self._send_command(0x45)  # SET_RAM_Y_ADDRESS_START_END_POSITION
        self._send_data(Ystart & 0xFF)
        self._send_data((Ystart >> 8) & 0xFF)
        self._send_data(Yend & 0xFF)
        self._send_data((Yend >> 8) & 0xFF)

    def _init_cmd(self, lut):
        self._send_command(0x01)
        self._send_data((self.HEIGHT - 1) & 0xFF)
        self._send_data(((self.HEIGHT - 1) >> 8) & 0xFF)
        self._send_data(0x00)
        self._send_command(0x0C)
        self._send_data(0xD7)
        self._send_data(0xD6)
        self._send_data(0x9D)
        self._send_command(0x2C)
        self._send_data(0xA8)
        self._send_command(0x3A)
        self._send_data(0x1A)
        self._send_command(0x3B)
        self._send_data(0x08)
        self._send_command(0x11)
        self._send_data(0x03)
        self._set_lut(lut)
        self._set_window(0, 0, self.WIDTH - 1, self.HEIGHT - 1)
        self._set_cursor(0, 0)

    def _init_full_cmd(self):
        self._init_cmd(self._lut_full)

    def _init_partial_cmd(self):
        self._init_cmd(self._lut_partial)

    def _trigger_display_full(self) -> None:
        self._send_command(0x22)  # DISPLAY_UPDATE_CONTROL_2
        self._send_data(0xC7)
        self._send_command(0x20)  # MASTER_ACTIVATION
        self._send_command(0xFF)  # TERMINATE_FRAME_READ_WRITE

    def _trigger_display_partial(self) -> None:
        self._send_command(0x22)  # DISPLAY_UPDATE_CONTROL_2
        self._send_data(0xC7)  # 0xcf ?
        self._send_command(0x20)  # MASTER_ACTIVATION
        self._send_command(0xFF)  # TERMINATE_FRAME_READ_WRITE

    def display(self, image, wait_idle: bool = True):
        """
        全刷显示图片
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.full:
            self._init(MODE.full)
        self._send_command(0x24)
        image_buf = self._img_to_data_bw(image)
        self._send_data2(image_buf)
        self._trigger_display_full()
        if wait_idle:
            self._wait_idle()

    def display_base(self, image, wait_idle=True):
        """
        局刷显示图片/静态部分
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.full:
            self._init(MODE.full)
        image_buf = self._img_to_data_bw(image)
        self._send_command(0x24)
        self._send_data2(image_buf)
        self._send_command(0x26)
        self._send_data2(image_buf)
        self._trigger_display_full()
        if wait_idle:
            self._wait_idle()

    def display_partial(self, image, wait_idle=True):
        """
        局刷显示图片/动态部分
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.partial:
            self._init(MODE.partial)
        image_buf = self._img_to_data_bw(image)
        self._send_command(0x24)
        self._send_data2(image_buf)
        self._trigger_display_partial()
        if wait_idle:
            self._wait_idle()

    def clear(self, wait_idle: bool = True):
        """
        清屏
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.full:
            self._init(MODE.full)
        self._send_command(0x24)
        self._send_data2(self._empty_buf)
        self._send_command(0x26)
        self._send_data2(self._empty_buf)
        self._trigger_display_full()
        if wait_idle:
            self._wait_idle()


class EPD_G4_42(EPD_Base):
    """
    Driver for WaveShare's 4.2 inch 4 Grayscale e-paper display
    Support partial refresh
    Screen Type: E042A13
    """

    WIDTH: int = 400
    HEIGHT: int = 300
    GRAY1 = 0xFF  # white
    GRAY2 = 0xC0  # light gray
    GRAY3 = 0x80  # dark gray
    GRAY4 = 0x00  # black

    _lut_all = [
        0x01, 0x0A, 0x1B, 0x0F, 0x03, 0x01, 0x01, 0x05, 0x0A, 0x01, 0x0A, 0x01,
        0x01, 0x01, 0x05, 0x08, 0x03, 0x02, 0x04, 0x01, 0x01, 0x01, 0x04, 0x04,
        0x02, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x0A, 0x1B, 0x0F, 0x03, 0x01,
        0x01, 0x05, 0x4A, 0x01, 0x8A, 0x01, 0x01, 0x01, 0x05, 0x48, 0x03, 0x82,
        0x84, 0x01, 0x01, 0x01, 0x84, 0x84, 0x82, 0x00, 0x01, 0x01, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
        0x01, 0x0A, 0x1B, 0x8F, 0x03, 0x01, 0x01, 0x05, 0x4A, 0x01, 0x8A, 0x01,
        0x01, 0x01, 0x05, 0x48, 0x83, 0x82, 0x04, 0x01, 0x01, 0x01, 0x04, 0x04,
        0x02, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x8A, 0x1B, 0x8F, 0x03, 0x01,
        0x01, 0x05, 0x4A, 0x01, 0x8A, 0x01, 0x01, 0x01, 0x05, 0x48, 0x83, 0x02,
        0x04, 0x01, 0x01, 0x01, 0x04, 0x04, 0x02, 0x00, 0x01, 0x01, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
        0x01, 0x8A, 0x9B, 0x8F, 0x03, 0x01, 0x01, 0x05, 0x4A, 0x01, 0x8A, 0x01,
        0x01, 0x01, 0x05, 0x48, 0x03, 0x42, 0x04, 0x01, 0x01, 0x01, 0x04, 0x04,
        0x42, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x07,
        0x17, 0x41, 0xA8, 0x32, 0x30
    ]  # fmt: skip

    def _set_lut(self):
        self._send_command(0x32)
        for i in range(227):
            self._send_data(self._lut_all[i])
        self._send_command(0x3F)
        self._send_data(self._lut_all[227])
        self._send_command(0x03)
        self._send_data(self._lut_all[228])
        self._send_command(0x04)
        self._send_data(self._lut_all[229])
        self._send_data(self._lut_all[230])
        self._send_data(self._lut_all[231])
        self._send_command(0x2C)
        self._send_data(self._lut_all[232])

    def _init_full_cmd(self):
        self._send_command(0x12)  # SWRESET
        self._wait_idle()
        self._send_command(0x21)  # Display update control
        self._send_data(0x40)
        self._send_data(0x00)
        self._send_command(0x3C)  # BorderWavefrom
        self._send_data(0x05)
        self._send_command(0x11)  # data  entry  mode
        self._send_data(0x03)  # X-mode
        self._send_command(0x44)
        self._send_data(0x00)
        self._send_data(0x31)
        self._send_command(0x45)
        self._send_data(0x00)
        self._send_data(0x00)
        self._send_data(0x2B)
        self._send_data(0x01)
        self._send_command(0x4E)
        self._send_data(0x00)
        self._send_command(0x4F)
        self._send_data(0x00)
        self._send_data(0x00)

    def _init_partial_cmd(self):
        self._send_command(0x3C)  # BorderWavefrom
        self._send_data(0x80)
        self._send_command(0x21)  # Display update control
        self._send_data(0x00)
        self._send_data(0x00)
        self._send_command(0x3C)  # BorderWavefrom
        self._send_data(0x80)
        self._send_command(0x44)
        self._send_data(0x00)
        self._send_data(0x31)
        self._send_command(0x45)
        self._send_data(0x00)
        self._send_data(0x00)
        self._send_data(0x2B)
        self._send_data(0x01)
        self._send_command(0x4E)
        self._send_data(0x00)
        self._send_command(0x4F)
        self._send_data(0x00)
        self._send_data(0x00)

    def _init_grayscale_cmd(self):
        self._send_command(0x12)  # SWRESET
        self._wait_idle()
        self._send_command(0x21)  # Display update control
        self._send_data(0x00)
        self._send_data(0x00)
        self._send_command(0x3C)  # BorderWavefrom
        self._send_data(0x03)
        self._send_command(0x0C)  # BTST
        self._send_data(0x8B)  # 8B
        self._send_data(0x9C)  # 9C
        self._send_data(0xA4)  # 96 A4
        self._send_data(0x0F)  # 0F
        self._set_lut()
        self._send_command(0x11)  # data  entry  mode
        self._send_data(0x03)  # X-mode
        self._send_command(0x44)
        self._send_data(0x00)
        self._send_data(0x31)
        self._send_command(0x45)
        self._send_data(0x00)
        self._send_data(0x00)
        self._send_data(0x2B)
        self._send_data(0x01)
        self._send_command(0x4E)
        self._send_data(0x00)
        self._send_command(0x4F)
        self._send_data(0x00)
        self._send_data(0x00)
        self._wait_idle()

    def _init_fast_cmd(self):
        self._send_command(0x12)  # SWRESET
        self._wait_idle()

        self._send_command(0x21)  # Display update control
        self._send_data(0x40)
        self._send_data(0x00)

        self._send_command(0x3C)  # BorderWavefrom
        self._send_data(0x05)

        self._send_command(0x1A)
        # self._send_data(0x6E)  # 1.5s
        self._send_data(0x5A)  # 1s

        self._send_command(0x22)  # Load temperature value
        self._send_data(0x91)
        self._send_command(0x20)
        self._wait_idle()

        self._send_command(0x11)  # data  entry  mode
        self._send_data(0x03)  # X-mode

        self._send_command(0x44)
        self._send_data(0x00)
        self._send_data(0x31)

        self._send_command(0x45)
        self._send_data(0x00)
        self._send_data(0x00)
        self._send_data(0x2B)
        self._send_data(0x01)

        self._send_command(0x4E)
        self._send_data(0x00)

        self._send_command(0x4F)
        self._send_data(0x00)
        self._send_data(0x00)
        self._wait_idle()

    def _trigger_display_full(self):
        self._send_command(0x22)  # DISPLAY_UPDATE_CONTROL_2
        self._send_data(0xF7)
        self._send_command(0x20)  # MASTER_ACTIVATION

    def _trigger_display_fast(self):
        self._send_command(0x22)  # Display Update Control
        self._send_data(0xC7)
        self._send_command(0x20)  # Activate Display Update Sequence

    def _trigger_display_grayscale(self):
        self._send_command(0x22)  # Display Update Control
        self._send_data(0xCF)
        self._send_command(0x20)  # Activate Display Update Sequence

    def _trigger_display_partial(self):
        self._send_command(0x22)  # Display Update Control
        self._send_data(0xFF)
        self._send_command(0x20)  # Activate Display Update Sequence

    def _process_4gray(self, image: list):
        data_a, data_b = [], []
        map_a = {0xC0: 0x01, 0x00: 0x00, 0x80: 0x00}
        map_b = {0xC0: 0x01, 0x00: 0x00, 0x80: 0x01}
        for i in range(0, 4736):
            bit = 0
            for j in range(0, 2):
                temp = image[i * 2 + j]
                for k in range(0, 2):
                    bit |= map_a.get(temp & 0xC0, 0x01)
                    bit <<= 1

                    temp <<= 2
                    bit |= map_a.get(temp & 0xC0, 0x01)
                    if j != 1 or k != 1:
                        bit <<= 1
                    temp <<= 2
            data_a.append(bit & 0xFF)
        for i in range(0, 4736):
            bit = 0
            for j in range(0, 2):
                temp = image[i * 2 + j]
                for k in range(0, 2):
                    bit |= map_b.get(temp & 0xC0, 0x00)
                    bit <<= 1

                    temp <<= 2
                    bit |= map_b.get(temp & 0xC0, 0x00)
                    if j != 1 or k != 1:
                        bit <<= 1
                    temp <<= 2
            data_b.append(bit & 0xFF)
        return data_a, data_b

    def display(self, image, wait_idle=True):
        """
        全刷显示图片 (双色模式)
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.full:
            self._init(MODE.full)
        image_buf = self._img_to_data_bw(image)
        self._send_command(0x24)
        self._send_data2(image_buf)
        self._send_command(0x26)
        self._send_data2(image_buf)
        self._trigger_display_full()
        if wait_idle:
            self._wait_idle()

    def display_base(self, image, wait_idle=True):
        """
        局刷显示图片/静态部分 (双色模式)
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        self.display(image, wait_idle)

    def display_partial(self, image, wait_idle=True):
        """
        局刷显示图片/动态部分 (双色模式)
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.partial:
            self._init(MODE.partial, reset=False)
        image_buf = self._img_to_data_bw(image)
        self._send_command(0x24)
        self._send_data2(image_buf)
        self._trigger_display_partial()
        if wait_idle:
            self._wait_idle()

    def display_grayscale(self, image, wait_idle=True):
        """
        全刷显示图片 (灰阶模式)
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.grayscale:
            self._init(MODE.grayscale)
        image_buf = self._img_to_data_4gray(image)
        data_a, data_b = self._process_4gray(image_buf)
        self._send_command(0x24)
        self._send_data2(data_a)
        self._send_command(0x26)
        self._send_data2(data_b)
        self._trigger_display_grayscale()
        if wait_idle:
            self._wait_idle()

    def display_fast(self, image, wait_idle=True):
        """
        全刷显示图片 (快速模式)
        image: PIL图片/cv2 ndarray
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.fast:
            self._init(MODE.fast)
        image_buf = self._img_to_data_bw(image)
        self._send_command(0x24)
        self._send_data2(image_buf)
        self._send_command(0x26)
        self._send_data2(image_buf)
        self._trigger_display_fast()
        if wait_idle:
            self._wait_idle()

    def clear(self, wait_idle=True):
        """
        清屏
        wait_idle: 是否阻塞等待屏幕空闲
        """
        if self._mode != MODE.full:
            self._init(MODE.full)
        self._send_command(0x24)
        self._send_data2(self._empty_buf)
        self._send_command(0x26)
        self._send_data2(self._empty_buf)
        self._trigger_display_full()
        if wait_idle:
            self._wait_idle()
