import atexit
import time

from loguru import logger
from PIL import Image

from drivers.interface import request_interface


class EPD_Base(object):
    # Display resolution
    WIDTH: int = 0
    HEIGHT: int = 0

    def __init__(self, auto_sleep: bool = True):
        self._inited = False
        self._spi = request_interface("spi", "EPD", 0, 4000000)
        self._io = request_interface("gpio", "EPD")
        self._io.set_mode("RST", "output_push_pull")
        self._io.set_mode("DC", "output_push_pull")
        self._io.set_mode("BUSY", "input_no_pull")
        self._auto_sleep = auto_sleep
        self._LWIDTH = (
            int(self.WIDTH / 8) if self.WIDTH % 8 == 0 else int(self.WIDTH / 8) + 1
        )
        self._empty_buf = [0xFF] * self.HEIGHT * self._LWIDTH
        atexit.register(self._exit_handler)

    def __enter__(self):
        logger.debug("EPD entered")
        self._init()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.sleep()

    def _exit_handler(self):
        if self._auto_sleep and self._inited:
            self.sleep()
            atexit.unregister(self._exit_handler)

    def _delay_ms(self, delaytime: float):
        time.sleep(delaytime / 1000.0)

    def _reset(self):
        self._io.write("DC", True)
        self._io.write("RST", True)
        self._delay_ms(20)
        self._io.write("RST", False)
        self._delay_ms(20)
        self._io.write("RST", True)
        self._delay_ms(20)

    def _send_command(self, command: int):
        self._io.write("DC", False)
        self._spi.write([command])
        self._io.write("DC", True)

    def _send_data(self, data: int):
        self._spi.write([data])

    def _send_data2(self, data):
        self._spi.write(data)

    def _wait_idle(self):
        while self._io.read("BUSY"):
            time.sleep(0.01)

    def _init_command(self):
        raise NotImplementedError

    def _sleep_command(self):
        raise NotImplementedError

    def _init(self):
        self._reset()
        self._wait_idle()
        self._init_command()
        self._wait_idle()
        self._inited = True

    def _get_image_buffer(self, image, invert: bool = False):
        """
        转换图片为数据
        image: PIL图片/ndarray
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
        self._sleep_command()
        self._delay_ms(400)
        self._inited = False
        logger.info("EPD entered deep-sleep mode")

    def fit_cv2(self, image):
        """
        Convert any cv2 image to EPD compatible image (size and color)
        """
        import cv2
        import numpy as np

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
                    mode="constant",
                    constant_values=255,
                )
            fin_size = (image.shape[1], image.shape[0])
            if fin_size != target_size:
                logger.debug(f"Image size is {fin_size}, force to {target_size}")
                image = cv2.resize(image, target_size, interpolation=cv2.INTER_AREA)
        return image


class EPD_BWR213(EPD_Base):
    """
    Driver for WeAct Studio's 2.13 inch Black/White/Red e-paper display
    """

    WIDTH: int = 122
    HEIGHT: int = 250

    def __init__(self, auto_sleep: bool = True):
        super().__init__(auto_sleep)

    def _init_command(self):
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

    def _sleep_command(self):
        self._send_command(0x10)  # enter deep sleep
        self._send_data(0x01)

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
        clear_none: 是否清除未提供的图层
        wait_idle:  是否阻塞等待屏幕空闲
        """
        if not self._inited:
            self._init()
        if image_black is None and image_red is None:
            return
        if image_black is not None:
            self._set_cursor(0x00, 0x00)
            self._send_command(0x24)
            image_buf_b = self._get_image_buffer(image_black)
            self._send_data2(image_buf_b)
        elif clear_none:
            self._set_cursor(0x00, 0x00)
            self._send_command(0x24)
            self._send_data2(self._empty_buf)
        if image_red is not None:
            self._set_cursor(0x00, 0x00)
            self._send_command(0x26)
            image_buf_r = self._get_image_buffer(image_red, invert=True)
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
        if not self._inited:
            self._init()
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
