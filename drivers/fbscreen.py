import logging
import mmap
import os
import time
import warnings
from ctypes import Structure, c_char, c_short, c_uint, c_ushort
from enum import Enum
from fcntl import ioctl  # type: ignore

import cv2
import numpy as np
from PIL import Image

warnings.filterwarnings("ignore")

logger = logging.getLogger("Screen")


class VirtualTerminal(object):
    # ioctls
    _VT_OPENQRY = 0x5600  # find next available vt
    _VT_GETMODE = 0x5601  # get mode of active vt
    _VT_SETMODE = 0x5602  # set mode of active vt
    _VT_GETSTATE = 0x5603  # get global vt state info
    _VT_SENDSIG = 0x5604  # signal to send to bitmask of vts
    _VT_RELDISP = 0x5605  # release display
    _VT_ACTIVATE = 0x5606  # make vt active
    _VT_WAITACTIVE = 0x5607  # wait for vt active
    _VT_DISALLOCATE = 0x5608  # free memory associated to vt
    _VT_SETACTIVATE = 0x560F  # Activate and set the mode of a console
    _KDSETMODE = 0x4B3A  # set text/graphics mode

    class _VtMode(Structure):
        _fields_ = [
            ("mode", c_char),  # vt mode
            ("waitv", c_char),  # if set, hang on writes if not active
            ("relsig", c_short),  # signal to raise on release request
            ("acqsig", c_short),  # signal to raise on acquisition
            ("frsig", c_short),  # unused (set to 0)
        ]

    class VtMode(Enum):
        AUTO = 0
        PROCESS = 1
        ACKACQ = 2

    class _VtState(Structure):
        _fields_ = [
            ("v_active", c_ushort),  # active vt
            ("v_signal", c_ushort),  # signal to send
            ("v_state", c_ushort),  # vt bitmask
        ]

    class KdMode(Enum):
        TEXT = 0x00
        GRAPHICS = 0x01
        TEXT0 = 0x02  # obsolete
        TEXT1 = 0x03  # obsolete

    def __init__(self, tty="/dev/tty0"):
        self._fd = open(tty, "r")

    def close(self):
        self._fd.close()

    def get_next_available(self):
        n = c_uint()
        ioctl(self._fd, self._VT_OPENQRY, n)
        return n.value

    def activate(self, num):
        ioctl(self._fd, self._VT_ACTIVATE, num)
        ioctl(self._fd, self._VT_WAITACTIVE, num)

    def get_active(self):
        state = VirtualTerminal._VtState()
        ioctl(self._fd, self._VT_GETSTATE, state)
        return state.v_active

    def set_graphics_mode(self):
        ioctl(self._fd, self._KDSETMODE, self.KdMode.GRAPHICS.value)

    def set_text_mode(self):
        ioctl(self._fd, self._KDSETMODE, self.KdMode.TEXT.value)


def exit_graphics_mode(tty="/dev/tty1"):
    vt = VirtualTerminal(tty=tty)
    vt.set_text_mode()
    vt.close()
    os.system('sudo bash -c "echo 1 > /sys/class/graphics/fbcon/cursor_blink"')
    print("Exited graphics mode")


def enter_graphics_mode(tty="/dev/tty1"):
    vt = VirtualTerminal(tty=tty)
    vt.set_graphics_mode()
    vt.close()
    print("Entered graphics mode")


class FbScreen:
    def __init__(self, framebuffer: str = "fb1", exclusive: bool = True) -> None:
        self._fbname = framebuffer
        self._exclusive = exclusive
        virtual_size = open(
            f"/sys/class/graphics/{framebuffer}/virtual_size", "r"
        ).read()
        width, height = virtual_size.split(",")
        self.width = int(width)
        self.height = int(height)
        bpp = open(f"/sys/class/graphics/{framebuffer}/bits_per_pixel", "r").read()
        self.bpp = int(bpp)
        self._fb_path = f"/dev/{framebuffer}"
        self._fbdev = os.open(self._fb_path, os.O_RDWR)
        if exclusive:
            self._tty = VirtualTerminal(self._fb_path.replace("fb", "tty"))
            self._tty.set_graphics_mode()
        else:
            self._tty = None
        self._fb = mmap.mmap(
            self._fbdev,
            self.width * self.height * self.bpp // 8,
            mmap.MAP_SHARED,
            mmap.PROT_WRITE | mmap.PROT_READ,
            offset=0,
        )
        logger.info(
            f"Screen inited: {self._fb_path}@{self.width}x{self.height} bpp={self.bpp}"
        )

    def __enter__(self):
        if self._exclusive:
            self.clear()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        ...

    def __del__(self):
        self.close()

    def close(self):
        if self._exclusive:
            self._tty.set_text_mode()
            self._tty.close()
        self._fb.close()
        logger.info(f"Screen closed")
        # os.system('sudo bash -c "echo 1 > /sys/class/graphics/fbcon/cursor_blink"')

    def _write_byte(self, data: bytes) -> None:
        self._fb.seek(0)
        self._fb.write(data)

    def _write_partial(
        self, data: bytes, x: int, y: int, width: int, height: int
    ) -> None:
        byte_width = self.bpp // 8
        for i in range(height):
            self._fb.seek((y + i) * self.width * byte_width + x * byte_width)
            self._fb.write(data[i * width * byte_width : (i + 1) * width * byte_width])

    def _pil_to_cv2(self, image: Image.Image) -> None:
        """
        Convert PIL image to cv2 image
        """
        image = image.convert("RGB")
        cv2_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        return cv2_image

    def read(self) -> np.ndarray:
        """
        Read the screen and return cv2 image
        """
        # from subprocess import check_output

        # data = bytearray(check_output(f"cat /dev/{self._fbname}", shell=True))
        self._fb.seek(0)
        data = self._fb.read(self.width * self.height * self.bpp // 8)
        frame = np.frombuffer(data, dtype=np.uint8)
        frame = frame.reshape((self.height, self.width, self.bpp // 8))
        return cv2.cvtColor(frame, cv2.COLOR_BGR5652BGR)

    def clear(self) -> None:
        """
        Clear the screen
        """
        self._write_byte(b"\0" * (self.width * self.height * self.bpp // 8))

    def fill(self, color="#000000") -> None:
        """
        Fill the screen with a color
        """
        c = color.replace("#", "")
        r = int(c[0:2], 16) >> 3
        g = int(c[2:4], 16) >> 2
        b = int(c[4:6], 16) >> 3
        rgb565 = (r << 11) | (g << 5) | b
        firstbit = (rgb565 >> 8) & 0xFF
        secondbit = rgb565 & 0xFF
        data = bytearray([secondbit, firstbit] * self.width * self.height)
        self._write_byte(data)

    def set_blank(self, blank=True) -> None:
        """
        Set screen blanking
        """
        if blank:
            os.system(
                f"sudo bash -c 'echo 1 > /sys/class/graphics/{self._fbname}/blank'"
            )
        else:
            os.system(
                f"sudo bash -c 'echo 0 > /sys/class/graphics/{self._fbname}/blank'"
            )

    def display(self, image):
        """
        Write the provided image to the hardware.
        image: cv2 or PIL image ***in screen resolution***
        """
        if isinstance(image, Image.Image):
            image = self._pil_to_cv2(image)
        data = bytearray(cv2.cvtColor(image, cv2.COLOR_BGR2BGR565))
        self._write_byte(data)

    def partial_display(self, image, x: int, y: int):
        """
        Write the provided image to the hardware.
        image: cv2 or PIL image smaller than screen resolution
        x, y: top left corner of the image
        """
        if isinstance(image, Image.Image):
            image = self._pil_to_cv2(image)
        data = bytearray(cv2.cvtColor(image, cv2.COLOR_BGR2BGR565))
        height, width = image.shape[:2]
        self._write_partial(data, x, y, width, height)

    def fit_display(self, image, keep_ratio=True, cut=False, enlarge=False):
        """
        Write the provided image to the hardware.
        image: cv2 or PIL image in any resolution
        keep_ratio: keep image ratio when resizing

        Available modes when keep_ratio is True:
        cut: cut image to fit screen
        enlarge: enlarge image if it is smaller than screen
        """
        if isinstance(image, Image.Image):
            image = self._pil_to_cv2(image)
        image = self._fit_image(image, keep_ratio, cut, enlarge)
        data = bytearray(cv2.cvtColor(image, cv2.COLOR_BGR2BGR565))
        self._write_byte(data)

    def _fit_image(self, image, keep_ratio, cut, enlarge):
        target_size = (self.width, self.height)
        image_size = (image.shape[1], image.shape[0])
        if target_size == image_size:
            return image
        if not keep_ratio:
            image = cv2.resize(image, target_size)
            return image
        x_ratio = target_size[0] / image_size[0]
        y_ratio = target_size[1] / image_size[1]
        ratio = max(x_ratio, y_ratio) if cut else min(x_ratio, y_ratio)
        if ratio < 1 or enlarge:
            image = cv2.resize(image, dsize=(0, 0), fx=ratio, fy=ratio)
        new_size = (image.shape[1], image.shape[0])
        if new_size != target_size:
            if new_size[0] > target_size[0] or new_size[1] > target_size[1]:
                top_cut = (new_size[1] - target_size[1]) // 2
                bottom_cut = new_size[1] - target_size[1] - top_cut
                left_cut = (new_size[0] - target_size[0]) // 2
                right_cut = new_size[0] - target_size[0] - left_cut
                if bottom_cut > 0 and right_cut > 0:
                    image = image[top_cut:-bottom_cut, left_cut:-right_cut]
                elif bottom_cut > 0:
                    image = image[top_cut:-bottom_cut, left_cut:]
                elif right_cut > 0:
                    image = image[top_cut:, left_cut:-right_cut]
                else:
                    image = image[top_cut:, left_cut:]
            else:
                left = int((target_size[0] - new_size[0]) / 2)
                right = target_size[0] - new_size[0] - left
                top = int((target_size[1] - new_size[1]) / 2)
                bottom = target_size[1] - new_size[1] - top
                image = cv2.copyMakeBorder(
                    image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0
                )
        final_size = (image.shape[1], image.shape[0])
        assert (
            final_size == target_size
        ), f"Failed to fit image: {final_size} != {target_size}"
        return image


_toast_thread = None


def show_toast(text, delay=2, color=(200, 255, 255)):
    with FbScreen(exclusive=False) as screen:
        text_width, text_height = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1
        )[0]
        height = text_height + 16
        width = text_width + 16
        x0 = (screen.width - width) // 2
        backup_img = screen.read()
        backup_img = backup_img[:height, x0 : x0 + width, :]
        img = np.zeros_like(backup_img)
        cv2.putText(
            img,
            text,
            (5, text_height + 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (int(color[2]), int(color[1]), int(color[0])),
            1,
        )
        screen.partial_display(img, x=x0, y=0)
        time.sleep(delay)
        screen.partial_display(backup_img, x=x0, y=0)


def show_toast_thread(text, delay=2, color=(200, 255, 255)):
    import threading

    global _toast_thread
    if _toast_thread is not None and _toast_thread.is_alive():
        _toast_thread.join()
    _toast_thread = threading.Thread(target=show_toast, args=(text, delay, color))
    _toast_thread.start()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    print("screen initialized")

    class fps_counter:
        def __init__(self, max_sample=40) -> None:
            self.t = time.time()
            self.max_sample = max_sample
            self.t_list = []

        def update(self) -> None:
            self.t_list.append(time.time() - self.t)
            self.t = time.time()
            if len(self.t_list) > self.max_sample:
                self.t_list.pop(0)

        def get(self) -> float:
            length = len(self.t_list)
            sum_t = sum(self.t_list)
            if length == 0:
                return 0.0
            else:
                return length / sum_t

    def circle_test():
        with FbScreen() as screen:
            width = screen.width
            height = screen.height
            test_img = np.zeros((height, width, 3), dtype=np.uint8)
            fps = fps_counter()
            start_time = time.time()
            circle_num = 3
            x = [width // 2 for _ in range(circle_num)]
            y = [height // 2 for _ in range(circle_num)]
            add_x = [1, 2, -2, -3]
            add_y = [1, -3, 1, -4]
            r = [60, 30, 20, 10]
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
                    cv2.circle(img, (int(x[i]), int(y[i])), r[i], colors[i], 2)
                cv2.putText(
                    img,
                    f"FPS: {fps.get():.2f} Ts: {time.time() - start_time:.0f}",
                    (30, 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    1,
                )
                t0 = time.time()
                screen.display(img)
                dt = time.time() - t0
                print(f"FPS: {fps.get():.2f} dt: {dt:.3f}")
                fps.update()

    try:
        circle_test()
    except KeyboardInterrupt:
        pass
    # show_toast("Test", 2)
