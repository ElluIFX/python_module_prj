import time
from math import ceil
from typing import TYPE_CHECKING, List, Union

from .interface import request_interface

if TYPE_CHECKING:
    import numpy as np

CMD_DELAY = 0.05
RESET_DELAY = 0.2


class VFD:
    """
    EEI-Technology VFD Screen Driver
    """

    FONT_5x10 = 0
    FONT_6x12 = 1
    FONT_7x15 = 2
    FONT_9x17 = 3
    FONT_12x22 = 4
    FONT_16x29 = 5

    FONT_SIZE = {
        FONT_5x10: (5, 10),
        FONT_6x12: (6, 12),
        FONT_7x15: (7, 15),
        FONT_9x17: (9, 17),
        FONT_12x22: (12, 22),
        FONT_16x29: (16, 29),
    }

    def __init__(self, width: int = 256, height: int = 64) -> None:
        """
        Initialize VFD driver

        Args:
            port (str): Serial port name
            baudrate (int): Serial baudrate
        """
        self._ser = None
        self._spi = None
        self.WIDTH = width
        self.HEIGHT = height
        self.PAGE_NUM = self.HEIGHT // 8

    def _write(self, data: Union[bytes, bytearray]) -> None:
        """
        Write data to screen

        Args:
            data (Union[bytes, bytearray]): data to be written
        """
        if self._ser is not None:
            self._ser.write(data)
        elif self._spi is not None:
            self._spi.write(data)
        else:
            raise RuntimeError("No serial or SPI port initialized")

    def init_serial(self, baudrate: int = 921600) -> None:
        self._ser = request_interface("uart", "VFD", baudrate)

    def init_spi(self, clock: int = 8_000_000) -> None:
        self._spi = request_interface("spi", "VFD", 2, clock)
        time.sleep(0.1)
        self._spi.byteorder = "lsb"
        print(f"SPI clock: {self._spi.speed_hz} Hz")

    def init_vfd(
        self, dma: bool = False, fsync: bool = False, brightness: int = 255
    ) -> None:
        """
        Do initialization (after power on)

        Args:
            dma (bool): Hardware selected DMA mode (SW8 ON)
            fsync (bool): Use Frame Synconization in DMA mode
            brightness (int): Initial brightness(0-255)
        """
        self._dma_mode = dma
        self._fsync_en = fsync

        self.software_reset()
        time.sleep(RESET_DELAY)
        self.software_reset()
        time.sleep(RESET_DELAY)
        self.set_brightness(brightness)
        time.sleep(CMD_DELAY)
        if self._dma_mode:
            # self.dma_clear_display()
            # time.sleep(CMD_DELAY)
            self.set_display_mode(fsync_mode=self._fsync_en)
        else:
            self.cha_clear_display()
            time.sleep(CMD_DELAY)
            self.set_display_mode()
            time.sleep(CMD_DELAY)
            self.cha_set_quantity(31)
        time.sleep(CMD_DELAY)
        self.set_display_power(True)
        time.sleep(0.5)

    def software_reset(self) -> None:
        """
        Send software reset command to VFD
        """
        self._write(b"\xAA")

    def set_display_power(self, enable: bool) -> None:
        """
        Set display power

        Args:
            enable (bool): True to enable display power, False to disable
        """
        data = b"\xA3\x01" if enable else b"\xA3\x00"
        self._write(data)

    def set_display_mode(
        self,
        negtive_scan: bool = False,
        fsync_mode: bool = False,
        disp_all_on: bool = False,
    ) -> None:
        """
        Set display mode

        Args:
            negtive_scan (bool): True to enable negative scan, False to disable
            fsync_mode (bool): True to enable frame synchronization, False to disable
            disp_all_on (bool): True to enable all display on, False to disable
        """
        opt = 0
        if disp_all_on:
            opt |= 1
        if fsync_mode:
            opt |= 2
        if negtive_scan:
            opt |= 4
        self._write(b"\xA0" + bytes([opt]))

    def set_brightness(self, brightness: int) -> None:
        """
        Set brightness

        Args:
            brightness (int): Brightness value (0-255)
        """
        assert 0 <= brightness <= 255, "Brightness must be in range [0, 255]"
        self._write(b"\xA1" + bytes([brightness]))

    def set_fsync_output(self, enable: bool, low_active: bool = False) -> None:
        """
        Set FSYNC output

        Args:
            enable (bool): True to enable FSYNC output, False to disable
            low_active (bool): True to set FSYNC output low active, False to set high active
        """
        if low_active and enable:
            opt = 1
        elif enable:
            opt = 0
        else:
            opt = 3
        self._write(b"\xA2" + bytes([opt]))

    def dma_clear_display(self) -> None:
        """
        Clear display memory in DMA mode
        """
        assert self._dma_mode, "Only available in DMA mode"
        self._write(b"\x55")

    def dma_sync_refresh(self, page: int) -> None:
        """
        Refresh display in DMA mode with frame synchronization

        Args:
            page (int): Page number (0 - PAGE_NUM-1) or PAGE_NUM for all pages
        """
        assert self._dma_mode, "Only available in DMA mode"
        assert self._fsync_en, "Only available when FSYNC is enabled"
        page = page if 0 <= page <= self.PAGE_NUM - 1 else self.PAGE_NUM
        self._write(b"\xB2" + bytes([page]))

    def dma_write_data(
        self, page: int, column: int, data: Union[List, bytes, bytearray]
    ) -> None:
        """
        Write data to display in DMA mode

        Args:
            page (int): Page number (0 - PAGE_NUM-1)
            column (int): Column number (0-255)
            data (Union[List, bytes, bytearray]): Data to be written

        Note: DMA Memory Rule
            # 0 1 2 3 4 5 6 7 8 9 10 11 ... WIDTH (COLUMN)
            0 D0/LSB
            1 D1  [PAGE 0]
            2 D2
            3 D3  data[0] written to column 0
            4 D4    data[1] written to column 1
            5 D5      ......
            6 D6        When column hit 255(data[255]),
            7 D7/MSB    page will add 1, and column reset to 0
            ---------------------------------------
            .
            .     [PAGE x] (8 pixel height)
            .
            ---------------------------------------
            HEIGHT (PAGE_NUM * 8)
        """
        assert self._dma_mode, "Only available in DMA mode"
        assert 0 <= page <= self.PAGE_NUM - 1, "Page number overflow"
        send_data = bytearray([0xB1, page, column])
        send_data.extend(data)
        self._write(send_data)

    def dma_write_cv2(
        self,
        img: "np.ndarray",
        threshold: int = 127,
        page_write: bool = True,
        page_cnt: int = 1,
    ) -> None:
        import numpy as np

        H, W = img.shape[:2]
        assert (
            W == self.WIDTH and H == self.HEIGHT
        ), f"Image size must be {self.WIDTH}x{self.HEIGHT}"
        assert len(img.shape) == 2, "Image must be grayscale"
        bits = np.packbits(img > threshold, axis=0, bitorder="little")
        byte_data = bits.flatten().tobytes()
        if not page_write:
            self.dma_write_data(0, 0, byte_data)
        else:
            for page in range(ceil(self.PAGE_NUM / page_cnt)):
                self.dma_write_data(
                    page * page_cnt,
                    0,
                    byte_data[
                        page * page_cnt * self.WIDTH : (page * page_cnt + page_cnt)
                        * self.WIDTH
                    ],
                )

    def cha_clear_display(self) -> None:
        """
        Clear display in character mode
        """
        assert not self._dma_mode, "Only available in Character mode"
        self._write(b"\xCC")

    def cha_set_quantity(self, num: int) -> None:
        """
        Set number of components in character mode

        Args:
            num (int): Number of components (1-31)
        """
        assert not self._dma_mode, "Only available in Character mode"
        assert 0 < num < 32, "Number of components must be in range [1, 31]"
        self._write(b"\xCA" + bytes([num]))

    def cha_set_font(
        self,
        id: int,
        font: int,
    ) -> None:
        """
        Set font for a component in character mode

        Args:
            id (int): Component ID (0-31)
            font (int): Font ID (0-7)
        """
        assert not self._dma_mode, "Only available in Character mode"
        assert 0 <= id <= 31, "Component ID must be in range [0, 31]"
        assert 0 <= font <= 7, "Font ID must be in range [0, 7]"
        self._write(b"\xCB" + bytes([(id << 3) | font]))

    def cha_set_pos(self, id: int, x: int, y: int) -> None:
        """
        Set position for a component in character mode

        Args:
            id (int): Component ID (0-31)
            x (int): X position (0-255)
            y (int): Y position (0-63)
        """
        assert not self._dma_mode, "Only available in Character mode"
        assert 0 <= id <= 31, "Component ID must be in range [0, 31]"
        assert 0 <= x <= 255, "X position must be in range [0, 255]"
        assert 0 <= y <= 63, "Y position must be in range [0, 63]"
        self._write(b"\xCD" + bytes([id, x, y]))

    def cha_set_text(self, id: int, text: str) -> None:
        """
        Set text for a component in character mode

        Args:
            id (int): Component ID (0-31)
            text (str): Text to be displayed
        """
        assert not self._dma_mode, "Only available in Character mode"
        assert 0 <= id <= 31, "Component ID must be in range [0, 31]"
        if len(text) == 0:
            text = " "
        self._write(b"\xCE" + bytes([id]) + text.encode("ascii") + b"\x00")

    def cha_terminal_init(self, font: int = FONT_5x10, h_resize: int = 0) -> None:
        """
        Initialize character terminal

        Args:
            font (int): Font ID (0-5)
            h_resize (int): Horizontal resize (plus on font width)
        """
        self._tm_L = (self.HEIGHT) // (self.FONT_SIZE[font][1] + h_resize)
        self._tm_W = (self.WIDTH) // self.FONT_SIZE[font][0]
        self._tm_lines: List[str] = [""]
        self._tm_lines_last = ["" for _ in range(self._tm_L)]
        self.cha_clear_display()
        for i in range(self._tm_L):
            self.cha_set_font(i, font)
            self.cha_set_pos(i, 0, (i + 1) * (self.FONT_SIZE[font][1] + h_resize) - 1)

    def cha_terminal_print(self, text: str) -> None:
        text.replace("\r\n", "\n").replace("\r", "\n")
        if len(self._tm_lines[-1]) != self._tm_W and (
            len(self._tm_lines[-1]) == 0 or self._tm_lines[-1][-1] != "\n"
        ):
            text = self._tm_lines.pop() + text
        while text != "":
            line = text[: self._tm_W]
            text = text[self._tm_W :]
            if "\n" in line:
                idx = line.index("\n")
                text = line[idx + 1 :] + text
                line = line[: idx + 1]
            self._tm_lines.append(line)
        self._tm_lines = self._tm_lines[-self._tm_L :]
        for i, line in enumerate(self._tm_lines):
            if line != self._tm_lines_last[i]:
                self.cha_set_text(i, line.rstrip("\n"))
                self._tm_lines_last[i] = line

    def cha_terminal_println(self, text: str) -> None:
        self.cha_terminal_print(text + "\n")

    def cha_terminal_clear(self) -> None:
        self._tm_lines = [""]
        self._tm_lines_last = ["" for _ in range(self._tm_L)]
        for i in range(self._tm_L):
            self.cha_set_text(i, "")

    def cha_terminal_print_screen(self, text: str) -> None:
        text.replace("\r\n", "\n").replace("\r", "\n")
        for i in range(self._tm_L):
            line = text[: self._tm_W]
            text = text[self._tm_W :]
            if "\n" in line:
                idx = line.index("\n")
                text = line[idx + 1 :] + text
                line = line[:idx]
            self.cha_set_text(i, line)
