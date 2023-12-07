import datetime
import struct
import time
from queue import Queue
from typing import Callable, Literal, Optional

from .base import ZHBaseLayer
from .imu import BNOIMU
from .utils import ByteVar

key_act_dict = {
    0: "null",
    1: "down",
    2: "up",
    3: "short",
    4: "long",
    5: "double",
    6: "double_continue",
    7: "hold",
    8: "hold_continue",
}

key_name_dict = {
    0: "N",
    1: "E",
    2: "S",
    3: "W",
    4: "NE",
    5: "SE",
    6: "SW",
    7: "NW",
    8: "CENTER",
    9: "LT1",
    10: "LT2",
    11: "RT1",
    12: "RT2",
}

gesture_name_dict = {
    0: "up",
    1: "down",
    2: "left",
    3: "right",
    4: "forward",
    5: "backward",
    6: "clockwise",
    7: "anticlockwise",
    8: "wave",
}

NODATA = b"\0"


class ZHProtocolLayer(ZHBaseLayer):
    """
    协议层, 定义了实际的控制命令
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)  # type: ignore
        self.imu = BNOIMU()
        self._data_handler[0x11] = self._update_key_event
        self._key_callback: Optional[Callable[[str, str], None]] = None
        self._key_queue = Queue(16)
        self._gesture_callback: Optional[Callable[[str], None]] = None
        self._gesture_queue = Queue(16)
        self._cursor_callback: Optional[Callable[[int, int], None]] = None
        self._cursor_queue = Queue(16)
        self._temp1 = ByteVar()
        self._temp2 = ByteVar()
        self._temp3 = ByteVar()
        self._temp4 = ByteVar()
        self._uart_temp = b""

    def set_rgb_led(self, r: int, g: int, b: int) -> None:
        """
        设置由板载的RGB LED
        r,g,b: 0-255
        """
        self._temp1.reset(r, "u8", int)
        self._temp2.reset(g, "u8", int)
        self._temp3.reset(b, "u8", int)
        self.send_raw_data(
            0x01, self._temp1.bytes + self._temp2.bytes + self._temp3.bytes
        )

    def _update_imu_data(self, data: bytes):
        self.imu.update(data)
        self._print_state()

    def set_imu(self, enable: bool, report_ms: int = 20) -> None:
        """
        设置IMU传感器
        enable: 是否开启
        report_ms: 报告间隔, 单位ms
        """
        assert 0 < report_ms, "report_ms must be greater than 0"
        self._temp1.reset(report_ms if enable else 0, "u16", int)
        self.send_raw_data(0x02 if enable else 0x03, self._temp1.bytes, ack=True)
        if enable:
            for data in self.imu._get_readble_vars():
                if data not in self._extra_data:
                    self._extra_data.append(data)
            if 0x10 not in self._data_handler:
                self._data_handler[0x10] = self._update_imu_data
            time.sleep(1)
        elif not enable:
            for data in self.imu._get_readble_vars():
                if data in self._extra_data:
                    self._extra_data.remove(data)
            if 0x10 in self._data_handler:
                self._data_handler.pop(0x10)

    def tare_imu(self, persist: bool = False) -> None:
        """
        清零IMU姿态
        persist: 将校准数据写入配置文件
        """
        self._temp1.reset(int(persist), "u8", int)
        self.send_raw_data(0x04, self._temp1.bytes, ack=True)

    def calibrate_imu(self) -> None:
        """
        IMU传感器校准
        """
        self.send_raw_data(0x05, NODATA, ack=True)

    def _update_key_event(self, data: bytes):
        key_name = key_name_dict.get(data[1])
        key_act = key_act_dict.get(data[0])
        if key_name and key_act:
            if self._key_queue.qsize() > 0:
                self._key_queue.put((key_name, key_act))
            if self._key_callback:
                self._key_callback(key_name, key_act)

    def clear_key_event(self) -> None:
        """
        清空按键事件队列
        """
        while self._key_queue.qsize() > 0:
            self._key_queue.get()

    def get_key_event(self) -> Optional[tuple[str, str]]:
        """
        获取按键事件
        return: (key_name, key_act)
        """
        try:
            return self._key_queue.get_nowait()
        except Exception:
            return None

    def set_key(
        self,
        simple_event=True,
        complex_event=True,
        long_ms=300,
        hold_ms=800,
        double_ms=200,
        continue_wait_ms=600,
        continue_send_ms=100,
        continue_send_speedup=1,
        continue_send_min_ms=10,
    ):
        """
        按键扫描参数设置
        """
        data = struct.pack(
            "<BBHHHHHHH",
            int(simple_event),
            int(complex_event),
            long_ms,
            hold_ms,
            double_ms,
            continue_wait_ms,
            continue_send_ms,
            continue_send_speedup,
            continue_send_min_ms,
        )
        self.send_raw_data(0x09, data, ack=True)

    def register_key_callback(self, callback: Callable[[str, str], None]) -> None:
        """
        注册按键回调函数
        """
        self._key_callback = callback

    def _update_gesture_event(self, data: bytes):
        u16 = struct.unpack("<H", data[:2])[0]
        for i in range(9):
            if u16 & (1 << i):
                gesture = gesture_name_dict.get(i)
                if gesture:
                    if self._gesture_queue.qsize() > 0:
                        self._gesture_queue.put(gesture)
                    if self._gesture_callback:
                        self._gesture_callback(gesture)

    def get_gesture_event(self) -> Optional[str]:
        """
        获取手势事件
        return: gesture_name
        """
        try:
            return self._gesture_queue.get_nowait()
        except Exception:
            return None

    def _update_cursor_event(self, data: bytes):
        x_i16, y_i16 = struct.unpack("<hh", data[:4])
        if self._cursor_queue.qsize() > 0:
            self._cursor_queue.put((x_i16, y_i16))
        if self._cursor_callback:
            self._cursor_callback(x_i16, y_i16)

    def register_gesture_callback(self, callback: Callable[[str], None]) -> None:
        """
        注册手势回调函数
        """
        self._gesture_callback = callback

    def get_cursor_event(self) -> Optional[tuple[int, int]]:
        """
        获取光标事件
        return: (x, y)
        """
        try:
            return self._cursor_queue.get_nowait()
        except Exception:
            return None

    def register_cursor_callback(self, callback: Callable[[int, int], None]) -> None:
        """
        注册光标回调函数
        """
        self._cursor_callback = callback

    def set_gesture(
        self, enable: bool, mode: Literal["gesture", "cursor"] = "gesture"
    ) -> None:
        """
        设置手势识别
        enable: 是否开启
        mode: 识别模式
        """
        if enable:
            self._data_handler[0x12] = self._update_gesture_event
            self._data_handler[0x13] = self._update_cursor_event
            if mode == "cursor":
                self.send_raw_data(0x07, NODATA, ack=True)
            else:
                self.send_raw_data(0x06, NODATA, ack=True)
            while self._gesture_queue.qsize() > 0:
                self._gesture_queue.get()
            while self._cursor_queue.qsize() > 0:
                self._cursor_queue.get()
            time.sleep(1)
        else:
            if 0x12 in self._data_handler:
                self._data_handler.pop(0x12)
            if 0x13 in self._data_handler:
                self._data_handler.pop(0x13)
            self.send_raw_data(0x08, NODATA, ack=True)

    def set_uart_baudrate(self, baudrate: int) -> None:
        """
        设置串口波特率
        """
        self._temp1.reset(baudrate, "u32", int)
        self.send_raw_data(0x0A, self._temp1.bytes, ack=True)

    def send_uart(self, data: bytes, split_pack: int = 128) -> None:
        """
        发送串口数据
        """
        if len(data) > split_pack:
            for i in range(0, len(data), split_pack):
                self.send_raw_data(0x0B, data[i : i + split_pack])
        else:
            self.send_raw_data(0x0B, data)

    def register_uart_callback(self, callback: Callable[[bytes], None]) -> None:
        """
        注册串口回调函数, 同时开启串口接收
        """
        self._data_handler[0x14] = callback
        self.send_raw_data(0x0C, b"\x01", ack=True)

    def start_uart(self) -> None:
        """
        开始串口接收
        """
        self.send_raw_data(0x0C, b"\x01", ack=True)
        if 0x14 not in self._data_handler:
            self._data_handler[0x14] = self._default_uart_callback

    def _default_uart_callback(self, data):
        self._uart_temp += data

    def stop_uart(self) -> None:
        """
        停止串口接收
        """
        self.send_raw_data(0x0C, b"\x00", ack=True)

    @property
    def uart_in_waiting(self) -> int:
        """
        串口接收缓冲区数据长度
        """
        return len(self._uart_temp)

    def read_uart(self, read_num: Optional[int] = None, timeout: int = -1) -> bytes:
        """
        轮询读取串口接收数据
        (如果注册了串口回调函数, 此函数无效)
        read_num: 读取数据长度, 默认为返回当前缓冲区所有数据
        timeout: 超时时间, 单位ms, 默认为永不超时
        """
        if read_num is not None:
            start = time.time()
            while time.time() - start < timeout / 1000 or timeout == -1:
                if len(self._uart_temp) >= read_num:
                    data = self._uart_temp[:read_num]
                    self._uart_temp = self._uart_temp[read_num:]
                    return data
                time.sleep(0.01)
        data = self._uart_temp
        self._uart_temp = b""
        return data

    def read_rtc(self) -> datetime.datetime:
        """
        读取RTC时间
        """
        rawdata = self.send_raw_data(0x0D, NODATA, request=True)
        assert isinstance(rawdata, bytes), "RTC Read Error"
        y, m, d, h, mi, s, _ = struct.unpack("<BBBBBBB", rawdata)
        y = y + 2000
        return datetime.datetime(y, m, d, h, mi, s)

    def set_rtc(self, dt: datetime.datetime) -> None:
        """
        设置RTC时间
        """
        y, m, d, h, mi, s = (
            dt.year - 2000,
            dt.month,
            dt.day,
            dt.hour,
            dt.minute,
            dt.second,
        )
        week = dt.weekday() + 1
        data = struct.pack("<BBBBBBB", y, m, d, h, mi, s, week)
        self.send_raw_data(0x0E, data, ack=True)
