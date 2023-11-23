import threading
import time
from threading import Event
from typing import Callable, Dict, List, Optional, Union

from loguru import logger
from .serial import SerialReaderBuffered, SerialReaderLike
from .smbus import I2C
from .utils import BaseEvent, BaseStruct, ByteVar

from drivers.interface import InterfaceManager


class ZHSettingStruct:
    ack_timeout = 0.1  # 应答帧超时时间
    send_timeout = 1  # 发送等待超时时间
    request_timeout = 0.5  # 查询等待超时时间
    ack_max_retry = 3  # 应答失败最大重发次数
    raise_if_no_ack = True  # 当ACK帧校验失败时抛出异常
    raise_if_send_timeout = True  # 当发送等待超时时抛出异常
    raise_if_request_timeout = True  # 当查询等待超时时抛出异常


class ZHStateStruct(BaseStruct):
    bat_volt = ByteVar("u16", float, 0.001)  # 电池电压 (V)
    charge = ByteVar("u8", bool)  # 充电状态
    chr_cur = ByteVar("u16", float, 0.001)  # 充电电流 (A)
    otg = ByteVar("u8", bool)  # OTG状态
    slider = ByteVar("u16", float, 0.001)  # 滑杆位置 (0~1)
    imu = ByteVar("u8", bool)  # IMU传感器状态
    gesture = ByteVar("u8", bool)  # 手势传感器状态

    PACK = [bat_volt, charge, chr_cur, otg, slider, imu, gesture]


class ZHEventStruct:
    test = BaseEvent()

    EVENT_CODE = {
        0x01: test,
    }



class ZHBaseLayer(object):
    """
    通讯层, 实现通讯功能
    """

    def __init__(self) -> None:
        super().__init__()
        self.running = False
        self.connected = False
        self._thread_list: List[threading.Thread] = []
        self._state_update_callback: Optional[Callable[[ZHStateStruct], None]] = None
        self._print_state_flag = False
        self._reader: Optional[SerialReaderLike] = None
        self._i2c: Optional[I2C] = None
        self._send_lock = threading.Lock()
        self._recivied_ack_dict: Dict[int, Event] = {}
        self._data_handler: Dict[int, Callable[[bytes], None]] = {}
        self._extra_data: List[ByteVar] = []
        self._request_data: Optional[bytes] = None
        self._i2c_mode = False
        self.state = ZHStateStruct()
        self.event = ZHEventStruct()
        self.settings = ZHSettingStruct()

    def start_listen_i2c(
        self,
        addr: int = 0x42,
        print_state=False,
        use_int: bool = False,
        callback: Optional[Callable[[ZHStateStruct], None]] = None,
    ):
        """通过I2C连接设备

        Args:
            i2c_num (int, optional): I2C总线号
            addr (int, optional): I2C设备地址
            print_state (bool, optional): 是否在终端打印状态
            callback (Callable[[ZH_State_Struct], None], optional): 状态更新回调函数
        """
        assert not self.running, "ZH is already running"
        self._state_update_callback = callback
        self._print_state_flag = print_state
        self._i2c_mode = True
        if use_int:
            self._intp = InterfaceManager.request_gpio_interface("Zero-Hat")
            self._intp.set_mode("INT", "input_no_pull")
        else:
            self._intp = None
        self._i2c = I2C(addr)
        self.running = True
        self.send_raw_data(b"\x01", 0x00)  # 先发个包唤醒设备
        time.sleep(0.5)
        _listen_thread = threading.Thread(target=self._listen_i2c_task)
        _listen_thread.daemon = True
        _listen_thread.start()
        self._thread_list.append(_listen_thread)

    def start_listen_serial(
        self,
        baudrate: int = 921600,
        print_state=False,
        callback: Optional[Callable[[ZHStateStruct], None]] = None,
    ):
        """通过串口连接设备

        Args:
            serial_dev (str, optional): 串口设备路径, 默认自动搜索
            baudrate (int, optional): 波特率
            print_state (bool, optional): 是否在终端打印状态
            callback (Callable[[ZH_State_Struct], None], optional): 状态更新回调函数
        """
        assert not self.running, "ZH is already running"
        self._state_update_callback = callback
        self._print_state_flag = print_state
        self._ser = InterfaceManager.request_uart_interface("Zero-Hat", baudrate)
        self._reader = SerialReaderBuffered(self._ser, [0xAA, 0x55])
        self.running = True
        _listen_thread = threading.Thread(target=self._listen_serial_task)
        _listen_thread.daemon = True
        _listen_thread.start()
        self._thread_list.append(_listen_thread)

    def close(self, joined=True) -> None:
        self.running = False
        self.connected = False
        if joined:
            for thread in self._thread_list:
                thread.join()
                self._thread_list.remove(thread)
        if self._reader:
            self._reader.close()
        if self._ser:
            self._ser.close()
        logger.info("[ZH] Threads closed, ZH offline")

    def _get_request(self):
        req_time = time.perf_counter()
        while self._request_data is None:
            if time.perf_counter() - req_time > self.settings.request_timeout:
                logger.error("[ZH] Wait request timeout")
                if self.settings.raise_if_request_timeout:
                    raise Exception("Wait request timeout")
                return None
            time.sleep(0.001)
        return self._request_data

    def _send_i2c(
        self,
        data: bytes,
        option: int,
        no_lock: bool = False,
        request: bool = False,
    ) -> Union[bool, None, bytes]:
        assert self._i2c is not None, "I2C is not initialized"
        send_data = bytes([option & 0x7F]) + data
        if request:
            self._request_data = None
        if not no_lock:
            if not self._send_lock.acquire(timeout=self.settings.send_timeout):
                logger.error("[ZH] Wait sending data timeout")
                if self.settings.raise_if_send_timeout:
                    raise Exception("Wait sending data timeout")
                return False
            try:
                self._i2c.write(send_data)
            finally:
                self._send_lock.release()
        else:
            self._i2c.write(send_data)
        if request:
            return self._get_request()
        return True

    def send_raw_data(
        self,
        data: bytes,
        option: int,
        ack: bool = False,
        no_lock: bool = False,
        request: bool = False,
        _ack_retry_count: int = None,  # type: ignore
    ) -> Union[bool, None, bytes]:
        """将数据向设备发送
        Args:
            data (bytes): bytes类型的数据
            option (int): 选项, 对应飞控代码
            ack (bool, optional): 是否需要应答验证
            no_lock (bool, optional): 是否不加锁发送
            request (bool, optional): 是否为查询请求
            _ack_retry_count (int, optional): 应答超时时最大重发次数, 此处由函数自动递归设置, 请修改settings中的选项.

        Returns:
            bool: 是否成功发送 / bytes: 查询结果 / None: 查询超时
        """
        assert self.running, "Device is closed"
        if self._i2c_mode:
            return self._send_i2c(data, option, no_lock, request)
        check_ack = None
        send_option = option & 0x7F
        if request and _ack_retry_count is None:
            self._request_data = None
        if ack:
            if _ack_retry_count is None:
                _ack_retry_count = self.settings.ack_max_retry
            check_ack = send_option
            for add_bit in data:
                check_ack = (check_ack + add_bit) & 0xFF
            self._recivied_ack_dict[check_ack] = Event()
            if _ack_retry_count <= 0:
                logger.error("Wait ACK reached max retry")
                if self.settings.raise_if_no_ack:
                    raise Exception("Wait ACK reached max retry")
                return False
            send_option |= 0x80  # 设置ACK位
        if isinstance(data, list):
            data = bytes(data)
        if not isinstance(data, bytes):
            raise TypeError("data must be bytes")
        send_data = (
            b"\xaa\x22"
            + send_option.to_bytes(1, "little")
            + len(data).to_bytes(1, "little")
            + data
        )
        send_data += (sum(send_data) & 0xFF).to_bytes(1, "little")  # checksum
        if not no_lock:
            if not self._send_lock.acquire(timeout=self.settings.send_timeout):
                logger.error("[ZH] Wait sending data timeout")
                if self.settings.raise_if_send_timeout:
                    raise Exception("Wait sending data timeout")
                return False
            try:
                self._ser.write(send_data)
            finally:
                self._send_lock.release()
        else:
            self._ser.write(send_data)
        if ack and check_ack in self._recivied_ack_dict:
            if not self._recivied_ack_dict[check_ack].wait(self.settings.ack_timeout):
                self._recivied_ack_dict.pop(check_ack)
                logger.warning(f"[ZH] ACK timeout, retry - {_ack_retry_count}")
                return self.send_raw_data(
                    data, option, ack, no_lock, request, _ack_retry_count - 1
                )
            self._recivied_ack_dict.pop(check_ack)
        if request:
            return self._get_request()
        return True

    def _listen_i2c_task(self):
        assert self._i2c is not None, "I2C is not initialized"
        logger.info("[ZH] Listen I2C thread started")
        heartbeat_t = time.perf_counter()
        conn_t = time.perf_counter()
        while self.running:
            try:
                if self._intp is not None:
                    while self._intp.read("INT"):
                        time.sleep(0.001)
                for data in self._i2c.read():
                    if self._update_data(data):
                        conn_t = time.perf_counter()
                        if not self.connected:
                            self.connected = True
                            logger.info("[ZH] Connected")
                if time.perf_counter() - heartbeat_t > 0.25:  # 心跳包
                    self.send_raw_data(b"\x01", 0x00, no_lock=True)
                    heartbeat_t = time.perf_counter()
                if self.connected and time.perf_counter() - conn_t > 0.5:  # 断连检测
                    self.connected = False
                    logger.warning("[ZH] Disconnected")
                # time.sleep(0.001)
            except Exception:
                logger.exception("[ZH] Listen I2C exception")
        logger.info("[ZH] Listen I2C thread closed")

    def _listen_serial_task(self):
        assert self._reader is not None, "SerialReader is not initialized"
        logger.info("[ZH] Listen serial thread started")
        heartbeat_t = time.perf_counter()
        conn_t = time.perf_counter()
        while self.running:
            try:
                while self._reader.read():
                    if self._update_data(self._reader.data):
                        conn_t = time.perf_counter()
                        if not self.connected:
                            self.connected = True
                            logger.info("[ZH] Connected")
                if time.perf_counter() - heartbeat_t > 0.25:  # 心跳包
                    self.send_raw_data(b"\x01", 0x00)
                    heartbeat_t = time.perf_counter()
                if self.connected and time.perf_counter() - conn_t > 0.5:  # 断连检测
                    self.connected = False
                    logger.warning("[ZH] Disconnected")
                time.sleep(0.001)  # 降低CPU占用
            except Exception:
                logger.exception("[ZH] Listen serial exception")
        logger.info("[ZH] Listen serial thread closed")

    def _update_data(self, _data: bytes) -> bool:
        cmd = _data[0]
        data = _data[1:]
        if cmd == 0x01:  # ACK返回
            if (event := self._recivied_ack_dict.get(data[0])) is None:
                logger.warning(f"[ZH] Unrecognized ACK: {data[0]}")
            else:
                event.set()
        elif cmd == 0x02:  # 事件通讯
            event_code = data[0]
            event_operator = data[1]
            if event_operator == 0x01:  # set
                self.event.EVENT_CODE[event_code].set()
                logger.debug(f"[ZH] Event {self.event.EVENT_CODE[event_code].name} set")
            elif event_operator == 0x02:  # clear
                self.event.EVENT_CODE[event_code].clear()
                logger.debug(
                    f"[ZH] Event {self.event.EVENT_CODE[event_code].name} clear"
                )
        elif cmd == 0x03:  # 调试信息
            logger.debug(f"[ZH] Message: {data.decode('utf-8', 'ignore')}")
        elif cmd == 0x04:  # 状态回传
            self.state.update_from_bytes(data)
            if callable(self._state_update_callback):
                self._state_update_callback(self.state)
            self._print_state()
        elif cmd == 0x05:  # 查询结果回传
            self._request_data = data
        elif cmd in self._data_handler:  # 自定义通讯
            self._data_handler[cmd](data)
        else:
            return False
        return True

    def _print_state(self):
        if self._print_state_flag:
            self.state.print(extra_data=self._extra_data)
