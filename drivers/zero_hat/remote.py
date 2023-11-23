import socket
import time
from multiprocessing.connection import Client, Listener
from threading import Thread
from typing import Callable, Optional, Tuple

from loguru import logger

from .application import ZHAppLayer
from .base import ZHStateStruct


def get_ip():
    try:
        return socket.gethostbyname(socket.gethostname())
    except Exception:
        return "127.0.0.1"


filter_id_dict = {
    "ack": 0x01,
    "event": 0x02,
    "message": 0x03,
    "state": 0x04,
    "request": 0x05,
}

default_filters = ("ack",)  # 默认数据过滤器


class ZeroHatServer(ZHAppLayer):
    """
    服务器, 负责转发指令到本地
    """

    def __init__(self, *args, **kwargs) -> None:
        self._conn = None
        self._remote_filter = tuple((filter_id_dict[t] for t in default_filters))
        super().__init__(*args, **kwargs)

    def _update_data(self, data: bytes):
        if self._conn and data[0] not in self._remote_filter:
            self._conn.send_bytes(data)
        return super()._update_data(data)

    def _server_task(self, port, authkey):
        addr = ("0.0.0.0", port)
        address = get_ip() + ":" + str(port)
        local_address = f"127.0.0.1:{port}"
        logger.info(f"[ZH_Server] Serving on {address}, {local_address}")
        while self.running:
            try:
                self._remote_filter = tuple(
                    (filter_id_dict[t] for t in default_filters)
                )
                with Listener(addr, authkey=authkey) as listener:
                    try:
                        with listener.accept() as self._conn:
                            logger.info(
                                f"[ZH_Server] Connection accepted from {listener.last_accepted}"
                            )
                            try:
                                filters = self._conn.recv()
                                self._remote_filter = tuple(
                                    (filter_id_dict[t] for t in filters)
                                )
                                logger.info(f"[ZH_Server] Set filters to {filters}")
                            except Exception:
                                self._remote_filter = tuple(
                                    (filter_id_dict[t] for t in default_filters)
                                )
                                logger.warning(
                                    "[ZH_Server] No valid filters received, use default"
                                )
                            while self.running:
                                if self._conn.poll(1):
                                    args = self._conn.recv()
                                    try:
                                        self.send_raw_data(*args)
                                    except Exception as e:
                                        logger.error(
                                            f"[ZH_Server] Handle data error: {e}"
                                        )
                    except Exception as e:
                        self._conn = None
                        if str(e):
                            logger.warning(f"[ZH_Server] Lost connection: {e}")
                        else:
                            logger.info("[ZH_Server] Connection closed by client")
            except Exception:
                self._conn = None
                logger.exception("[ZH_Server] Listener error, restarting")
                time.sleep(1)

    def serve_forever(self, port=5654, authkey=b"fc"):
        """
        启动服务器(永久阻塞)
        port:服务器端口
        authkey:认证密钥
        """
        if not self.running:
            raise Exception("Serial listening must be started before init server")
        self._server_task(port, authkey)

    def start_server(self, port=5654, authkey=b"fc"):
        """
        启动服务器(非阻塞)
        port:服务器端口
        authkey:认证密钥
        """
        if not self.running:
            raise Exception("Serial listening must be started before init server")
        self._listen_thread = Thread(target=self._server_task, args=(port, authkey))
        self._listen_thread.daemon = True
        self._listen_thread.start()

    def close(self, joined=True) -> None:
        self.running = False
        self._listen_thread.join()
        return super().close(joined)


class ZeroHatClient(ZHAppLayer):
    """
    客户端, 与服务器通信
    """

    def __init__(self, *args, **kwargs) -> None:
        self._conn = None
        super().__init__(*args, **kwargs)

    def start_listen_serial(self, *args, **kwargs):
        raise Exception("Client does not need to listen serial, use connect() instead")

    def start_listen_i2c(self, *args, **kwargs):
        raise Exception("Client does not need to listen i2c, use connect() instead")

    def _client_task(self, addr, authkey):
        while self.running:
            try:
                with Client(addr, authkey=authkey) as self._conn:
                    logger.info("[ZH_Client] Connected to server")
                    self._conn.send(self._remote_filters)
                    self.connected = True
                    while self.running:
                        if self._conn.poll(1):
                            data = self._conn.recv_bytes()
                            self._update_data(data)
            except Exception:
                logger.exception("[ZH_Client] Listen error")
                logger.warning("[ZH_Client] Connection lost, try reconnecting")
                self.connected = False
                self._conn = None
                time.sleep(1)
        logger.info("[ZH_Client] Listen thread stopped")

    def connect(
        self,
        host="127.0.0.1",
        port=5654,
        authkey=b"fc",
        print_state: bool = False,
        callback: Optional[Callable[[ZHStateStruct], None]] = None,
        filters: Tuple[str, ...] = default_filters,
        block: bool = True,
        timeout: float = -1,
    ):
        """
        连接服务器
        host:服务器地址
        port:服务器端口
        authkey:认证密钥
        print_state:是否打印状态
        callback:状态更新回调函数
        filters:设置远程数据过滤器
        block:是否阻塞直至连接成功
        timeout:阻塞超时时间, 超时则抛出异常
        """
        self.running = True
        addr = (host, port)
        self._state_update_callback = callback
        self._print_state_flag = print_state
        self._remote_filters = filters
        self._listen_thread = Thread(target=self._client_task, args=(addr, authkey))
        self._listen_thread.daemon = True
        self._listen_thread.start()
        logger.info(f"[ZH_Client] Connecting to {addr}")
        if block:
            t0 = time.perf_counter()
            while not self.connected:
                time.sleep(0.1)
                if timeout > 0 and time.perf_counter() - t0 > timeout:
                    self.close()
                    raise Exception("Connection timeout")

    def close(self, joined=True) -> None:
        self.running = False
        self._listen_thread.join()
        return super().close(joined)

    def send_raw_data(
        self,
        data: bytes,
        option: int,
        ack: bool = False,
        no_lock: bool = False,
        request: bool = False,
    ):
        if self._conn is None:
            raise Exception("Remote not connected")
        if request:
            self._request_data = None
        self._conn.send((data, option, ack, no_lock, request))
        if request:
            return self._get_request()
