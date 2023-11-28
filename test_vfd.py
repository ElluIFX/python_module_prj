import time

import cv2
import numpy as np

from drivers.interface import register_interface
from drivers.vfd import VFD

register_interface("ch347", "spi", True, cs=1)


class fps_counter:
    def __init__(self, max_sample=60) -> None:
        self.t = time.perf_counter()
        self.max_sample = max_sample
        self.t_list: list[float] = []
        self._fps = 0.0

    def update(self) -> None:
        now = time.perf_counter()
        self.t_list.append(now - self.t)
        self.t = now
        if len(self.t_list) > self.max_sample:
            self.t_list.pop(0)
        length = len(self.t_list)
        sum_t = sum(self.t_list)
        if length == 0:
            fps = 0.0
        else:
            fps = length / sum_t
        self._fps += (fps - self._fps) / 10

    @property
    def fps(self) -> float:
        return self._fps


video = cv2.VideoCapture("badapple.mp4")

disp = VFD()
disp.init_spi()
disp.init_vfd(dma=True)

fpsc = fps_counter()
frame_num = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
frame_cnt = 0
while True:
    # time.sleep(0.01)
    ret, frame = video.read()
    if not ret:
        break
    fpsc.update()
    frame_cnt += 1
    frame = cv2.resize(frame, (128, 64))
    frame = np.concatenate((frame, np.zeros((64, 128, 3), dtype=np.uint8)), axis=1)
    cv2.putText(
        frame,
        "Bad Apple",
        (140, 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        frame,
        f"FPS:{fpsc.fps:.2f}",
        (140, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        frame,
        f"FRM:{frame_cnt}/{frame_num}",
        (140, 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    disp.dma_write_cv2(gray)
    cv2.imshow("frame", gray)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
