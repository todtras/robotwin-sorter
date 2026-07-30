"""
gui/webcam_worker.py — 웹캠 캡처를 위한 별도 스레드

sim_worker.py와 완전히 같은 QThread 패턴을 따릅니다 (start_capture로 스레드 시작,
stop_capture로 멈춤, run() 안에서 cv2로 프레임을 읽어 시그널로 GUI에 전달).
헷갈리면 sim_worker.py의 start_simulation()/run() 주석을 먼저 참고하세요.

★ vision/camera.py의 Camera는 아직 구현 전(raise NotImplementedError 상태)이라,
  팀원 모듈에 의존하지 않기 위해 여기서는 cv2.VideoCapture를 직접 다룹니다.
  Camera가 나중에 완성되면 이 파일의 run()만 그쪽 호출로 바꿔 끼우면 됩니다.

시뮬레이션 워커와는 완전히 독립적으로 동작해야 합니다. 대시보드가 열려있는 동안은
계속 촬영하는 게 자연스러우니(시뮬레이션 Start/Stop과 무관), MainWindow가 생성될 때
바로 start_capture()를 호출하도록 설계하세요.
"""

from __future__ import annotations

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage
import cv2

import config


class WebcamWorker(QThread):
    """웹캠 프레임을 읽어 GUI로 전달하는 워커 스레드.

    Signals:
        frame_ready(QImage): 촬영된 프레임.
        log_message(str): 카메라 연결 실패 등 알림.
    """

    frame_ready = Signal(QImage)
    log_message = Signal(str)

    def __init__(self, camera_index: int = config.CAMERA_INDEX) -> None:
        super().__init__()
        self._camera_index = camera_index
        self._running = False

    def start_capture(self) -> None:
        self._running = True
        if not self.isRunning():
            self.start()

    def stop_capture(self) -> None:
        self._running = False
        
    def run(self) -> None:
        cap = cv2.VideoCapture(self._camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)

        if not cap.isOpened():
            self.log_message.emit(f"Failed to open camera index {self._camera_index}")
            return

        try:
            while self._running:
                ok, frame_bgr = cap.read()
                if not ok:
                    self.msleep(100)
                    continue

                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                image = QImage(frame_rgb.data, frame_rgb.shape[1], frame_rgb.shape[0], QImage.Format_RGB888).copy()
                self.frame_ready.emit(image)

                self.msleep(33)  # 대략 30fps 유지
        finally:
            cap.release()

        


