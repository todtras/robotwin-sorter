"""
gui/webcam_worker.py — 웹캠 캡처를 위한 별도 스레드

sim_worker.py와 완전히 같은 QThread 패턴을 따릅니다 (start_capture로 스레드 시작,
stop_capture로 멈춤, run() 안에서 cv2로 프레임을 읽어 시그널로 GUI에 전달).
헷갈리면 sim_worker.py의 start_simulation()/run() 주석을 먼저 참고하세요.

시뮬레이션 워커와는 완전히 독립적으로 동작합니다. 대시보드가 열려있는 동안은
계속 촬영하는 게 자연스러우니(시뮬레이션 Start/Stop과 무관), MainWindow가 생성될 때
바로 start_capture()를 호출합니다.

★ SimWorker(use_dummy=False)가 실제 검출을 하려면 이 웹캠 프레임이 필요합니다.
  같은 장치를 두 번 열면(cv2.VideoCapture 중복 open) 충돌하므로, 웹캠은 이 클래스가
  유일하게 열고, raw_frame_ready로 원본 BGR numpy 프레임을 MainWindow를 거쳐
  SimWorker.set_latest_frame()에 직접 넘겨줍니다 (화면 표시용 frame_ready(QImage)와는
  별개 시그널).
"""

from __future__ import annotations

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage
import cv2

import config


class WebcamWorker(QThread):
    """웹캠 프레임을 읽어 GUI로 전달하는 워커 스레드.

    Signals:
        frame_ready(QImage): 화면 표시용으로 변환된 프레임.
        raw_frame_ready(object): 원본 BGR numpy 프레임. SimWorker의 실제 검출용.
        log_message(str): 카메라 연결 실패 등 알림.
    """

    frame_ready = Signal(QImage)
    raw_frame_ready = Signal(object)
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

    def set_camera_index(self, index: int) -> None:
        """카메라 인덱스를 바꿀 때 MainWindow가 호출.

        ★ 스레드가 이미 cv2.VideoCapture를 열어서 돌고 있는 도중에 이 값만 바꾸면
          당장은 반영되지 않습니다(run()이 시작할 때 딱 한 번만 VideoCapture를 엽니다).
          MainWindow 쪽에서
              stop_capture() -> wait() -> set_camera_index(new_index) -> start_capture()
          순서로 호출해서 스레드를 완전히 재시작해야 새 카메라 인덱스로 다시 열립니다.
        """
        self._camera_index = index

    def get_camera_index(self) -> int:
        return self._camera_index

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

                # OpenCV는 BGR 순서로 프레임을 주는데, cv2.cvtColor로 RGB로 바꾸는
                # 대신 QImage.Format_BGR888로 그대로 감싸면 매 프레임 색상 재배열
                # 복사(cvtColor)를 통째로 생략할 수 있음.
                height, width = frame_bgr.shape[:2]
                image = QImage(
                    frame_bgr.data, width, height, 3 * width, QImage.Format.Format_BGR888
                ).copy()  # ★ 스레드 경계 넘기기 전 복제 필수 (sim_worker와 동일 이유)
                self.frame_ready.emit(image)
                self.raw_frame_ready.emit(frame_bgr.copy())

                # cap.read()가 카메라 자체 fps(config.CAMERA_FPS)만큼 이미 블로킹하므로
                # 여기서 추가로 msleep을 하면 체감 fps가 그만큼 더 줄어듦 -> 제거.
        finally:
            cap.release()
