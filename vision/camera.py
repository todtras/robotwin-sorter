"""
vision/camera.py — 웹캠 입력
담당: 윤주연 | Day 3 오전

★ tools/capture.py(학습 데이터 촬영)와 **완전히 같은 설정**을 써야 합니다.
  그래서 양쪽 모두 config의 상수를 import합니다. 숫자를 직접 쓰지 마세요.
"""

from __future__ import annotations

import config


class Camera:
    """웹캠 래퍼.

    사용 예::

        with Camera() as cam:
            frame = cam.read()
    """

    def __init__(self, index: int = config.CAMERA_INDEX) -> None:
        self.index = index
        self.cap = None

    def open(self) -> None:
        """TODO(주연):
            self.cap = cv2.VideoCapture(self.index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

        ★ set()은 요청일 뿐 보장이 아닙니다. 실제로 적용됐는지
          cap.get(cv2.CAP_PROP_FRAME_WIDTH)로 반드시 확인하세요.
          640이 아니면 학습 데이터와 어긋나 인식률이 떨어집니다.
        """
        raise NotImplementedError

    def read(self):
        """BGR numpy 배열 (480, 640, 3) 반환. 실패 시 None."""
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
