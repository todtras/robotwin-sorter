"""
vision/camera.py — 웹캠 입력
담당: 윤주연 | Day 3 오전

★ tools/capture.py(학습 데이터 촬영)와 **완전히 같은 설정**을 써야 합니다.
  그래서 양쪽 모두 config의 상수를 import합니다. 숫자를 직접 쓰지 마세요.
"""

from __future__ import annotations

import cv2

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
        """웹캠을 열고 config에 정의된 해상도/FPS로 설정을 시도한다.

        ★ set()은 요청일 뿐 보장이 아니므로, 적용 결과를 get()으로 확인해
          해상도가 어긋나면(학습 데이터와 불일치) 바로 알 수 있게 한다.
        """
        self.cap = cv2.VideoCapture(self.index)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"웹캠을 열 수 없습니다 (index={self.index}). "
                f"config.CAMERA_INDEX를 0, 1, 2 순으로 바꿔가며 시도해보세요."
            )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_w, actual_h) != (config.FRAME_WIDTH, config.FRAME_HEIGHT):
            print(
                f"[camera] 경고: 요청 해상도 {config.FRAME_WIDTH}x{config.FRAME_HEIGHT} "
                f"미적용, 실제 {actual_w}x{actual_h}. 학습 데이터와 어긋나 "
                f"인식률이 떨어질 수 있습니다."
            )

    def read(self):
        """BGR numpy 배열 (480, 640, 3) 반환. 실패 시 None."""
        if self.cap is None:
            raise RuntimeError("open()을 먼저 호출하세요.")
        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    """실시간 프리뷰: python -m vision.camera 로 실행.
    웹캠 + TrashDetector를 함께 돌려 bbox 오버레이를 눈으로 확인합니다. Q로 종료."""
    from vision.detector import TrashDetector

    detector = TrashDetector()
    with Camera() as cam:
        print(f"[camera] index={cam.index} 실행 중. 창에서 Q를 누르면 종료.")
        while True:
            frame = cam.read()
            if frame is None:
                print("[camera] 프레임을 읽지 못했습니다.")
                break
            detections = detector.detect(frame)
            vis = detector.draw(frame, detections)
            cv2.imshow("camera preview (Q=quit)", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()
