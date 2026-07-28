"""
vision/detector.py — 커스텀 YOLO 추론
담당: 윤주연 | Day 3 오전

★ 최종 산출물: detect(frame) -> list[Detection]
  통합 모듈은 이 함수 하나만 알면 됩니다.
"""

from __future__ import annotations

import config
from common.schema import CLASS_NAMES, Detection


class TrashDetector:
    """학습된 YOLOv8n으로 PET/CAN/일반을 검출합니다."""

    def __init__(self, model_path: str = str(config.MODEL_PATH),
                 conf_threshold: float = config.CONF_THRESHOLD) -> None:
        """TODO(주연): self.model = YOLO(model_path)"""
        self.model_path = model_path
        self.conf = conf_threshold
        self.model = None

    def detect(self, frame) -> list[Detection]:
        """BGR 프레임 -> Detection 리스트. 검출 없으면 빈 리스트.

        TODO(주연) Day 3 오전:
            results = self.model(frame, imgsz=config.INFERENCE_IMGSZ,
                                 conf=self.conf, verbose=False)
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                Detection(
                    category=CLASS_NAMES[cls_id],   # ★ data.yaml 순서와 일치 필수
                    class_id=cls_id,
                    pixel_x=(x1 + x2) // 2,         # 좌상단이 아니라 중심
                    pixel_y=(y1 + y2) // 2,
                    confidence=float(box.conf[0]),
                    bbox=(x1, y1, x2, y2),
                )

        ★ verbose=False를 빼면 프레임마다 로그가 쏟아져 터미널이 마비됩니다.
        """
        raise NotImplementedError

    def draw(self, frame, detections: list[Detection]):
        """디버깅용 시각화. bbox와 클래스명을 프레임 위에 그립니다.
        데모 영상 촬영(Day 8)에도 씁니다."""
        raise NotImplementedError
