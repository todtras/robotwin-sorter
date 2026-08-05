"""
vision/detector.py — 커스텀 YOLO 추론
담당: 윤주연 | Day 3 오전

★ 최종 산출물: detect(frame) -> list[Detection]
  통합 모듈은 이 함수 하나만 알면 됩니다.
"""

from __future__ import annotations

import cv2
from ultralytics import YOLO

import config
from common.schema import CLASS_NAMES, Detection


class TrashDetector:
    """학습된 YOLOv8n으로 PET/CAN/일반을 검출합니다."""

    def __init__(self, model_path: str = str(config.MODEL_PATH),
                 conf_threshold: float = config.CONF_THRESHOLD) -> None:
        self.model_path = model_path
        self.conf = conf_threshold
        self.model = YOLO(model_path)

    def detect(self, frame) -> list[Detection]:
        """BGR 프레임 -> Detection 리스트. 검출 없으면 빈 리스트."""
        results = self.model(frame, imgsz=config.INFERENCE_IMGSZ,
                              conf=self.conf, iou=0.5, agnostic_nms=True, verbose=False)

        detections: list[Detection] = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append(Detection(
                category=CLASS_NAMES[cls_id],   # ★ data.yaml 순서와 일치 필수
                class_id=cls_id,
                pixel_x=(x1 + x2) // 2,         # 좌상단이 아니라 중심
                pixel_y=(y1 + y2) // 2,
                confidence=float(box.conf[0]),
                bbox=(x1, y1, x2, y2),
            ))
        return detections

    def draw(self, frame, detections: list[Detection]):
        """디버깅용 시각화. bbox와 클래스명을 프레임 위에 그립니다.
        데모 영상 촬영(Day 8)에도 씁니다."""
        vis = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            color = [int(c * 255) for c in config.CATEGORY_COLORS[det.category][2::-1]]
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            label = f"{det.category} {det.confidence:.2f}"
            cv2.putText(vis, label, (x1, max(y1 - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return vis


if __name__ == "__main__":
    """실시간 프리뷰: python -m vision.detector 로 실행.
    웹캠 프레임을 detect()에 넣고 draw()로 bbox 오버레이를 확인합니다. Q로 종료.
    (프레임별 원본 검출을 그대로 보여줌 — confidence 근처 깜빡임 debounce는
    안 함. 실제 시스템의 안정화는 vision/stabilizer.py에서 하며, 이 프리뷰는
    detector.detect() 자체의 raw 출력을 확인하는 용도다.)
    """
    from vision.camera import Camera

    detector = TrashDetector()

    with Camera() as cam:
        print(f"[detector] model={detector.model_path} 실행 중. 창에서 Q를 누르면 종료.")
        while True:
            frame = cam.read()
            if frame is None:
                print("[detector] 프레임을 읽지 못했습니다.")
                break
            detections = detector.detect(frame)
            vis = detector.draw(frame, detections)
            cv2.imshow("detector preview (Q=quit)", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()
