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

    ★ 테스트용 임시 히스테리시스: confidence가 CONF_THRESHOLD 근처에서
      깜빡이는 걸 눈으로 덜 거슬리게 보려고 카테고리 단위로 연속 프레임
      조건을 걸어둔 것. 실제 시스템 반영은 vision/stabilizer.py에서 한다.
    """
    from vision.camera import Camera

    CONFIRM_FRAMES = 3   # 연속 이만큼 검출돼야 "확정"
    RELEASE_FRAMES = 3   # 연속 이만큼 안 잡혀야 "해제"

    detector = TrashDetector()
    present_streak: dict[str, int] = {}
    absent_streak: dict[str, int] = {}
    confirmed: set[str] = set()

    with Camera() as cam:
        print(f"[detector] model={detector.model_path} 실행 중. 창에서 Q를 누르면 종료.")
        while True:
            frame = cam.read()
            if frame is None:
                print("[detector] 프레임을 읽지 못했습니다.")
                break
            detections = detector.detect(frame)
            current_categories = {d.category for d in detections}
            tracked_categories = set(present_streak) | set(absent_streak) | current_categories

            for cat in tracked_categories:
                if cat in current_categories:
                    present_streak[cat] = present_streak.get(cat, 0) + 1
                    absent_streak[cat] = 0
                    if cat not in confirmed and present_streak[cat] >= CONFIRM_FRAMES:
                        confirmed.add(cat)
                        det = next(d for d in detections if d.category == cat)
                        print(f"[detector] 폐기물 검출: {cat} (conf={det.confidence:.2f}, "
                              f"bbox={det.bbox}, frame_shape={frame.shape})")
                else:
                    present_streak[cat] = 0
                    absent_streak[cat] = absent_streak.get(cat, 0) + 1
                    if cat in confirmed and absent_streak[cat] >= RELEASE_FRAMES:
                        confirmed.discard(cat)

            display_detections = [d for d in detections if d.category in confirmed]
            vis = detector.draw(frame, display_detections)
            cv2.imshow("detector preview (Q=quit)", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()
