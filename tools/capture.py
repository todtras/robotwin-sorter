"""
tools/capture.py — 학습 데이터 수집
담당: 윤주연 | Day 1 오전

실행: python -m tools.capture
스페이스바를 누를 때마다 프레임 저장. Q로 종료.
물체 배치를 바꾸고 스페이스를 반복하면 한 시간에 200~300장 모입니다.

★ 촬영 전 체크리스트 (기획서 3장)
  1. 웹캠이 책상 위 수직으로 **테이프 고정** — 움직이면 전부 다시 찍어야 함
  2. 작업 영역에 **어두운 배경** — 선택이 아니라 필수.
     투명 페트병은 밝은 책상에서 경계가 안 잡히고,
     알루미늄 캔은 반사광 때문에 형태가 뭉개집니다
  3. 네 귀퉁이에 마커(검은 테이프/포스트잇) — 캘리브레이션 대응점
  4. 조명 고정 (형광등 켠 상태로 통일)

★ 촬영 규칙
  - 클래스별 100장 이상, 합계 300장 이상
  - 개체 다양성 > 장수. 같은 병 100장보다 5종 20장씩이 훨씬 낫습니다
  - 작업 영역 전체에 골고루 배치 (한쪽에만 놓으면 반대편에서 인식 실패)
  - 여러 방향으로 회전시켜 촬영 (탑다운 뷰라 회전 다양성이 중요)
  - 30% 정도는 물체 2~3개를 함께, 일부는 살짝 겹치게
  - 찌그러진 캔, 구겨진 봉지 등 변형 상태도 포함
"""

from __future__ import annotations

import time

import cv2

import config
from vision.camera import Camera

RAW_DIR = config.DATASET_DIR / "raw"


def main() -> None:
    """스페이스바로 프레임을 한 장씩 저장. Q로 종료.

    ★ Camera()가 config의 해상도/FPS를 그대로 적용하고 불일치 시 경고를
      찍어주므로, 여기서 별도로 해상도를 확인할 필요는 없다(vision/camera.py 참고).
    """
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    saved = 0
    with Camera() as cam:
        print(f"[capture] 저장 위치: {RAW_DIR}")
        print("[capture] SPACE=저장, Q=종료")
        while True:
            frame = cam.read()
            if frame is None:
                print("[capture] 프레임을 읽지 못했습니다.")
                break
            cv2.imshow("capture (SPACE=save, Q=quit)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                path = RAW_DIR / f"img_{int(time.time() * 1000)}.jpg"
                cv2.imwrite(str(path), frame)
                saved += 1
                print(f"[capture] 저장: {path.name} (총 {saved}장)")
            elif key == ord('q'):
                break
    cv2.destroyAllWindows()
    print(f"[capture] 종료. 총 {saved}장 저장됨.")


if __name__ == "__main__":
    main()
