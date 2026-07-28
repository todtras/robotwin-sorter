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

import config

RAW_DIR = config.DATASET_DIR / "raw"


def main() -> None:
    """TODO(주연) Day 1 오전:

        RAW_DIR.mkdir(parents=True, exist_ok=True)
        cap = cv2.VideoCapture(config.CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)

        while True:
            ret, frame = cap.read()
            if not ret: break
            cv2.imshow("capture (SPACE=save, Q=quit)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                path = RAW_DIR / f"img_{int(time.time()*1000)}.jpg"
                cv2.imwrite(str(path), frame)
            elif key == ord('q'):
                break

        cap.release(); cv2.destroyAllWindows()

    ★ 해상도가 config 값과 실제로 같은지 첫 프레임에서 확인하고 출력하세요.
      학습 데이터와 실시간 추론의 해상도가 다르면 인식률이 크게 떨어집니다.
    """
    raise NotImplementedError


if __name__ == "__main__":
    main()
