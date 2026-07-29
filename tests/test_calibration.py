"""
tests/test_calibration.py — 좌표 변환 검증
담당: 진선우 | Day 2 오후

실행: python -m tests.test_calibration
실험 2(좌표 변환 정확도)의 측정 스크립트로 그대로 확장하세요.
"""

from __future__ import annotations
from integration.calibration import Calibrator
import config
import cv2
import numpy as np

def test_corners_map_correctly() -> None:
    """캘리브레이션에 쓴 네 귀퉁이는 대응 월드 좌표로 정확히 변환돼야 합니다.

    TODO(선우): Calibrator를 만들고 CALIB_IMAGE_POINTS 각 점을
    pixel_to_world에 넣어 CALIB_WORLD_POINTS와 비교. 오차 1mm 이내.

    여기서 틀리면 호모그래피 대응점 순서가 어긋난 것입니다.
    """
    cal = Calibrator()
    for img_pt, world_pt in zip(config.CALIB_IMAGE_POINTS, config.CALIB_WORLD_POINTS):
      wx, wy = cal.pixel_to_world(img_pt[0], img_pt[1])
      error = ((wx - world_pt[0]) ** 2 + (wy - world_pt[1]) ** 2) ** 0.5
      assert error < 0.001, f"오차 {error:.4f}m at {img_pt}"
      print(f"  {img_pt} -> ({wx:.4f}, {wy:.4f}) OK")


def test_workspace_bounds() -> None:
    """작업영역 안팎 판정이 맞는지 확인.

    TODO(선우):
      - 중앙점 (0.5, 0.0) -> True
      - 영역 밖 (1.0, 0.0) -> False
    """
    cal = Calibrator()
    assert cal.is_in_workspace(0.5, 0.0) == True
    assert cal.is_in_workspace(1.0, 0.0) == False
    assert cal.is_in_workspace(0.35, -0.25) == True
    assert cal.is_in_workspace(0.34, 0.0) == False
    print("  작업영역 판정 OK")


def measure_grid_error() -> None:
    """★ 실험 2 본체: 3x3 격자 9지점 x 5회 = 45회 측정.

    TODO(선우) Day 7:
      1. 책상에 자로 재서 9지점 표시
      2. 각 지점에 물체를 놓고 검출 좌표를 pixel_to_world에 통과
      3. 실측 좌표와의 오차(mm)를 CSV로 기록
      4. 지점별 오차 히트맵 + 평균/최대 오차 -> 보고서 그림

    비교축: 선형 보간 vs 호모그래피
    목표: 평균 오차 2cm 이내
    """
    raise NotImplementedError


if __name__ == "__main__":
    test_corners_map_correctly()
    test_workspace_bounds()
    print("모든 검증 통과")
