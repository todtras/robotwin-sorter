"""
integration/calibration.py — 픽셀 -> 월드 좌표 변환
담당: 진선우 | Day 2 오후

호모그래피를 쓰는 이유: 카메라를 완벽한 수직으로 고정하기는 불가능한데,
호모그래피는 기울기를 자동 보정합니다. 코드량은 선형 보간과 같습니다.
"""

from __future__ import annotations
import cv2
import numpy as np
import config


class Calibrator:
    """이미지 좌표 <-> 월드 좌표 변환기."""

    def __init__(self,
                 image_points=config.CALIB_IMAGE_POINTS,
                 world_points=config.CALIB_WORLD_POINTS) -> None:
        """TODO(선우) Day 2 오후:
            self.H = cv2.getPerspectiveTransform(
                np.float32(image_points), np.float32(world_points))

        ★ image_points와 world_points의 순서가 1:1 대응해야 합니다.
          하나라도 어긋나면 좌표가 대각선으로 뒤집힙니다.
        """
        self.image_points = image_points
        self.world_points = world_points
        self.H = cv2.getPerspectiveTransform(
            np.float32(image_points), np.float32(world_points))

    def pixel_to_world(self, px: int, py: int) -> tuple[float, float]:
        """픽셀 좌표 -> 월드 좌표 (미터).

        TODO(선우):
            pt = np.float32([[[px, py]]])
            out = cv2.perspectiveTransform(pt, self.H)
            return float(out[0][0][0]), float(out[0][0][1])
        """
        pt = np.float32([[[px, py]]])
        out = cv2.perspectiveTransform(pt, self.H)
        return float(out[0][0][0]), float(out[0][0][1])

    def is_in_workspace(self, x: float, y: float) -> bool:
        """월드 좌표가 로봇 작업영역 안인지 확인.

        TODO(선우):
            return (config.WORKSPACE_X[0] <= x <= config.WORKSPACE_X[1]
                    and config.WORKSPACE_Y[0] <= y <= config.WORKSPACE_Y[1])

        False면 스폰하지 말고 fail_reason="out_of_workspace"로 기록하세요.
        """
        return (config.WORKSPACE_X[0] <= x <= config.WORKSPACE_X[1]
                    and config.WORKSPACE_Y[0] <= y <= config.WORKSPACE_Y[1])

    def measure_error(self, px: int, py: int,
                      true_xy: tuple[float, float]) -> float:
        """실측 위치와의 오차(미터) 반환. 실험 2의 측정 함수입니다.
        목표: config.CALIB_ERROR_TOLERANCE_M(2cm) 이내."""
        wx, wy = self.pixel_to_world(px, py)
        return ((wx - true_xy[0]) ** 2 + (wy - true_xy[1]) ** 2) ** 0.5
