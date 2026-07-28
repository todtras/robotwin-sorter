"""
vision/stabilizer.py — 좌표 안정화
담당: 윤주연 | Day 3 오후

★ 이 모듈이 없으면 데모 중 반드시 오작동합니다.
  사람이 물체를 손에서 놓기 전에 로봇이 출발하기 때문입니다.
"""

from __future__ import annotations

from collections import deque

import config


class MovingAverageFilter:
    """최근 N프레임 좌표의 이동평균. 검출 좌표의 떨림을 줄입니다."""

    def __init__(self, window: int = config.STABILIZE_WINDOW) -> None:
        self.buf_x: deque[int] = deque(maxlen=window)
        self.buf_y: deque[int] = deque(maxlen=window)

    def update(self, px: int, py: int) -> tuple[int, int]:
        """새 좌표를 넣고 평활화된 좌표를 반환.

        TODO(주연):
            self.buf_x.append(px); self.buf_y.append(py)
            return (int(sum(self.buf_x) / len(self.buf_x)),
                    int(sum(self.buf_y) / len(self.buf_y)))
        """
        raise NotImplementedError

    def reset(self) -> None:
        """버퍼를 비웁니다. 새 물체가 등장하면 호출하세요."""
        self.buf_x.clear()
        self.buf_y.clear()


class StabilityChecker:
    """물체가 멈췄는지(= 사람 손이 떠났는지) 판정합니다."""

    def __init__(self,
                 frame_count: int = config.STABLE_FRAME_COUNT,
                 tolerance: int = config.STABLE_PIXEL_TOLERANCE) -> None:
        self.history: deque[tuple[int, int]] = deque(maxlen=frame_count)
        self.tolerance = tolerance

    def is_stable(self, px: int, py: int) -> bool:
        """최근 frame_count 프레임 동안 좌표 변화가 tolerance 미만이면 True.

        TODO(주연) Day 3 오후:
          1. history에 (px, py) 추가
          2. len(history) < maxlen 이면 아직 판단 불가 -> False
          3. history의 x 최대-최소, y 최대-최소가 모두 tolerance 미만이면 True

        True가 나온 물체만 통합 모듈로 넘기세요.
        """
        raise NotImplementedError
