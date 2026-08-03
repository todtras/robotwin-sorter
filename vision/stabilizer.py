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
        """새 좌표를 넣고 평활화된 좌표를 반환."""
        self.buf_x.append(px)
        self.buf_y.append(py)
        return (int(sum(self.buf_x) / len(self.buf_x)),
                int(sum(self.buf_y) / len(self.buf_y)))

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

        True가 나온 물체만 통합 모듈로 넘기세요.
        """
        self.history.append((px, py))
        if len(self.history) < self.history.maxlen:
            return False
        xs = [p[0] for p in self.history]
        ys = [p[1] for p in self.history]
        return (max(xs) - min(xs) < self.tolerance) and (max(ys) - min(ys) < self.tolerance)


class PresenceFilter:
    """카테고리별 검출 여부에 히스테리시스를 적용해 깜빡임을 걸러낸다.

    confidence가 CONF_THRESHOLD 근처에서 프레임마다 오르락내리락하면
    detect()의 결과에 카테고리가 있다/없다가 매 프레임 바뀌는데, 그걸
    그대로 하류(로봇 동작 트리거 등)로 넘기면 오작동 원인이 된다.

    연속 confirm_frames 프레임 동안 검출돼야 "확정"으로 인정하고,
    확정된 뒤에는 연속 release_frames 프레임 동안 검출 안 돼야 "해제"한다.
    """

    def __init__(self, confirm_frames: int = 3, release_frames: int = 3) -> None:
        self.confirm_frames = confirm_frames
        self.release_frames = release_frames
        self._present_streak: dict[str, int] = {}
        self._absent_streak: dict[str, int] = {}
        self._confirmed: set[str] = set()

    def update(self, current_categories: set[str]) -> set[str]:
        """이번 프레임에 검출된 카테고리 집합을 넣고, 확정된 카테고리 집합을 반환."""
        tracked = set(self._present_streak) | set(self._absent_streak) | current_categories
        for cat in tracked:
            if cat in current_categories:
                self._present_streak[cat] = self._present_streak.get(cat, 0) + 1
                self._absent_streak[cat] = 0
                if self._present_streak[cat] >= self.confirm_frames:
                    self._confirmed.add(cat)
            else:
                self._present_streak[cat] = 0
                self._absent_streak[cat] = self._absent_streak.get(cat, 0) + 1
                if self._absent_streak[cat] >= self.release_frames:
                    self._confirmed.discard(cat)
        return set(self._confirmed)
