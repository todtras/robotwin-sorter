"""
tests/dummy_vision.py — 가짜 비전 모듈
담당: 진선우 | ★ Day 2 오전 최우선

이게 있어야 태익/선우가 주연의 모델 학습을 기다리지 않고 개발할 수 있습니다.
Day 2 오전에 이것부터 만드세요.
"""

from __future__ import annotations

import random

import config
from common.schema import CATEGORY_TO_BIN, CLASS_NAMES, Detection, SortTask


def make_dummy_detection(category: str | None = None) -> Detection:
    """무작위 Detection 하나 생성. category를 주면 그 클래스로 고정."""
    if category is None:
        category = random.choice(CLASS_NAMES)
    return Detection(
        category=category,
        class_id=CLASS_NAMES.index(category),
        pixel_x=random.randint(150, 500),
        pixel_y=random.randint(120, 380),
        confidence=random.uniform(0.6, 0.95),
        bbox=(0, 0, 0, 0),
    )


def make_dummy_task(category: str | None = None) -> SortTask:
    """로봇 단독 테스트용 SortTask.

    태익 사용 예 (Day 2~3)::

        from tests.dummy_vision import make_dummy_task
        ok = controller.execute_task(make_dummy_task("pet"))

    ★ body_id=0은 실제 PyBullet 객체가 아닙니다.
      grasp()를 실제로 테스트하려면 p.createMultiBody()로 박스를 하나
      띄우고 그 id를 넣으세요.
    """
    det = make_dummy_detection(category)
    x = random.uniform(*config.WORKSPACE_X)
    y = random.uniform(*config.WORKSPACE_Y)
    return SortTask(
        body_id=0,
        target_xyz=(x, y, config.WORKSPACE_Z),
        target_bin=CATEGORY_TO_BIN[det.category],
        category=det.category,
        source=det,
    )


class DummyDetector:
    """TrashDetector와 같은 인터페이스. 무작위 결과를 뱉습니다."""

    def __init__(self, detect_probability: float = 0.3) -> None:
        self.p = detect_probability

    def detect(self, frame=None) -> list[Detection]:
        if random.random() > self.p:
            return []
        return [make_dummy_detection()]
