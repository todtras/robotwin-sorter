"""
tests/dummy_robot.py — 가짜 로봇 모듈
담당: 진선우 | Day 2 오전

선우가 로봇 구현을 기다리지 않고 파이프라인을 완성하기 위한 스텁입니다.
"""

from __future__ import annotations

import random
import time

from common.schema import SortTask


class DummyArmController:
    """ArmController와 같은 인터페이스. 1초 자고 True를 반환합니다."""

    def __init__(self, success_rate: float = 1.0, delay_sec: float = 1.0) -> None:
        self.success_rate = success_rate
        self.delay_sec = delay_sec

    def execute_task(self, task: SortTask) -> bool:
        """항상(또는 success_rate 확률로) 성공. 실제 동작 시간을 흉내냅니다.

        success_rate를 0.7로 낮추면 실패 경로의 로깅이 잘 도는지
        확인할 수 있습니다.
        """
        time.sleep(self.delay_sec)
        return random.random() < self.success_rate

    def go_home(self) -> None:
        pass
