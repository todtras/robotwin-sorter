"""
robot/fsm.py — 로봇 상태 기계
담당: 김태익 | Day 3 오전

한 번의 분류 작업을 9개 상태로 쪼갭니다. 상태를 안 나누고 순차 코드로 짜면
"어디서 실패했는지"를 로그에 남길 수 없어 실험 데이터가 나오지 않습니다.
"""

from __future__ import annotations

from enum import Enum, auto


class RobotState(Enum):
    """로봇 상태.

    정상 흐름:
        IDLE -> APPROACH -> DESCEND -> GRASP -> LIFT
             -> MOVE_TO_BIN -> RELEASE -> RETURN -> IDLE

    어느 상태에서든 IK 실패나 타임아웃이면 ERROR로 빠집니다.
    """

    IDLE = auto()
    """새 SortTask 수신 대기. 팔은 홈 포지션."""

    APPROACH = auto()
    """물체 위 config.APPROACH_HEIGHT(10cm) 상공으로 이동.
    바로 대각선 하강하면 물체를 옆에서 쳐서 날립니다."""

    DESCEND = auto()
    """수직 하강. x, y는 그대로 두고 z만 낮춥니다."""

    GRASP = auto()
    """p.createConstraint로 팔 끝과 물체를 고정 관절로 용접.
    물리 마찰 방식은 미끄러져서 실패합니다 (기획서 4.1 Step 3)."""

    LIFT = auto()
    """config.LIFT_HEIGHT(15cm) 상승. 이동 중 충돌 방지."""

    MOVE_TO_BIN = auto()
    """config.BIN_POSITIONS[task.target_bin] 상공으로 수평 이동."""

    RELEASE = auto()
    """p.removeConstraint로 용접 해제. 물체가 통 안으로 낙하."""

    RETURN = auto()
    """config.HOME_POSITION으로 복귀. 카메라 시야를 비켜줍니다."""

    ERROR = auto()
    """IK 실패 또는 타임아웃. 홈 복귀 후 IDLE로 돌아가고
    해당 태스크는 실패로 기록됩니다."""


TRANSITIONS: dict[RobotState, tuple[RobotState, str]] = {
    # 현재 상태: (다음 상태, 전이 조건)
    RobotState.IDLE:        (RobotState.APPROACH,    "SortTask 수신 & 목표가 작업영역 내"),
    RobotState.APPROACH:    (RobotState.DESCEND,     "상공 도달 (오차 1cm 이내)"),
    RobotState.DESCEND:     (RobotState.GRASP,       "물체 높이 도달"),
    RobotState.GRASP:       (RobotState.LIFT,        "제약 생성 완료"),
    RobotState.LIFT:        (RobotState.MOVE_TO_BIN, "안전 높이 도달"),
    RobotState.MOVE_TO_BIN: (RobotState.RELEASE,     "수거함 상공 도달"),
    RobotState.RELEASE:     (RobotState.RETURN,      "제약 해제 완료"),
    RobotState.RETURN:      (RobotState.IDLE,        "홈 포지션 도달"),
    RobotState.ERROR:       (RobotState.IDLE,        "홈 복귀 완료 (태스크는 실패 기록)"),
}
"""정상 전이표. 문서용이자 검증용입니다.
보고서의 상태 다이어그램을 이 표에서 그대로 그리면 됩니다."""
