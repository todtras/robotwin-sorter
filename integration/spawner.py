"""
integration/spawner.py — PyBullet 동적 객체 스폰
담당: 진선우 | Day 3 오전

실제 웹캠에서 검출된 물체를 가상 세계에 "복제"하는 부분.
디지털 트윈이라는 이름이 여기서 나옵니다.
"""

from __future__ import annotations

import pybullet as p

import config
from common.schema import CATEGORY_TO_BIN, Detection, SortTask


class ObjectSpawner:
    """검출된 물체를 시뮬레이션에 생성/제거합니다."""

    def __init__(self) -> None:
        self.active_bodies: dict[int, SortTask] = {}

    def spawn(self, detection: Detection,
              world_xy: tuple[float, float]) -> SortTask:
        """물체를 스폰하고 로봇에게 넘길 SortTask를 만듭니다.

        TODO(선우) Day 3 오전:"""
        half = config.OBJECT_HALF_EXTENT
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half]*3,
                                      rgbaColor=config.CATEGORY_COLORS[detection.category])
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half]*3)
        body_id = p.createMultiBody(
            baseMass=config.OBJECT_MASS,
            baseCollisionShapeIndex=col,      # ★ 빼먹으면 바닥을 뚫고 떨어짐
            baseVisualShapeIndex=vis,
            basePosition=[world_xy[0], world_xy[1], half],
        )
        task = SortTask(
            body_id=body_id,
            target_xyz=(world_xy[0], world_xy[1], half),
            target_bin=CATEGORY_TO_BIN[detection.category],
            category=detection.category,
            source=detection,
        )
        self.active_bodies[body_id] = task
        return task

        """
        ★ basePosition의 z는 물체 절반 높이 이상이어야 합니다.
          0으로 두면 바닥에 반쯤 박힌 채 시작합니다.

        여유가 되면(Day 8 이후): pet은 GEOM_CYLINDER, can은 짧은 원기둥으로
        형상을 구분하면 디지털 트윈다운 화면이 나옵니다. 우선순위 낮음.
        """

    def remove(self, body_id: int) -> None:
        """작업이 끝난 물체를 제거. p.removeBody(body_id).

        ★ 제거를 안 하면 수거함 위에 박스가 쌓여 다음 물체와 충돌합니다.
        """
        p.removeBody(body_id)
        self.active_bodies.pop(body_id,None)
