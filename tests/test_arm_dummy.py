"""
tests/test_arm_dummy.py — robot/ 단독 더미 테스트
담당: 김태익 | Day 3

Scene + ArmController + FSM을 다른 모듈(비전/통합) 없이 한 바퀴 돌려봅니다.
실행: python -m tests.test_arm_dummy

★ SortTask.body_id=0인 tests/dummy_vision.make_dummy_task()는 진짜
  PyBullet 객체가 아니라서 grasp()가 실제로 뭔가를 붙잡는지는 확인이
  안 됩니다. 여기서는 p.createMultiBody()로 박스를 하나 직접 띄우고
  그 id를 SortTask.body_id에 넣어서 grasp/lift/release까지 검증합니다.
"""

from __future__ import annotations

import pybullet as p

import config
from common.schema import CATEGORY_TO_BIN, SortTask
from robot.arm_controller import ArmController
from robot.scene import Scene


def spawn_test_object(xyz: tuple[float, float, float]) -> int:
    """집을 물체로 쓸 작은 박스를 xyz에 띄우고 body_id를 반환."""
    half_extents = [config.WORKSPACE_Z] * 3  # z와 맞춰 테이블에 딱 붙게
    return p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[1, 0, 0, 1]),
        basePosition=xyz,
    )


def run_dummy_task() -> None:
    scene = Scene(use_gui=True)
    scene.build()

    xyz = (0.5, 0.0, config.WORKSPACE_Z)
    body_id = spawn_test_object(xyz)

    category = "pet"
    task = SortTask(
        body_id=body_id,
        target_xyz=xyz,
        target_bin=CATEGORY_TO_BIN[category],
        category=category,
    )

    controller = ArmController(scene.robot_id)
    ok = controller.execute_task(task)

    print("성공:", ok)
    print("last_result:", controller.last_result)

    input("엔터를 누르면 종료합니다...")
    scene.disconnect()


if __name__ == "__main__":
    run_dummy_task()
