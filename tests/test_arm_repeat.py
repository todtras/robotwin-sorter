"""
tests/test_arm_repeat.py — 더미 SortTask 10회 반복 성공률 측정
담당: 김태익 | Day 3 오후

Day 3 오후 DoD: 더미 좌표 10회 중 8회 성공.
실행: python -m tests.test_arm_repeat
"""

from __future__ import annotations

import dataclasses

import pybullet as p

import config
from robot.arm_controller import ArmController
from robot.scene import Scene
from tests.dummy_vision import make_dummy_task
from tests.test_arm_dummy import spawn_test_object

N_RUNS = 10
TARGET_SUCCESS = 8


def run_repeat(n: int = N_RUNS, use_gui: bool = False) -> None:
    scene = Scene(use_gui=use_gui)
    scene.build()
    controller = ArmController(scene.robot_id, use_gui=use_gui)

    results: list[tuple[bool, str | None]] = []

    for i in range(n):
        task = make_dummy_task()  # body_id=0(가짜)로 나오니 실제 물체로 교체
        body_id = spawn_test_object(task.target_xyz)
        task = dataclasses.replace(task, body_id=body_id)

        label_id = None
        if use_gui:
            # parentObjectUniqueId를 주면 텍스트 위치가 그 body 기준 로컬
            # 좌표가 되어, 물체가 들려서 움직여도 텍스트가 따라다님.
            local_offset = (0, 0, config.OBJECT_HALF_EXTENT + 0.02)
            label_id = p.addUserDebugText(config.BIN_LABELS[task.target_bin], local_offset,
                                           textColorRGB=[1, 1, 0], textSize=1.5,
                                           parentObjectUniqueId=body_id, parentLinkIndex=-1)

        ok = controller.execute_task(task)
        fail_reason = controller.last_result.fail_reason if controller.last_result else None
        results.append((ok, fail_reason))

        print(f"[{i + 1}/{n}] category={task.category} bin={task.target_bin} "
              f"xyz={tuple(round(v, 3) for v in task.target_xyz)} "
              f"ok={ok} fail_reason={fail_reason}")

        p.removeBody(body_id)  # 다음 반복과 안 겹치게 정리
        if label_id is not None:
            p.removeUserDebugItem(label_id)

    scene.disconnect()

    success = sum(1 for ok, _ in results if ok)
    print(f"\n성공률: {success}/{n} (목표 {TARGET_SUCCESS}/{n})")
    if success < TARGET_SUCCESS:
        fail_reasons = [r for ok, r in results if not ok]
        print(f"실패 사유 분포: {fail_reasons}")


if __name__ == "__main__":
    # 성공률 측정용 자동화 스크립트라 기본은 헤드리스(빠르게).
    # 눈으로 보고 싶으면 run_repeat(use_gui=True)로 직접 호출하세요.
    run_repeat(use_gui=True)
