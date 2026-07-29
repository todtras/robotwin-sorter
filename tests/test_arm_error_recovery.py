"""
tests/test_arm_error_recovery.py — ERROR 상태 복구 검증
담당: 김태익 | Day 3

일부러 타임아웃을 내서 ERROR -> IDLE 복구가 제대로 되는지, 그리고
복구 후 같은 컨트롤러로 새 태스크를 정상 수행할 수 있는지 확인합니다.
실행: python -m tests.test_arm_error_recovery

★ 도달 불가능한 좌표(로봇 팔 사정권 밖)를 목표로 줘서 APPROACH의
  move_to()가 config.MOVE_TIMEOUT_SEC(정상값)을 다 채우고 진짜로
  타임아웃 나게 만드는 방식입니다.

  ★ config.MOVE_TIMEOUT_SEC 자체를 줄이는 방법도 되지만, 그러면
    ERROR 핸들러 안에서 호출되는 go_home()도 같은(짧아진) 타임아웃을
    쓰게 돼서 복귀 이동 자체가 중간에 잘립니다. 좌표를 불가능하게
    만드는 쪽이 go_home()엔 영향이 없어서 실제로 홈까지 복귀하는
    걸 눈으로 볼 수 있습니다.
"""

from __future__ import annotations

import config
from common.schema import CATEGORY_TO_BIN, SortTask
from robot.arm_controller import ArmController
from robot.fsm import RobotState
from robot.scene import Scene
from tests.test_arm_dummy import spawn_test_object


def make_task(body_id: int, xyz: tuple[float, float, float], category: str = "pet") -> SortTask:
    return SortTask(
        body_id=body_id,
        target_xyz=xyz,
        target_bin=CATEGORY_TO_BIN[category],
        category=category,
    )


UNREACHABLE_XYZ = (5.0, 5.0, 0.5)
"""로봇 팔 사정권(~0.8m) 밖. APPROACH의 move_to()가 절대 수렴 못 해서
config.MOVE_TIMEOUT_SEC을 그대로 다 채우고 타임아웃 납니다."""


def test_error_recovery(use_gui: bool = False) -> None:
    scene = Scene(use_gui=use_gui)
    scene.build()

    xyz = (0.5, 0.0, config.WORKSPACE_Z)
    body_id = spawn_test_object(xyz)
    controller = ArmController(scene.robot_id, use_gui=use_gui)

    # body_id는 정상 위치에 있지만, 목표 좌표를 사정권 밖으로 줘서
    # 일부러 도달 실패(타임아웃)를 유발.
    ok = controller.execute_task(make_task(body_id, UNREACHABLE_XYZ))

    assert ok is False, "타임아웃을 냈는데 성공으로 처리됨"
    assert controller.state == RobotState.IDLE, f"ERROR 복구 후 IDLE이 아님: {controller.state}"
    assert controller.last_result is not None, "last_result가 안 채워짐"
    assert controller.last_result.success is False
    assert controller.last_result.fail_reason == "timeout", controller.last_result.fail_reason
    print("[1/2] 타임아웃 -> ERROR -> IDLE 복구 확인:", controller.last_result)
    if use_gui:
        input("팔이 홈으로 복귀한 게 보이면 엔터를 눌러 다음 태스크로 진행...")

    # 복구 후에도 같은 컨트롤러로 정상 태스크를 처리할 수 있어야 함.
    # (물체는 아직 안 잡혔으니 같은 body_id를 재사용해도 됨)
    ok2 = controller.execute_task(make_task(body_id, xyz))

    assert ok2 is True, "복구 후 정상 태스크가 실패함"
    assert controller.last_result.success is True
    print("[2/2] 복구 후 정상 태스크 재수행 확인:", controller.last_result)

    if use_gui:
        input("엔터를 누르면 종료합니다...")
    scene.disconnect()


if __name__ == "__main__":
    test_error_recovery(use_gui=config.USE_GUI)
    print("모든 검증 통과")
