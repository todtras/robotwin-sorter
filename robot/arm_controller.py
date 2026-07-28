"""
robot/arm_controller.py — 로봇팔 제어
담당: 김태익 | Day 2 ~ Day 3

★ 이 파일이 로봇 파트의 최종 산출물입니다.
  다른 모듈은 execute_task() 하나만 알면 됩니다.
"""

from __future__ import annotations

import config
from common.schema import SortTask
from robot.fsm import RobotState


class ArmController:
    """KUKA iiwa 제어기.

    독립 개발 방법 (Day 2~3, 다른 모듈 없이)::

        from tests.dummy_vision import make_dummy_task
        controller = ArmController(robot_id)
        ok = controller.execute_task(make_dummy_task())
    """

    def __init__(self, robot_id: int, urdf_path: str = config.ROBOT_URDF) -> None:
        self.robot_id = robot_id
        self.urdf_path = urdf_path
        self.ee_index = config.EE_LINK_INDEX
        self.constraint_id: int | None = None
        self.state = RobotState.IDLE

    # -- Day 2 오전 --------------------------------------------------------
    def move_to(self, xyz: tuple[float, float, float],
                timeout: float = config.MOVE_TIMEOUT_SEC) -> bool:
        """목표 위치로 이동. 도달하면 True, 실패/타임아웃이면 False.

        TODO(태익) Day 2 오전:
          1. p.calculateInverseKinematics(
                 bodyUniqueId=self.robot_id,
                 endEffectorLinkIndex=self.ee_index,
                 targetPosition=xyz,
                 targetOrientation=p.getQuaternionFromEuler([0, math.pi, 0]),
                 maxNumIterations=config.IK_MAX_ITERATIONS,
                 residualThreshold=config.IK_RESIDUAL_THRESHOLD)
             -> Euler [0, pi, 0]은 그리퍼가 수직 아래를 보게 하는 자세
          2. 각 관절에 p.setJointMotorControl2(POSITION_CONTROL,
                 force=config.JOINT_FORCE)
          3. while 루프:
               p.stepSimulation()            <- 이거 빼먹으면 팔이 안 움직임
               실제 = p.getLinkState(...)[4]  <- 월드 위치
               if 거리(실제, xyz) < config.POSITION_TOLERANCE: return True
               if 경과시간 > timeout: return False

        ★ 흔한 실수: 도달 판정 루프에서 stepSimulation()을 안 부름.
          IK는 계산됐는데 팔이 제자리인 현상의 원인입니다.
        """
        raise NotImplementedError

    # -- Day 2 오후 --------------------------------------------------------
    def grasp(self, body_id: int) -> bool:
        """물체를 집습니다 (제약 기반).

        TODO(태익) Day 2 오후:
            self.constraint_id = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=self.ee_index,
                childBodyUniqueId=body_id,
                childLinkIndex=-1,               # -1 = 베이스 링크
                jointType=p.JOINT_FIXED,         # 용접
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, config.GRASP_OFFSET_Z],
                childFramePosition=[0, 0, 0],
            )
            return True

        왜 물리 마찰이 아니라 제약인가:
          - KUKA iiwa에는 애초에 그리퍼(손가락)가 없습니다
          - 마찰 방식은 계수/힘/질량 튜닝이 안 맞으면 계속 미끄러집니다
          - 이 프로젝트의 목표는 파지 역학이 아니라 분류 파이프라인 검증입니다

        ★ 단, 마찰 방식도 한 번은 시도해서 실패 화면을 남기세요.
          보고서에 "마찰 시도 -> 미끄러짐 -> 제약 전환"으로 쓰면
          근거 있는 기술적 의사결정이 됩니다.
        """
        raise NotImplementedError

    def release(self) -> None:
        """물체를 놓습니다.

        TODO(태익):
            if self.constraint_id is not None:
                p.removeConstraint(self.constraint_id)
                self.constraint_id = None

        ★ None 체크 필수. 안 잡은 상태에서 호출하면 PyBullet이 죽습니다.
        """
        raise NotImplementedError

    def go_home(self) -> None:
        """config.HOME_POSITION으로 복귀. ERROR 복구 시에도 호출됩니다."""
        raise NotImplementedError

    # -- Day 3 오전 --------------------------------------------------------
    def execute_task(self, task: SortTask) -> bool:
        """★ 최종 산출물. FSM을 한 바퀴 돌려 분류를 완수합니다.

        TODO(태익) Day 3 오전:
          IDLE -> APPROACH -> DESCEND -> GRASP -> LIFT
               -> MOVE_TO_BIN -> RELEASE -> RETURN -> IDLE

          x, y, z = task.target_xyz
          bin_xyz = config.BIN_POSITIONS[task.target_bin]

          단계마다 move_to() 반환값을 확인하고, False면 ERROR로 전이 후
          go_home() -> return False.

        ★ Day 3 오후에 단계별 시간 측정 코드를 미리 심어두세요.
          (t_ik_ms, t_execute_ms — 실험 3에서 씁니다.
           Day 7에 급하게 넣으려면 이미 실험이 끝나 있습니다.)
        """
        raise NotImplementedError
