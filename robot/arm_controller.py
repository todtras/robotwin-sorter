"""
robot/arm_controller.py — 로봇팔 제어
담당: 김태익 | Day 2 ~ Day 3

★ 이 파일이 로봇 파트의 최종 산출물입니다.
  다른 모듈은 execute_task() 하나만 알면 됩니다.
"""

from __future__ import annotations
import time

import pybullet as p

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

    def move_to(self, xyz: tuple[float, float, float],
                timeout: float = config.MOVE_TIMEOUT_SEC) -> bool:
        """목표 위치로 이동. 도달하면 True, 실패/타임아웃이면 False."""

        # IK : 해당 위치로 이동하기 위한 관절 각도를 계산하는 것.
        # return 값 : 7개의 관절 각도 리스트
        ik = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_index,
            targetPosition=xyz,
            targetOrientation=p.getQuaternionFromEuler([0, 3.14159, 0]),    # 그리퍼가 수직 아래를 보게하는 자세
            maxNumIterations=config.IK_MAX_ITERATIONS,
            residualThreshold=config.IK_RESIDUAL_THRESHOLD
        )

        for i in range(7):
            print(f"Joint {i} : {ik[i]}")

        # 관절의 모터를 POSITION_CONTROL 모드로 설정하고, 목표 위치를 IK로 계산된 각도로 설정
        for i in range(p.getNumJoints(self.robot_id)):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=ik[i],
                force=config.JOINT_FORCE
            )

        start_time = time.time()

        while True:
            p.stepSimulation()  # 실제 시뮬레이션을 한 스텝 진행. 이걸 안 하면 팔이 움직이지 않음

            # 실제 위치를 가져오기 위해 getLinkState()를 호출하고, 반환된 튜플에서 4번째 요소(월드 좌표)를 actual_position에 저장
            actual_position = p.getLinkState(self.robot_id, self.ee_index)[4]

            # 실제 위치와 목표 위치의 거리 계산
            distance = ((actual_position[0] - xyz[0]) ** 2 +
                        (actual_position[1] - xyz[1]) ** 2 +
                        (actual_position[2] - xyz[2]) ** 2) ** 0.5

            # 임계값 이하 : 목표 위치에 도달한 것으로 간주
            if distance < config.POSITION_TOLERANCE:
                return True

            # 타임아웃 체크 : 지정된 시간 초과 시 False 반환
            if time.time() - start_time > timeout:
                return False

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
        pass

    def release(self) -> None:
        """물체를 놓습니다.

        TODO(태익):
            if self.constraint_id is not None:
                p.removeConstraint(self.constraint_id)
                self.constraint_id = None

        ★ None 체크 필수. 안 잡은 상태에서 호출하면 PyBullet이 죽습니다.
        """
        pass

    def go_home(self) -> None:
        """config.HOME_POSITION으로 복귀. ERROR 복구 시에도 호출됩니다."""
        pass

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
        pass
