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
from common.schema import FailReason, SortResult, SortTask
from robot.fsm import RobotState


class ArmController:
    """KUKA iiwa 제어기.

    독립 개발 방법 (Day 2~3, 다른 모듈 없이)::

        from tests.dummy_vision import make_dummy_task
        controller = ArmController(robot_id)
        ok = controller.execute_task(make_dummy_task())
    """

    def __init__(self, robot_id: int, urdf_path: str = config.ROBOT_URDF,
                 use_gui: bool = config.USE_GUI) -> None:
        self.robot_id = robot_id
        self.urdf_path = urdf_path
        self.ee_index = config.EE_LINK_INDEX
        self.constraint_id: int | None = None
        self.state = RobotState.IDLE
        self.use_gui = use_gui  # GUI 모드면 move_to()가 240Hz로 재생 (Scene.step()과 동일)
        self._ik_time_ms = 0.0  # move_to() 호출마다 누적. execute_task() 시작 시 리셋.
        self.last_result: SortResult | None = None
        """가장 최근 execute_task() 결과. 로봇이 아는 필드(task, success,
        fail_reason, t_ik_ms, t_execute_ms)만 채워져 있음 — 나머지
        (t_capture_ms 등)는 pipeline이 자기 몫을 채워서 완성합니다."""

        # calculateInverseKinematics()에 관절 한계를 안 알려주면 물리적으로
        # 불가능한 각도를 계산해버릴 수 있음(그러면 위치 제어기가 한계에서
        # 멈춰서 영원히 목표에 못 감). null-space 방식을 쓰려면 필요.
        num_joints = p.getNumJoints(self.robot_id)
        self._joint_lower = []
        self._joint_upper = []
        self._joint_ranges = []
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            self._joint_lower.append(info[8])
            self._joint_upper.append(info[9])
            self._joint_ranges.append(info[9] - info[8])

    def move_to(self, xyz: tuple[float, float, float],
                timeout: float = config.MOVE_TIMEOUT_SEC) -> bool:
        """목표 위치로 이동. 도달하면 True, 실패/타임아웃이면 False."""

        # IK : 해당 위치로 이동하기 위한 관절 각도를 계산하는 것.
        # return 값 : 7개의 관절 각도 리스트
        # 관절 한계(lowerLimits/upperLimits/jointRanges)와 현재 자세(restPoses)를
        # 같이 넘겨서 null-space 방식으로 풀게 함. 안 그러면 한계 밖 각도가
        # 나올 수 있고, 그러면 위치 제어기가 한계에 걸려서 영원히 멈춤.
        rest_poses = [p.getJointState(self.robot_id, i)[0] for i in range(len(self._joint_lower))]
        ik_start = time.time()
        ik = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_index,
            targetPosition=xyz,
            targetOrientation=p.getQuaternionFromEuler([0, 3.14159, 0]),    # 그리퍼가 수직 아래를 보게하는 자세
            lowerLimits=self._joint_lower,
            upperLimits=self._joint_upper,
            jointRanges=self._joint_ranges,
            restPoses=rest_poses,
            maxNumIterations=config.IK_MAX_ITERATIONS,
            residualThreshold=config.IK_RESIDUAL_THRESHOLD
        )
        self._ik_time_ms += (time.time() - ik_start) * 1000

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

        elapsed_sim_time = 0.0

        while True:
            p.stepSimulation()  # 실제 시뮬레이션을 한 스텝 진행. 이걸 안 하면 팔이 움직이지 않음
            elapsed_sim_time += config.SIM_TIMESTEP
            if self.use_gui:
                time.sleep(config.SIM_TIMESTEP * config.SIM_SLOWDOWN)  # GUI 재생 배속 조절 (Scene.step()과 동일)

            # 실제 위치를 가져오기 위해 getLinkState()를 호출하고, 반환된 튜플에서 4번째 요소(월드 좌표)를 actual_position에 저장
            actual_position = p.getLinkState(self.robot_id, self.ee_index)[4]

            # 실제 위치와 목표 위치의 거리 계산
            distance = ((actual_position[0] - xyz[0]) ** 2 +
                        (actual_position[1] - xyz[1]) ** 2 +
                        (actual_position[2] - xyz[2]) ** 2) ** 0.5

            # 임계값 이하 : 목표 위치에 도달한 것으로 간주
            if distance < config.POSITION_TOLERANCE:
                return True

            # 타임아웃 체크: 시뮬레이션된 시간 기준(실제 시계 아님).
            # wall-clock으로 재면 SIM_SLOWDOWN 때문에 GUI에서 재생 배속을
            # 늦출수록 로봇에게 주어지는 물리적 시도 시간이 줄어드는 버그가 생김.
            if elapsed_sim_time > timeout:
                return False

    def settle(self, steps: int = config.SETTLE_STEPS) -> None:
        """관절 목표는 그대로 둔 채 steps번 더 stepSimulation().

        move_to()는 목표 근처(POSITION_TOLERANCE)에 들어오자마자 바로
        return하는데, 그 시점에 아직 잔여 속도가 남아있을 수 있음.
        time.sleep()은 물리 시간을 안 흘려서 이 용도로는 무의미 —
        반드시 stepSimulation()을 더 호출해야 관성이 죽음.
        """
        for _ in range(steps):
            p.stepSimulation()
            if self.use_gui:
                time.sleep(config.SIM_TIMESTEP * config.SIM_SLOWDOWN)

    # -- Day 2 오후 --------------------------------------------------------
    def grasp(self, body_id: int) -> bool:
        """물체를 집습니다 (제약 기반).

        왜 물리 마찰이 아니라 제약인가:
          - KUKA iiwa에는 애초에 그리퍼(손가락)가 없습니다
          - 마찰 방식은 계수/힘/질량 튜닝이 안 맞으면 계속 미끄러집니다
          - 이 프로젝트의 목표는 파지 역학이 아니라 분류 파이프라인 검증입니다

        ★ 항상 True 반환: p.createConstraint는 인자가 유효하면 실패하지
          않고, 제약 방식은 마찰/그립력 없이 즉시 용접이라 "놓침" 자체가
          물리적으로 일어나지 않습니다. FailReason.grasp_lost는 이 방식에서는
          사실상 도달 불가 — execute_task()의 해당 분기는 방어적으로만 남겨둠.
        """

        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot_id,   # 로봇 팔 id
            parentLinkIndex=self.ee_index,      # 그리퍼 링크 인덱스
            childBodyUniqueId=body_id,          # 잡을 물체 id
            childLinkIndex=-1,                  # -1 = 베이스 링크
            jointType=p.JOINT_FIXED,            # 용접 방식으로 붙이기
            jointAxis=[0, 0, 0],
            # 어느 지점을 서로 붙일 것인지 지정
            parentFramePosition=[0, 0, config.GRASP_OFFSET_Z],  
            childFramePosition=[0, 0, 0]
        )

        return True

    def release(self) -> None:
        """물체를 놓습니다."""
    
        if self.constraint_id is not None:
            p.removeConstraint(self.constraint_id)
            self.constraint_id = None

    def go_home(self) -> bool:
        """config.HOME_POSITION으로 복귀. ERROR 복구 시에도 호출됩니다."""
        return self.move_to(config.HOME_POSITION, timeout=config.MOVE_TIMEOUT_SEC)

    # -- Day 3 오전 --------------------------------------------------------
    def execute_task(self, task: SortTask) -> bool:
        """★ 최종 산출물. FSM을 한 바퀴 돌려 분류를 완수합니다.

          IDLE -> APPROACH -> DESCEND -> GRASP -> LIFT
               -> MOVE_TO_BIN -> RELEASE -> RETURN -> IDLE

        반환은 계약대로 bool만. 로봇이 아는 만큼의 SortResult(성공 여부,
        fail_reason, t_ik_ms, t_execute_ms)는 self.last_result에 채워두니
        pipeline이 t_capture_ms 등 나머지를 채워서 완성합니다.
        """
        x, y, z = task.target_xyz
        bin_xyz = config.BIN_POSITIONS[task.target_bin]

        self.state = RobotState.APPROACH
        self._ik_time_ms = 0.0
        fail_reason: FailReason | None = None

        start_time = time.time()

        while True:
            match self.state:
                case RobotState.APPROACH:
                    if self.move_to((x, y, z + config.APPROACH_HEIGHT), timeout=config.MOVE_TIMEOUT_SEC):
                        self.state = RobotState.DESCEND
                        t_approach_ms = (time.time() - start_time) * 1000
                    else:
                        fail_reason = "timeout : approach"
                        self.state = RobotState.ERROR

                case RobotState.DESCEND:
                    # z까지 그대로 내려가면 그리퍼(flange)가 바닥과 충돌해 못 내려감.
                    # GRASP_OFFSET_Z만큼 띄운 높이에서 멈추고, grasp()의 constraint
                    # 프레임 오프셋이 물체를 그 지점까지 끌어올려 붙잡음.
                    if self.move_to((x, y, z + config.GRASP_OFFSET_Z), timeout=config.MOVE_TIMEOUT_SEC):
                        self.state = RobotState.GRASP
                        t_descend_ms = (time.time() - start_time) * 1000
                    else:
                        fail_reason = "timeout : descend"
                        self.state = RobotState.ERROR

                case RobotState.GRASP:
                    # grasp()는 제약 방식이라 사실상 항상 True.
                    # else는 grasp() 구현이 나중에 바뀔 경우를 위한 방어 코드.
                    if self.grasp(task.body_id):
                        t_grasp_ms = (time.time() - start_time) * 1000
                        self.state = RobotState.LIFT
                    else:
                        fail_reason = "grasp_lost"
                        self.state = RobotState.ERROR

                case RobotState.LIFT:
                    if self.move_to((x, y, z + config.LIFT_HEIGHT), timeout=config.MOVE_TIMEOUT_SEC):
                        self.state = RobotState.MOVE_TO_BIN
                        t_lift_ms = (time.time() - start_time) * 1000
                    else:
                        fail_reason = "timeout : lift"
                        self.state = RobotState.ERROR

                case RobotState.MOVE_TO_BIN:
                    # bin_xyz를 그대로 쓰면 통 바닥(z=0)까지 내려가라는 뜻이 돼서
                    # DESCEND와 같은 이유로 바닥과 충돌함. LIFT 높이를 유지한 채
                    # 통 상공으로 수평 이동만 하고, 물체는 RELEASE에서 낙하시킴.
                    if self.move_to((bin_xyz[0], bin_xyz[1], z + config.LIFT_HEIGHT), timeout=config.MOVE_TIMEOUT_SEC):
                        
                        self.state = RobotState.RELEASE
                        t_move_to_bin_ms = (time.time() - start_time) * 1000
                    else:
                        fail_reason = "timeout : move_to_bin"
                        self.state = RobotState.ERROR

                case RobotState.RELEASE:
                    self.settle()  # 바로 놓으면 관성으로 인해 물체가 날라감. 안정화 시간
                    self.release()
                    t_release_ms = (time.time() - start_time) * 1000

                    self.state = RobotState.RETURN

                case RobotState.RETURN:
                    if self.go_home():
                        t_return_ms = (time.time() - start_time) * 1000
                        self.state = RobotState.IDLE

                        self.last_result = SortResult(
                            task=task,
                            success=True,
                            fail_reason=None,
                            t_ik_ms=self._ik_time_ms,
                            t_execute_ms=(time.time() - start_time) * 1000,
                        )
                        # 나머지 필드(t_capture_ms 등)는 pipeline이 채웁니다.

                        return True

                    fail_reason = "timeout : return"
                    self.state = RobotState.ERROR

                case RobotState.ERROR:
                    # GRASP 이후(LIFT/MOVE_TO_BIN/RETURN)에 실패했으면 물체가
                    # 아직 팔에 용접된 상태. 안 놓으면 홈 복귀 중 끌려오고,
                    # 다음 태스크의 grasp()가 이전 물체 위에 또 얹힘.
                    self.release()
                    self.go_home()
                    self.state = RobotState.IDLE

                    self.last_result = SortResult(
                        task=task,
                        success=False,
                        fail_reason=fail_reason,
                        t_ik_ms=self._ik_time_ms,
                        t_execute_ms=(time.time() - start_time) * 1000,
                    )
                    # 나머지 필드(t_capture_ms 등)는 pipeline이 채웁니다.

                    return False
