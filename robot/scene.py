"""
robot/scene.py — PyBullet 씬 구성
담당: 김태익 | Day 1 오후

바닥, 로봇팔, 수거함 3개를 배치해 시뮬레이션 무대를 만듭니다.
"""

from __future__ import annotations

import time
import pybullet as p
import pybullet_data
import config


class Scene:
    """PyBullet 시뮬레이션 씬.

    사용 예::

        scene = Scene()
        scene.build()
        robot_id = scene.robot_id
    """

    def __init__(self, use_gui: bool = config.USE_GUI) -> None:
        self.use_gui = use_gui
        self.client_id: int | None = None
        self.plane_id: int | None = None
        self.robot_id: int | None = None
        self.bin_ids: dict[str, int] = {}

    def build(self) -> None:
        """씬 전체를 구성합니다."""
        
        # 모드를 GUI / DIRECT로 선택 후, PyBullet 서버에 연결
        mode = p.GUI if self.use_gui else p.DIRECT
        self.client_id = p.connect(mode)

        # pybullet_data 안의 urdf, obj, stl 파일을 찾기 위한 경로 추가
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

        p.setGravity(0, 0, -9.81)  # 중력 설정

        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(
            config.ROBOT_URDF,                          # config.py에 정의된 모델 사용    
            useFixedBase=True)                          # Base 고정 안 하면 팔이 중력에 쓰러짐

        num_joints = p.getNumJoints(self.robot_id)      # 관절 수 확인

        print(f"[Scene] plane_id = {self.plane_id}, robot_id = {self.robot_id}, num_joints = {num_joints}")
        if num_joints <= config.EE_LINK_INDEX:
            raise ValueError(f"EE_LINK_INDEX={config.EE_LINK_INDEX} is out of range for robot with {num_joints} joints")
        
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)      # 관절 정보 확인
            print(f"  joint {i}: {info[1].decode()}  type = {info[2]}  lower = {info[8]}  upper = {info[9]}")

        self._spawn_bins()  # 수거함 3개 배치

        if self.use_gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=config.SIM_CAMERA_DISTANCE,
                cameraYaw=config.SIM_CAMERA_YAW,
                cameraPitch=config.SIM_CAMERA_PITCH,
                cameraTargetPosition=config.SIM_CAMERA_TARGET,
            )

        return self.client_id, self.plane_id, self.robot_id

    def _spawn_bins(self) -> None:
        """수거함 3개를 색상 박스로 배치."""
        
        for name, pos in config.BIN_POSITIONS.items():
            vis = p.createVisualShape(p.GEOM_BOX,
                                      halfExtents=config.BIN_HALF_EXTENTS,
                                      rgbaColor=config.BIN_COLORS[name])
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=config.BIN_HALF_EXTENTS)
            body = p.createMultiBody(baseMass=0,                              # 0 = 고정체
                                     baseCollisionShapeIndex=col,              # ★ 없으면 물체가 통을 뚫고 떨어짐
                                     baseVisualShapeIndex=vis,
                                     basePosition=pos)
            self.bin_ids[name] = body

            if self.use_gui:
                label_pos = (pos[0], pos[1], pos[2] + config.BIN_HALF_EXTENTS[2] + 0.02)
                p.addUserDebugText(config.BIN_LABELS[name], label_pos,
                                    textColorRGB=[1, 1, 1], textSize=1.5)


    def step(self, n: int = 1) -> None:
        """시뮬레이션을 n스텝 진행. p.stepSimulation()을 n번 호출."""
        for _ in range(n):
            p.stepSimulation()
            if self.use_gui:
                time.sleep(config.SIM_TIMESTEP * config.SIM_SLOWDOWN)  # GUI 재생 배속 조절

    def disconnect(self) -> None:
        """p.disconnect(). 프로그램 종료 시 반드시 호출."""
        p.disconnect(self.client_id)

if __name__ == "__main__":
    # Day 1 DoD 확인용: python -m robot.scene
    # 창이 뜨고 로봇 + 색깔 통 3개가 보이면 성공
    scene = Scene()
    scene.build()
    input("엔터를 누르면 종료합니다...")
    scene.disconnect()
