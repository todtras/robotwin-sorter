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

        # plane.urdf 대신 단순 박스 바닥. plane.urdf는 100x100m 체커무늬 텍스처
        # 쿼드라 TinyRenderer(소프트웨어 래스터라이저)에서 매 프레임 화면을 거의
        # 다 채우며 그려지는 게 렌더 비용의 대부분(실측 320x240에서 27.7ms 중
        # ~27ms)을 차지함 — 로봇팔이 이동하는 동안 on_step()마다 이 비용을
        # 반복해서 치르는 게 배치 처리 중 화면이 멈춰 보이는 주된 원인이었음
        # (2026-08-05). 작업영역(WORKSPACE_X/Y, 최대 0.65m)과 수거함
        # (BIN_POSITIONS, 최대 0.6m)을 넉넉히 덮는 4x4m 박스로 교체 — halfExtents
        # z를 아주 얇게 잡고 중심을 -half만큼 내려서 윗면이 정확히 z=0에 오게
        # 함(spawner.py가 물체를 z=OBJECT_HALF_EXTENT에 스폰하는 전제와 동일).
        floor_half_extents = [2.0, 2.0, 0.01]
        floor_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=floor_half_extents, rgbaColor=[0.7, 0.7, 0.7, 1.0]
        )
        floor_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=floor_half_extents)
        self.plane_id = p.createMultiBody(
            baseMass=0,  # 0 = 고정체
            baseCollisionShapeIndex=floor_col,
            baseVisualShapeIndex=floor_vis,
            basePosition=[0, 0, -floor_half_extents[2]],
        )
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
