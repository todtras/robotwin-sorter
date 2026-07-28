"""
robot/scene.py — PyBullet 씬 구성
담당: 김태익 | Day 1 오후

바닥, 로봇팔, 수거함 3개를 배치해 시뮬레이션 무대를 만듭니다.
"""

from __future__ import annotations

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
        """씬 전체를 구성합니다.

        TODO(태익) Day 1:
          1. p.connect(p.GUI if use_gui else p.DIRECT)
          2. p.setAdditionalSearchPath(pybullet_data.getDataPath())
             -> 이걸 안 하면 plane.urdf를 못 찾습니다
          3. p.setGravity(0, 0, -9.81)
          4. self.plane_id = p.loadURDF("plane.urdf")
          5. self.robot_id = p.loadURDF(config.ROBOT_URDF, useFixedBase=True)
             -> useFixedBase=True 빼먹으면 로봇이 넘어집니다
          6. self._spawn_bins()
          7. p.getNumJoints(self.robot_id)로 관절 수 확인 ->
             config.EE_LINK_INDEX가 맞는지 검증
        """
        raise NotImplementedError

    def _spawn_bins(self) -> None:
        """수거함 3개를 색상 박스로 배치.

        TODO(태익) Day 1:
          config.BIN_POSITIONS / BIN_COLORS를 순회하며

            vis = p.createVisualShape(p.GEOM_BOX,
                                      halfExtents=config.BIN_HALF_EXTENTS,
                                      rgbaColor=config.BIN_COLORS[name])
            body = p.createMultiBody(baseMass=0,          # 0 = 고정체
                                     baseVisualShapeIndex=vis,
                                     basePosition=pos)
            self.bin_ids[name] = body

        실제로 물체가 "담기는" 물리 구현은 불필요합니다.
        수거함 상공에서 떨어뜨리면 성공으로 간주합니다.
        """
        raise NotImplementedError

    def step(self, n: int = 1) -> None:
        """시뮬레이션을 n스텝 진행. p.stepSimulation()을 n번 호출."""
        raise NotImplementedError

    def disconnect(self) -> None:
        """p.disconnect(). 프로그램 종료 시 반드시 호출."""
        raise NotImplementedError


if __name__ == "__main__":
    # Day 1 DoD 확인용: python -m robot.scene
    # 창이 뜨고 로봇 + 색깔 통 3개가 보이면 성공
    scene = Scene()
    scene.build()
    input("엔터를 누르면 종료합니다...")
    scene.disconnect()
