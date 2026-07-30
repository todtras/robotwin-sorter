"""
integration/pipeline.py — 메인 진입점
담당: 진선우 | Day 3 ~ Day 4

실행::

    python -m integration.pipeline

전체 시스템을 이 파일 하나로 돌립니다. Day 4 통합의 무대입니다.
"""

from __future__ import annotations
import time
import pybullet as p
import pybullet_data

import config
from common.logger import SortLogger
from common.schema import SortResult
from integration.calibration import Calibrator
from integration.spawner import ObjectSpawner
from tests.dummy_robot import DummyArmController
from tests.dummy_vision import DummyDetector


class Pipeline:
    """전체 시스템 메인 루프."""
    def __init__(self, use_dummy: bool = True) -> None:

        # --- 모듈 생성 ---
        self.calibrator = Calibrator()
        self.spawner = ObjectSpawner()
        self.logger = SortLogger()

        if use_dummy:
            self.detector = DummyDetector(detect_probability=0.3)
            self.arm = DummyArmController(success_rate=0.9, delay_sec=0.5)
                    # --- PyBullet 초기화 ---
            mode = p.GUI if config.USE_GUI else p.DIRECT
            self.physics_client = p.connect(mode)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.loadURDF("plane.urdf")

            self.camera = None
            self.detector = DummyDetector(detect_probability=0.3)
            self.arm = DummyArmController(success_rate=0.9, delay_sec=0.5)

        else:
            from vision.camera import Camera
            from vision.detector import TrashDetector
            from robot.scene import Scene
            from robot.arm_controller import ArmController

            scene = Scene()
            scene.build()
            self.camera = Camera()
            self.camera.open()
            self.detector = TrashDetector()
            self.arm = ArmController(scene.robot_id)

        # 중복 처리 방지용
        self.processing_coords: list[tuple[float, float]] = []

    def is_duplicate(self, wx: float, wy: float) -> bool:
        """이미 처리 중인 좌표 근처인지 확인."""
        for cx, cy in self.processing_coords:
            dist = ((wx - cx) ** 2 + (wy - cy) ** 2) ** 0.5
            if dist < config.DUPLICATE_RADIUS_M:
                return True
        return False

    def run(self, max_cycles: int = 20) -> None:
        """메인 루프. max_cycles만큼 처리 후 종료."""
        print(f"[pipeline] 시작 (max_cycles={max_cycles})")
        cycle = 0

        while cycle < max_cycles:
            # ① 검출
            t0 = time.time()
            if self.camera is not None:
                frame = self.camera.read()
            else:
                frame = None
            detections = self.detector.detect(frame)
            t_detect = (time.time() - t0) * 1000

            if not detections:
                p.stepSimulation()
                time.sleep(config.SIM_TIMESTEP)
                continue

            for det in detections:
                # ② 좌표 변환
                t1 = time.time()
                wx, wy = self.calibrator.pixel_to_world(det.pixel_x, det.pixel_y)
                t_transform = (time.time() - t1) * 1000

                # 작업영역 확인
                if not self.calibrator.is_in_workspace(wx, wy):
                    self.logger.record_failure(det, "out_of_workspace")
                    continue

                # 중복 확인
                if self.is_duplicate(wx, wy):
                    continue

                # ③ 스폰
                self.processing_coords.append((wx, wy))
                task = self.spawner.spawn(det, (wx, wy))

                # ④ 로봇 실행
                t2 = time.time()
                ok = self.arm.execute_task(task)
                t_execute = (time.time() - t2) * 1000

                # ⑤ 로깅
                result = SortResult(
                    task=task,
                    success=ok,
                    fail_reason=None if ok else "timeout",
                    t_detect_ms=t_detect,
                    t_transform_ms=t_transform,
                    t_execute_ms=t_execute,
                )
                self.logger.record(result)

                # ⑥ 정리
                self.spawner.remove(task.body_id)
                self.processing_coords.remove((wx, wy))

                cycle += 1
                print(f"  [{cycle}] {det.category} -> {task.target_bin} | {'OK' if ok else 'FAIL'}")

            p.stepSimulation()

        # 결과 요약
        summary = self.logger.summary()
        print(f"\n[pipeline] 완료!")
        print(f"  총 {summary['total']}회, 성공률 {summary['success_rate']:.0%}")
        if summary["fail_reasons"]:
            print(f"  실패 사유: {summary['fail_reasons']}")
        print(f"  CSV: {self.logger.path}")

    def shutdown(self) -> None:
        p.disconnect()

def main() -> None:
    pipeline = Pipeline(use_dummy=True)# 더미 on off
    try:
        pipeline.run(max_cycles=20)
    finally:
        pipeline.shutdown()

if __name__ == "__main__":
    main()
