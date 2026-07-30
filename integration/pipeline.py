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
        # --- PyBullet 초기화 ---
        mode = p.GUI if config.USE_GUI else p.DIRECT
        self.physics_client = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        # --- 모듈 생성 ---
        self.calibrator = Calibrator()
        self.spawner = ObjectSpawner()
        self.logger = SortLogger()

        if use_dummy:
            self.detector = DummyDetector(detect_probability=0.3)
            self.arm = DummyArmController(success_rate=0.9, delay_sec=0.5)
        else:
            # Day 5에 실제 모듈로 교체
            raise NotImplementedError("실제 모듈은 Day 5에 연결")

        # 중복 처리 방지용
        self.processing_coords: list[tuple[float, float]] = []
        self._running = False

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
            detections = self.detector.detect(frame=None)
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
    pipeline = Pipeline(use_dummy=True)
    try:
        pipeline.run(max_cycles=20)
    finally:
        pipeline.shutdown()
    """메인 루프.

    TODO(선우) Day 3~4:

        camera     = Camera();            camera.open()
        detector   = TrashDetector()
        calibrator = Calibrator()
        scene      = Scene();             scene.build()
        spawner    = ObjectSpawner()
        arm        = ArmController(scene.robot_id)
        logger     = SortLogger()

        while True:
            frame = camera.read()
            detections = detector.detect(frame)

            for det in detections:
                if not stability.is_stable(det.pixel_x, det.pixel_y):
                    continue                       # 아직 사람 손 위
                if is_duplicate(det):
                    continue                       # 이미 처리 중인 물체
                wx, wy = calibrator.pixel_to_world(det.pixel_x, det.pixel_y)
                if not calibrator.is_in_workspace(wx, wy):
                    logger.record_failure(det, "out_of_workspace")
                    continue
                task = spawner.spawn(det, (wx, wy))
                ok   = arm.execute_task(task)
                logger.record(task, ok, timings)
                spawner.remove(task.body_id)

            scene.step()

    ★ 동시성 주의: 로봇이 처리하는 동안에도 검출은 계속 들어옵니다.
      초기 버전은 큐에 쌓아두고 **순차 처리**로 단순하게 가세요.
      멀티스레드는 Day 8 이후 여유 있을 때만. 지금 하면 디버깅이 지옥입니다.

    ★ Day 4는 이 파이프라인에서 detector만 색상 기반 검출기로 바꿔
      좌표 변환 + 로봇 제어 연결만 먼저 검증합니다.
      (모델의 불확실성을 배제해야 실패 원인 범위가 좁아집니다)
    """


if __name__ == "__main__":
    main()
