"""
integration/pipeline.py — 메인 진입점
담당: 진선우 | Day 3 ~ Day 4

실행::

    python -m integration.pipeline

전체 시스템을 이 파일 하나로 돌립니다. Day 4 통합의 무대입니다.

★ [GUI 연동] gui/sim_worker.py가 이 Pipeline을 QThread 안에서 소유하고
  start()/stop()/step_cycle()로 제어합니다. Pipeline은 PySide6를 전혀 몰라야
  CLI 단독 실행도, GUI 없는 테스트도 계속 가능합니다.

★ Pipeline은 카메라 장치를 직접 열지 않습니다. GUI에서는 웹캠을 이미
  gui/webcam_worker.py가 미리보기용으로 열어두고 있어서, Pipeline까지 같은
  장치를 또 열면 충돌합니다. 대신 frame을 step_cycle()/run()의 인자로
  받아서 처리만 합니다 — 카메라를 여는 쪽(CLI의 main(), 또는 GUI)이 각자
  책임지고 프레임을 넘겨주세요.
"""

from __future__ import annotations
import time

import pybullet as p

import config
from common.logger import SortLogger
from common.schema import SortResult
from integration.calibration import Calibrator
from integration.spawner import ObjectSpawner
from robot.scene import Scene
from tests.dummy_robot import DummyArmController
from tests.dummy_vision import DummyDetector


class Pipeline:
    """전체 시스템 메인 루프."""

    def __init__(self, use_dummy: bool = True, use_gui: bool = config.USE_GUI) -> None:
        # --- 씬 구성 (바닥 + 로봇 + 수거함) ---
        self.scene = Scene(use_gui=use_gui)
        self.scene.build()

        # --- 모듈 생성 ---
        self.calibrator = Calibrator()
        self.spawner = ObjectSpawner()
        self.logger = SortLogger()

        if use_dummy:
            self.detector = DummyDetector(detect_probability=0.3)
            self.arm = DummyArmController(success_rate=0.9, delay_sec=0.5)
        else:
            from robot.arm_controller import ArmController
            from vision.detector import TrashDetector

            self.detector = TrashDetector()
            self.arm = ArmController(self.scene.robot_id, use_gui=use_gui)

        self._use_dummy = use_dummy

        # 중복 처리 방지용
        self.processing_coords: list[tuple[float, float]] = []
        self._running = False

        # 가장 최근 step_cycle() 호출에서 검출된 목록. GUI가 웹캠 미리보기 위에
        # bbox를 겹쳐 그릴 때 재사용 — YOLO를 다시 돌리지 않기 위함.
        self.last_detections = []

    def start(self) -> None:
        """루프 실행 플래그를 켭니다. GUI의 Start 버튼이 호출."""
        self._running = True

    def stop(self) -> None:
        """루프 실행 플래그를 끕니다. GUI의 Stop 버튼이 호출.

        ★ 지금 진행 중인 arm.execute_task() 한 번(최대 MOVE_TIMEOUT_SEC마다
          체크)은 즉시 끊기지 않습니다. step_cycle()의 detection 루프 시작
          지점에서만 self._running을 확인하기 때문입니다."""
        self._running = False

    def is_duplicate(self, wx: float, wy: float) -> bool:
        """이미 처리 중인 좌표 근처인지 확인."""
        for cx, cy in self.processing_coords:
            dist = ((wx - cx) ** 2 + (wy - cy) ** 2) ** 0.5
            if dist < config.DUPLICATE_RADIUS_M:
                return True
        return False

    def step_cycle(self, frame=None) -> int:
        """검출 -> 처리 한 사이클. CLI(run())와 GUI(SimWorker.run()) 양쪽이
        이 메서드를 반복 호출합니다.

        frame: BGR numpy 프레임. 더미 모드에서는 detector가 무시하므로 None이어도 됨.
        반환값: 이번 호출에서 완료(성공/실패 로깅까지 끝)된 분류 작업 수.
        """
        if not self._use_dummy and frame is None:
            # 실제 모드인데 프레임이 없는 경우(웹캠 아직 미연결, read() 실패 등).
            # frame=None을 그대로 넘기면 ultralytics가 번들 샘플 이미지로 조용히
            # 대체해 가짜 검출을 만들어내므로, 아예 검출을 건너뜀.
            self.last_detections = []
            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)
            return 0

        t0 = time.time()
        detections = self.detector.detect(frame)
        t_detect = (time.time() - t0) * 1000
        self.last_detections = detections

        if not detections:
            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)
            return 0

        completed = 0
        for det in detections:
            if not self._running:
                break

            # ① 좌표 변환
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

            # ② 스폰 -> ③ 로봇 실행 -> ④ 로깅. arm.execute_task()가 예외를 던져도
            # processing_coords/스폰된 body가 영구히 안 남게 finally로 정리.
            self.processing_coords.append((wx, wy))
            task = None
            try:
                task = self.spawner.spawn(det, (wx, wy))

                t2 = time.time()
                ok = self.arm.execute_task(task)
                t_execute = (time.time() - t2) * 1000

                result = SortResult(
                    task=task,
                    success=ok,
                    fail_reason=None if ok else "timeout",
                    t_detect_ms=t_detect,
                    t_transform_ms=t_transform,
                    t_execute_ms=t_execute,
                )
                self.logger.record(result)

                completed += 1
                print(f"  {det.category} -> {task.target_bin} | {'OK' if ok else 'FAIL'}")
            finally:
                if task is not None:
                    self.spawner.remove(task.body_id)
                self.processing_coords.remove((wx, wy))

        p.stepSimulation()
        return completed

    def run(self, max_cycles: int | None = 20, camera=None) -> None:
        """CLI용 루프. max_cycles만큼 처리 후 종료 (None이면 stop() 전까지 무한 루프).

        camera: vision.camera.Camera처럼 .read() -> frame을 주는 객체. None이면
          더미 모드로 간주하고 매 사이클 frame=None을 넘깁니다.
        """
        print(f"[pipeline] 시작 (max_cycles={max_cycles})")
        self.start()
        cycle = 0

        while self._running and (max_cycles is None or cycle < max_cycles):
            frame = camera.read() if camera is not None else None
            cycle += self.step_cycle(frame)

        self.stop()

        # 결과 요약
        summary = self.logger.summary()
        print(f"\n[pipeline] 완료!")
        print(f"  총 {summary['total']}회, 성공률 {summary['success_rate']:.0%}")
        if summary["fail_reasons"]:
            print(f"  실패 사유: {summary['fail_reasons']}")
        print(f"  CSV: {self.logger.path}")

    def shutdown(self) -> None:
        self.scene.disconnect()


def main() -> None:
    use_dummy = True  # 실제 웹캠+YOLO+로봇으로 돌리려면 False (모델/캘리브레이션 준비 필요)

    pipeline = Pipeline(use_dummy=use_dummy)
    camera = None
    if not use_dummy:
        from vision.camera import Camera

        camera = Camera()
        camera.open()

    try:
        pipeline.run(max_cycles=20, camera=camera)
    finally:
        if camera is not None:
            camera.close()
        pipeline.shutdown()


if __name__ == "__main__":
    main()
