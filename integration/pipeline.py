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

import cv2
import pybullet as p
import pybullet_data

import config
from common.logger import SortLogger
from common.schema import SortResult
from integration.calibration import Calibrator
from integration.spawner import ObjectSpawner
from robot.scene import Scene
from tests.dummy_robot import DummyArmController
from tests.dummy_vision import DummyDetector

SORTED_COOLDOWN_SEC = 1.0
"""한 번 분류한 좌표는 이 시간 동안만 재검출을 무시합니다 (영구 아님).
같은 물체가 계속 웹캠에 잡혀서 매 프레임 재처리되는 건 막으면서도, 그 자리에
사람이 다른 물체를 새로 놓으면 이 시간이 지난 뒤엔 다시 정상 인식되게 하기
위함. 너무 짧으면 같은 프레임 연속 재검출을 못 거르고(원래 문제), 너무 길면
물체를 빨리 교체했을 때 한동안 무시됨.
★ 5.0 -> 1.0으로 낮춤. 실제 재처리 간격은 이 값 단독이 아니라
  "로봇팔 작업시간(보통 1.5~4초) + 이 값"이라, 원래 막으려던 버그(초당
  수십 번 재처리)는 작업시간 자체가 이미 1초 이상이라 재발 위험이 낮음.
  줄인 이유: 같은 자리에 물체를 새로 놓아도 5초 쿨다운 동안 반응이
  없는 것처럼 보이는 문제(=오검출이 아니라 의도된 dedup) 완화."""


class Pipeline:
    """전체 시스템 메인 루프."""

    def __init__(self, use_dummy: bool = True, use_gui: bool = config.USE_GUI) -> None:
        # --- 씬 구성 (바닥 + 로봇 + 수거함) ---
        self.scene = Scene(use_gui=use_gui)
        self.scene.build()

        # -----------------------------------------------------
        # 공통 모듈
        # -----------------------------------------------------
        self.calibrator = Calibrator()
        self.spawner = ObjectSpawner()
        self.logger = SortLogger()

        # -----------------------------------------------------
        # 더미 모드
        # -----------------------------------------------------
        if use_dummy:
            mode = p.GUI if config.USE_GUI else p.DIRECT
            self.physics_client = p.connect(mode)

            p.setAdditionalSearchPath(
                pybullet_data.getDataPath()
            )
            p.setGravity(0, 0, -9.81)
            p.loadURDF("plane.urdf")

            self.camera = None

            self.detector = DummyDetector(
                detect_probability=0.3
            )

            self.arm = DummyArmController(
                success_rate=0.9,
                delay_sec=0.5,
            )

        # -----------------------------------------------------
        # 실제 모드
        # -----------------------------------------------------
        else:
            from robot.arm_controller import ArmController
            from vision.detector import TrashDetector

            self.detector = TrashDetector()
            self.arm = ArmController(self.scene.robot_id, use_gui=use_gui)

        self._use_dummy = use_dummy

        # 중복 처리 방지용 (동시 처리 중인 좌표. task 끝나면 바로 빠짐)
        self.processing_coords: list[tuple[float, float]] = []
        # 이미 분류를 마친(성공/실패 무관) 좌표 + 처리 시각. SORTED_COOLDOWN_SEC
        # 동안만 재검출을 무시함 (영구 아님 — 그 자리에 새 물체를 놓으면 쿨다운
        # 지난 뒤 다시 인식됨). ★ 이게 없으면 웹캠이 보는 실제 물체가 안
        # 치워지는 한 매 프레임 같은 좌표가 계속 "새 검출"로 들어와서 무한정
        # 재처리됨 (대시보드에서 "분류 완료"가 초당 수십 번씩 뜨던 원인).
        # Reset하면 Pipeline을 새로 만드니 자연히 비워짐.
        self.sorted_coords: list[tuple[float, float, float]] = []
        self._running = False

        # 가장 최근 step_cycle() 호출에서 검출된 목록. GUI가 웹캠 미리보기 위에
        # bbox를 겹쳐 그릴 때 재사용 — YOLO를 다시 돌리지 않기 위함.
        self.last_detections = []

        # config.DETECT_EVERY_N_FRAMES 프레임마다 한 번만 YOLO를 돌리기 위한 카운터.
        self._frame_counter = 0
        # 가장 최근 "실제로" detect()를 호출했을 때 걸린 시간(ms). 건너뛴 사이클엔
        # 이 값을 그대로 재사용해서 로깅함 — 0.0으로 찍으면 실험 3(지연시간 분해)
        # CSV의 평균 t_detect_ms가 1/N만큼 희석돼 실제보다 낮게 왜곡되기 때문.
        self._last_t_detect_ms = 0.0

    def start(self) -> None:
        """루프 실행 플래그를 켭니다. GUI의 Start 버튼이 호출."""
        self._running = True

    def stop(self) -> None:
        """루프 실행 플래그를 끕니다. GUI의 Stop 버튼이 호출.

        ★ 지금 진행 중인 arm.execute_task() 한 번(최대 MOVE_TIMEOUT_SEC마다
          체크)은 즉시 끊기지 않습니다. step_cycle()의 detection 루프 시작
          지점에서만 self._running을 확인하기 때문입니다."""
        self._running = False

    # =========================================================
    # 외부 제어
    # =========================================================

    def start(self) -> None:
        """파이프라인을 실행 상태로 변경."""
        self._running = True

    def stop(self) -> None:
        """현재 배치가 끝나는 지점에서 실행을 중지."""
        self._running = False

    # =========================================================
    # 중복 좌표 확인
    # =========================================================

    def is_duplicate(self, wx: float, wy: float) -> bool:
        """이미 처리 중이거나(동시성), 쿨다운이 안 지난 좌표 근처인지 확인."""
        now = time.time()
        # 만료된 항목은 여기서 같이 청소 (호출될 때마다 자연히 정리되므로
        # sorted_coords가 무한정 커지지 않음).
        self.sorted_coords = [
            (cx, cy, t) for cx, cy, t in self.sorted_coords if now - t < SORTED_COOLDOWN_SEC
        ]

        for cx, cy in self.processing_coords:
            distance = (
                (wx - cx) ** 2
                + (wy - cy) ** 2
            ) ** 0.5

            if distance < config.DUPLICATE_RADIUS_M:
                return True

        return False

    # =========================================================
    # 웹캠 화면 표시
    # =========================================================

    def draw_detections(self, frame, detections, message=None):
        """웹캠 화면에 바운딩 박스와 상태 메시지를 표시."""

        if frame is None:
            return

        for det in detections:
            x1, y1, x2, y2 = det.bbox

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

            label = (
                f"{det.category} "
                f"{det.confidence:.2f}"
            )

            cv2.putText(
                frame,
                label,
                (x1, max(y1 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        if message:
            cv2.putText(
                frame,
                message,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        cv2.imshow("camera", frame)

    # =========================================================
    # 실제 물체가 치워질 때까지 대기
    # =========================================================

    def is_same_batch(
        self,
        previous,
        current,
        tolerance_px: int = 12,
    ) -> bool:
        """검출 개수, 클래스, 위치가 거의 같은지 확인."""

        if len(previous) != len(current):
            return False

        previous_sorted = sorted(
            previous,
            key=lambda det: (det.pixel_x, det.pixel_y),
        )

        current_sorted = sorted(
            current,
            key=lambda det: (det.pixel_x, det.pixel_y),
        )

        for prev_det, curr_det in zip(
            previous_sorted,
            current_sorted,
        ):
            if prev_det.category != curr_det.category:
                return False

            dx = prev_det.pixel_x - curr_det.pixel_x
            dy = prev_det.pixel_y - curr_det.pixel_y

            distance = (dx * dx + dy * dy) ** 0.5

            if distance > tolerance_px:
                return False

        return True


    def wait_for_stable_batch(
        self,
        settle_seconds: float = 1.5,
    ):
        """물체 개수와 위치가 settle_seconds 동안 유지될 때 배치를 확정."""

        candidate_detections = []
        last_change_time = time.time()
        last_t_detect = 0.0

        print("[pipeline] 물체를 모두 놓아주세요.")

        while self._running:
            if self.camera is not None:
                frame = self.camera.read()

                if frame is None:
                    p.stepSimulation()
                    time.sleep(config.SIM_TIMESTEP)
                    continue
            else:
                frame = None

            t0 = time.time()
            detections = self.detector.detect(frame)
            last_t_detect = (time.time() - t0) * 1000

            if not self.is_same_batch(
                candidate_detections,
                detections,
            ):
                candidate_detections = list(detections)
                last_change_time = time.time()

            stable_duration = time.time() - last_change_time
            remaining = max(0.0, settle_seconds - stable_duration)

            if candidate_detections:
                message = (
                    f"Detected: {len(candidate_detections)} | "
                    f"Start in: {remaining:.1f}s"
                )
            else:
                message = "Place all objects"

            self.draw_detections(
                frame,
                detections,
                message,
            )

            if frame is not None:
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    self.stop()
                    return [], last_t_detect

            if (
                candidate_detections
                and stable_duration >= settle_seconds
            ):
                print(
                    f"[pipeline] 배치 확정: "
                    f"{len(candidate_detections)}개"
                )

                return (
                    list(candidate_detections),
                    last_t_detect,
                )

            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)

        return [], last_t_detect

    def wait_until_workspace_clear(
        self,
        required_empty_frames: int = 10,
    ) -> None:
        """작업영역에서 실제 물체가 치워질 때까지 기다립니다.

        연속 required_empty_frames 프레임 동안 검출 결과가
        없으면 다음 배치를 시작합니다.
        """

        if self.camera is None:
            # 더미 모드에서는 실제 물체 제거를 확인할 수 없음
            return

        print(
            "\n[pipeline] 배치 수거 완료."
            " 실제 물체를 작업영역에서 치워주세요."
        )

        empty_count = 0

        while (
            self._running
            and empty_count < required_empty_frames
        ):
            frame = self.camera.read()

            if frame is None:
                empty_count = 0
                time.sleep(config.SIM_TIMESTEP)
                continue

            detections = self.detector.detect(frame)

            if detections:
                empty_count = 0
            else:
                empty_count += 1

            message = (
                "Remove objects "
                f"{empty_count}/{required_empty_frames}"
            )

            self.draw_detections(
                frame,
                detections,
                message,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                self.stop()
                return

            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)

        if self._running:
            print(
                "[pipeline] 작업영역 비움 확인."
                " 다음 배치를 검출합니다."
            )

    # =========================================================
    # 메인 루프
    # =========================================================

    def run(
        self,
        max_cycles: int | None = 50,
    ) -> None:
        """여러 물체를 한 번에 스폰하고 순차적으로 수거합니다.

        max_cycles:
            처리할 전체 물체 수.
            None이면 stop() 호출 전까지 계속 실행합니다.
        """

        print(
            f"[pipeline] 시작 "
            f"(max_cycles={max_cycles})"
        )

        self.start()

        cycle = 0

        while self._running:
            if (
                max_cycles is not None
                and cycle >= max_cycles
            ):
                break

            # =================================================
            # 1. 웹캠 프레임 읽기
            # =================================================

            detections, t_detect = self.wait_for_stable_batch(
                settle_seconds=1.5,
            )

            if not self._running:
                break
        for cx, cy, _ in self.sorted_coords:
            dist = ((wx - cx) ** 2 + (wy - cy) ** 2) ** 0.5
            if dist < config.DUPLICATE_RADIUS_M:
                return True
        return False

    def step_cycle(self, frame=None, on_step=None, on_state_change=None) -> int:
        """검출 -> 처리 한 사이클. CLI(run())와 GUI(SimWorker.run()) 양쪽이
        이 메서드를 반복 호출합니다.

        frame: BGR numpy 프레임. 더미 모드에서는 detector가 무시하므로 None이어도 됨.
        on_step: 로봇팔 이동 중간중간 호출되는 콜백 (arm.execute_task()로 그대로
          전달됨). GUI가 이동 과정을 화면에 스트리밍할 때 씀 — CLI 등 안 쓰는
          쪽은 None으로 두면 됨.
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

        # ★ 1순위 최적화: YOLO 추론이 CPU에서 프레임당 수십 ms가 걸려 이게 곧
        #   대시보드 fps의 상한이 됨. config.DETECT_EVERY_N_FRAMES마다 한 번만
        #   실제로 추론하고, 나머지 프레임은 last_detections(직전 결과)를 그대로
        #   재사용 — 물체가 그 사이 몇 cm 이상 움직이진 않으므로 정확도 손해는
        #   거의 없이 추론 횟수를 1/N로 줄임. 더미 모드는 detect()가 사실상
        #   공짜라 건너뛸 이유가 없으므로 매 프레임 그대로 돌림.
        self._frame_counter += 1
        should_detect = self._use_dummy or self._frame_counter % config.DETECT_EVERY_N_FRAMES == 0

        if should_detect:
            t0 = time.time()
            detections = self.detector.detect(frame)
            t_detect = (time.time() - t0) * 1000
            self.last_detections = detections
            self._last_t_detect_ms = t_detect
        else:
            detections = self.last_detections
            t_detect = self._last_t_detect_ms  # 직전 실측값 재사용 (0.0으로 찍으면 로그 왜곡됨)

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

            print(
                f"\n[pipeline] "
                f"{len(detections)}개 물체 검출"
            )

            # 화면 왼쪽 물체부터 처리
            ordered_detections = sorted(
                detections,
                key=lambda det: det.pixel_x,
            )

            # 각 원소:
            # {
            #   "task": SortTask,
            #   "detection": Detection,
            #   "wx": float,
            #   "wy": float,
            #   "t_transform": float,
            #   "removed": bool,
            # }
            batch_tasks = []

            # =================================================
            # 3. 검출된 모든 물체를 먼저 스폰
            # =================================================

            for det in ordered_detections:
                if not self._running:
                    break

                # 최대 처리 개수를 넘지 않도록 제한
                if (
                    max_cycles is not None
                    and cycle + len(batch_tasks)
                    >= max_cycles
                ):
                    break

                t1 = time.time()

                wx, wy = self.calibrator.pixel_to_world(
                    det.pixel_x,
                    det.pixel_y,
                )

                t_transform = (
                    time.time() - t1
                ) * 1000

                # 작업 영역 확인
                if not self.calibrator.is_in_workspace(
                    wx,
                    wy,
                ):
                    self.logger.record_failure(
                        det,
                        "out_of_workspace",
                    )

                    print(
                        f"  제외: {det.category} "
                        f"({wx:.3f}, {wy:.3f}) "
                        "- 작업영역 밖"
                    )

                    continue

                # 동일 배치 내 중복 좌표 확인
                if self.is_duplicate(wx, wy):
                    print(
                        f"  제외: {det.category} "
                        f"({wx:.3f}, {wy:.3f}) "
                        "- 중복 좌표"
                    )

                    continue

                # 먼저 좌표를 등록해야 같은 배치의 다음 검출과
                # 중복 여부를 비교할 수 있음
                self.processing_coords.append(
                    (wx, wy)
                )

                try:
                    task = self.spawner.spawn(
                        det,
                        (wx, wy),
                    )

                    batch_tasks.append(
                        {
                            "task": task,
                            "detection": det,
                            "wx": wx,
                            "wy": wy,
                            "t_transform": t_transform,
                            "removed": False,
                        }
                    )

                    print(
                        f"  스폰 완료: "
                        f"{det.category} "
                        f"({wx:.3f}, {wy:.3f}) "
                        f"body_id={task.body_id}"
                    )

                except Exception as error:
                    if (
                        (wx, wy)
                        in self.processing_coords
                    ):
                        self.processing_coords.remove(
                            (wx, wy)
                        )

                    print(
                        f"  스폰 실패: "
                        f"{det.category} - {error}"
                    )

            # 유효한 물체가 하나도 없으면 다시 검출
            if not batch_tasks:
                p.stepSimulation()
                time.sleep(config.SIM_TIMESTEP)
                continue

            print(
                f"[pipeline] "
                f"{len(batch_tasks)}개 물체 "
                "전체 스폰 완료"
            )

            # 모든 객체가 PyBullet 화면에 나타나도록
            # 잠시 물리 시뮬레이션 진행
            for _ in range(30):
                p.stepSimulation()

                if config.USE_GUI:
                    time.sleep(config.SIM_TIMESTEP)

            # =================================================
            # 4. 이미 스폰된 물체를 하나씩 순차 수거
            # =================================================

            for item in batch_tasks:
                if not self._running:
                    break

                if (
                    max_cycles is not None
                    and cycle >= max_cycles
                ):
                    break

                task = item["task"]
                det = item["detection"]
                wx = item["wx"]
                wy = item["wy"]
                t_transform = item["t_transform"]

                print(
                    f"\n  수거 시작: "
                    f"{det.category} "
                    f"({wx:.3f}, {wy:.3f})"
                )

                try:
                    t2 = time.time()

                    ok = self.arm.execute_task(task)

                    t_execute = (
                        time.time() - t2
                    ) * 1000

                    # 실제 ArmController에 last_result가 있으면
                    # 구체적인 실패 사유 사용
                    arm_last_result = getattr(
                        self.arm,
                        "last_result",
                        None,
                    )

                    if ok:
                        fail_reason = None
                    elif arm_last_result is not None:
                        fail_reason = (
                            arm_last_result.fail_reason
                        )
                    else:
                        fail_reason = "timeout"

                    result = SortResult(
                        task=task,
                        success=ok,
                        fail_reason=fail_reason,
                        t_detect_ms=t_detect,
                        t_transform_ms=t_transform,
                        t_execute_ms=t_execute,
                    )

                    self.logger.record(result)

                    cycle += 1

                    print(
                        f"  [{cycle}] "
                        f"{det.category} "
                        f"-> {task.target_bin} | "
                        f"{'OK' if ok else 'FAIL'}"
                    )

                except Exception as error:
                    print(
                        f"  처리 오류: "
                        f"{det.category} - {error}"
                    )

                    self.logger.record_failure(
                        det,
                        "execution_error",
                    )

                    cycle += 1

                finally:
                    # 현재 수거가 끝난 가상 객체만 제거
                    if not item["removed"]:
                        try:
                            self.spawner.remove(
                                task.body_id
                            )
                        except Exception as error:
                            print(
                                f"  객체 제거 오류: "
                                f"body_id={task.body_id}, "
                                f"{error}"
                            )

                        item["removed"] = True

                    if (
                        (wx, wy)
                        in self.processing_coords
                    ):
                        self.processing_coords.remove(
                            (wx, wy)
                        )

                # 한 물체를 제거한 뒤 시뮬레이션 갱신
                p.stepSimulation()

            # =================================================
            # 5. 중간 종료 시 아직 남은 객체 정리
            # =================================================

            for item in batch_tasks:
                task = item["task"]
                wx = item["wx"]
                wy = item["wy"]

                if not item["removed"]:
                    try:
                        self.spawner.remove(
                            task.body_id
                        )
                    except Exception as error:
                        print(
                            f"  잔여 객체 제거 오류: "
                            f"body_id={task.body_id}, "
                            f"{error}"
                        )

                    item["removed"] = True

                if (
                    (wx, wy)
                    in self.processing_coords
                ):
                    self.processing_coords.remove(
                        (wx, wy)
                    )

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

            # =================================================
            # 6. 전체 배치 수거 후 실제 물체 제거 대기
            # =================================================

            if (
                self._running
                and (
                    max_cycles is None
                    or cycle < max_cycles
                )
            ):
                self.wait_until_workspace_clear(
                    required_empty_frames=10
                )

        self.stop()

        # =====================================================
        # 결과 요약
        # =====================================================

        summary = self.logger.summary()

        print("\n[pipeline] 완료!")

        print(
            f"  총 {summary['total']}회, "
            f"성공률 "
            f"{summary['success_rate']:.0%}"
        )

        if summary["fail_reasons"]:
            print(
                f"  실패 사유: "
                f"{summary['fail_reasons']}"
            )

        print(f"  CSV: {self.logger.path}")

    # =========================================================
    # 종료
    # =========================================================

    def shutdown(self) -> None:
        """카메라, OpenCV 창, PyBullet 연결 종료."""

        self.stop()

        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass

        cv2.destroyAllWindows()

        if p.isConnected():
            p.disconnect()


def main() -> None:
    # True: 더미 모드
    # False: 실제 웹캠 + YOLO + 로봇팔
    pipeline = Pipeline(use_dummy=False)

    try:
        pipeline.run(max_cycles=20)

    except KeyboardInterrupt:
        print("\n[pipeline] 사용자 중단")

    finally:
        if camera is not None:
            camera.close()
        pipeline.shutdown()


if __name__ == "__main__":
    main()