"""
integration/pipeline.py — Qt 대시보드용 통합 파이프라인

실행:
    python -m gui

중요:
- GUI의 WebcamWorker가 카메라를 열고 프레임을 전달합니다.
- Pipeline은 카메라를 직접 열지 않습니다.
- SimWorker가 QThread 안에서 Pipeline을 생성하고 step_cycle()을 반복 호출합니다.
- PyBullet 관련 호출은 SimWorker 스레드 안에서만 일어나야 합니다.
"""

from __future__ import annotations

import time
from typing import Any, Callable

import pybullet as p

import config
from common.logger import SortLogger
from common.schema import SortResult
from integration.calibration import Calibrator
from integration.spawner import ObjectSpawner
from robot.scene import Scene
from tests.dummy_robot import DummyArmController
from tests.dummy_vision import DummyDetector


# 첫 물체가 검출된 뒤 추가 물체를 놓을 수 있도록 기다리는 시간
BATCH_COLLECTION_SEC = 5.0

# 전체 수거 후 연속 이 횟수만큼 검출이 없어야 다음 배치를 시작
REQUIRED_EMPTY_DETECTIONS = 3

# 최근 처리 좌표 재검출 방지 시간
SORTED_COOLDOWN_SEC = 1.0


class Pipeline:
    """웹캠 검출 → 전체 스폰 → 순차 수거를 담당하는 파이프라인."""

    STATE_WAITING = "waiting"
    STATE_COLLECTING = "collecting"
    STATE_WAITING_CLEAR = "waiting_clear"

    def __init__(
        self,
        use_dummy: bool = True,
        use_gui: bool = config.USE_GUI,
        log_fn: Callable[[str], None] = print,
    ) -> None:
        # Pipeline은 PySide6를 몰라야 하므로(CLI/테스트 단독 실행), 로그를 직접
        # print()하는 대신 이 콜백을 통해서만 내보냄. GUI에서는
        # SimWorker가 log_fn=self.log_message.emit을 넘겨서 로그 패널로 연결하고,
        # CLI에서는 기본값(print)이 그대로 콘솔에 찍힘.
        self._log = log_fn

        # -----------------------------------------------------
        # PyBullet 씬
        # -----------------------------------------------------
        self.scene = Scene(use_gui=use_gui)
        self.scene.build()

        # -----------------------------------------------------
        # 공통 모듈
        # -----------------------------------------------------
        self.calibrator = Calibrator()
        self.spawner = ObjectSpawner()
        self.logger = SortLogger()

        # -----------------------------------------------------
        # 검출기 / 로봇팔
        # -----------------------------------------------------
        if use_dummy:
            self.detector = DummyDetector(
                detect_probability=0.3,
            )
            self.arm = DummyArmController(
                success_rate=0.9,
                delay_sec=0.5,
            )
        else:
            from robot.arm_controller import ArmController
            from vision.detector import TrashDetector

            self.detector = TrashDetector()
            self.arm = ArmController(
                self.scene.robot_id,
                use_gui=use_gui,
            )

        self._use_dummy = use_dummy
        self._running = False

        # -----------------------------------------------------
        # 좌표 중복 처리 방지
        # -----------------------------------------------------
        self.processing_coords: list[tuple[float, float]] = []
        self.sorted_coords: list[tuple[float, float, float]] = []

        # -----------------------------------------------------
        # GUI 웹캠 bbox 표시용
        # -----------------------------------------------------
        self.last_detections: list[Any] = []

        # -----------------------------------------------------
        # YOLO 추론 간격 관리
        # -----------------------------------------------------
        self._frame_counter = 0
        self._last_t_detect_ms = 0.0

        # -----------------------------------------------------
        # 배치 상태
        # -----------------------------------------------------
        self._state = self.STATE_WAITING

        self._collection_started_at: float | None = None
        self._best_batch_detections: list[Any] = []
        self._best_batch_t_detect_ms = 0.0

        self._empty_detection_count = 0

        # GUI 옵션바에서 실시간 조절 가능 (SimWorker.apply_settings 참고).
        # 기본값은 모듈 상수(BATCH_COLLECTION_SEC/REQUIRED_EMPTY_DETECTIONS)로 시작.
        self.batch_collection_sec = BATCH_COLLECTION_SEC
        self.required_empty_detections = REQUIRED_EMPTY_DETECTIONS

    # =========================================================
    # 실행 제어
    # =========================================================

    def start(self) -> None:
        """Pipeline을 실행 상태로 변경합니다."""
        self._running = True

    def stop(self) -> None:
        """Pipeline을 중지 상태로 변경합니다."""
        self._running = False

    # =========================================================
    # 검출
    # =========================================================

    def _detect(
        self,
        frame,
    ) -> tuple[list[Any], float, bool]:
        """설정된 프레임 간격에 맞춰 detector.detect()를 호출합니다.

        반환:
            detections:
                현재 사용할 검출 결과

            t_detect_ms:
                가장 최근 실제 검출 시간

            did_detect:
                이번 step_cycle에서 실제로 검출기를 실행했는지 여부
        """

        self._frame_counter += 1

        detect_every_n_frames = max(
            1,
            int(config.DETECT_EVERY_N_FRAMES),
        )

        should_detect = (
            self._use_dummy
            or self._frame_counter % detect_every_n_frames == 0
        )

        if should_detect:
            started_at = time.time()

            detections = list(
                self.detector.detect(frame)
            )

            t_detect_ms = (
                time.time() - started_at
            ) * 1000

            self.last_detections = detections
            self._last_t_detect_ms = t_detect_ms

            return (
                detections,
                t_detect_ms,
                True,
            )

        return (
            self.last_detections,
            self._last_t_detect_ms,
            False,
        )

    # =========================================================
    # 배치 수집
    # =========================================================

    def _reset_collection(self) -> None:
        self._collection_started_at = None
        self._best_batch_detections = []
        self._best_batch_t_detect_ms = 0.0

    def _start_collection(
        self,
        detections: list[Any],
        t_detect_ms: float,
    ) -> None:
        """첫 검출을 기준으로 배치 수집을 시작합니다."""

        self._state = self.STATE_COLLECTING
        self._collection_started_at = time.monotonic()

        self._best_batch_detections = list(
            detections
        )

        self._best_batch_t_detect_ms = (
            t_detect_ms
        )

        self._log(
            "[pipeline] 배치 수집 시작: "
            f"현재 {len(detections)}개, "
            f"{self.batch_collection_sec:.1f}초 동안 "
            "물체를 놓아주세요"
        )

    def _update_collection(
        self,
        detections: list[Any],
        t_detect_ms: float,
        did_detect: bool,
    ) -> tuple[list[Any], float] | None:
        """배치 수집 상태를 갱신합니다.

        수집 시간 동안 가장 많은 물체가 검출된 프레임을 보관합니다.
        시간이 끝나면 확정된 검출 결과를 반환합니다.
        """

        if self._collection_started_at is None:
            self._reset_collection()
            self._state = self.STATE_WAITING
            return None

        # 실제 detector.detect() 결과만 후보 갱신에 사용
        if did_detect and detections:
            current_count = len(detections)
            best_count = len(
                self._best_batch_detections
            )

            if current_count > best_count:
                self._best_batch_detections = list(
                    detections
                )

                self._best_batch_t_detect_ms = (
                    t_detect_ms
                )

                self._log(
                    "[pipeline] 배치 물체 추가 감지: "
                    f"{current_count}개"
                )

            elif current_count == best_count:
                # 같은 개수라면 최신 bbox와 중심좌표 사용
                self._best_batch_detections = list(
                    detections
                )

                self._best_batch_t_detect_ms = (
                    t_detect_ms
                )

        elapsed = (
            time.monotonic()
            - self._collection_started_at
        )

        if elapsed < self.batch_collection_sec:
            return None

        confirmed = list(
            self._best_batch_detections
        )

        confirmed_t_detect_ms = (
            self._best_batch_t_detect_ms
        )

        self._reset_collection()

        if not confirmed:
            self._state = self.STATE_WAITING
            return None

        self._log(
            "[pipeline] 배치 확정: "
            f"{len(confirmed)}개"
        )

        return (
            confirmed,
            confirmed_t_detect_ms,
        )

    # =========================================================
    # 작업영역 밖 검출 제거
    # =========================================================

    def _filter_workspace_detections(
        self,
        detections: list[Any],
    ) -> list[Any]:
        """월드 좌표 기준으로 작업영역 밖 검출을 걸러냅니다."""

        return [
            det
            for det in detections
            if self.calibrator.is_in_workspace(
                *self.calibrator.pixel_to_world(
                    det.pixel_x, det.pixel_y
                )
            )
        ]

    # =========================================================
    # 중복 좌표 확인
    # =========================================================

    def is_duplicate(
        self,
        wx: float,
        wy: float,
    ) -> bool:
        """처리 중이거나 최근 처리한 좌표인지 확인합니다."""

        now = time.time()

        self.sorted_coords = [
            (cx, cy, timestamp)
            for cx, cy, timestamp
            in self.sorted_coords
            if (
                now - timestamp
                < SORTED_COOLDOWN_SEC
            )
        ]

        for cx, cy in self.processing_coords:
            distance = (
                (wx - cx) ** 2
                + (wy - cy) ** 2
            ) ** 0.5

            if (
                distance
                < config.DUPLICATE_RADIUS_M
            ):
                return True

        for cx, cy, _ in self.sorted_coords:
            distance = (
                (wx - cx) ** 2
                + (wy - cy) ** 2
            ) ** 0.5

            if (
                distance
                < config.DUPLICATE_RADIUS_M
            ):
                return True

        return False

    # =========================================================
    # 작업영역 비움 확인
    # =========================================================

    def _handle_waiting_clear(
        self,
        detections: list[Any],
        did_detect: bool,
    ) -> None:
        """실제 작업영역에서 물체가 모두 치워졌는지 확인합니다."""

        # 이전 검출 결과를 재사용한 프레임은 세지 않음
        if not did_detect:
            return

        if detections:
            self._empty_detection_count = 0
            return

        self._empty_detection_count += 1

        if (
            self._empty_detection_count
            < self.required_empty_detections
        ):
            return

        self._empty_detection_count = 0
        self.sorted_coords.clear()
        self._reset_collection()

        self._state = self.STATE_WAITING

        self._log(
            "[pipeline] 작업영역 비움 확인 "
            "— 다음 배치를 기다립니다"
        )

    # =========================================================
    # 배치 전체 스폰
    # =========================================================

    def _spawn_batch(
        self,
        detections: list[Any],
    ) -> list[dict[str, Any]]:
        """확정된 물체를 모두 먼저 PyBullet에 스폰합니다."""

        batch_items: list[
            dict[str, Any]
        ] = []

        # 화면 왼쪽 물체부터 순차 수거
        ordered_detections = sorted(
            detections,
            key=lambda det: det.pixel_x,
        )

        for det in ordered_detections:
            if not self._running:
                break

            started_at = time.time()

            wx, wy = (
                self.calibrator.pixel_to_world(
                    det.pixel_x,
                    det.pixel_y,
                )
            )

            t_transform_ms = (
                time.time() - started_at
            ) * 1000

            if not self.calibrator.is_in_workspace(
                wx,
                wy,
            ):
                self.logger.record_failure(
                    det,
                    "out_of_workspace",
                )

                self._log(
                    f"  제외: {det.category} "
                    f"({wx:.3f}, {wy:.3f}) "
                    "- 작업영역 밖"
                )

                continue

            if self.is_duplicate(wx, wy):
                self._log(
                    f"  제외: {det.category} "
                    f"({wx:.3f}, {wy:.3f}) "
                    "- 중복 좌표"
                )

                continue

            self.processing_coords.append(
                (wx, wy)
            )

            try:
                task = self.spawner.spawn(
                    det,
                    (wx, wy),
                )

                batch_items.append(
                    {
                        "task": task,
                        "detection": det,
                        "wx": wx,
                        "wy": wy,
                        "t_transform_ms": (
                            t_transform_ms
                        ),
                        "removed": False,
                    }
                )

                self._log(
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

                self._log(
                    f"  스폰 실패: "
                    f"{det.category} - {error}"
                )

        if batch_items:
            self._log(
                "[pipeline] 전체 스폰 완료: "
                f"{len(batch_items)}개"
            )

            # 렌더링에 모든 물체가 반영되도록
            for _ in range(10):
                p.stepSimulation()

        return batch_items

    # =========================================================
    # 로봇 실행
    # =========================================================

    def _execute_arm_task(
        self,
        task,
        on_step=None,
        on_state_change=None,
    ) -> bool:
        """실제/더미 로봇 인터페이스 차이를 처리합니다."""

        if self._use_dummy:
            # experiment/qt-dashboard의 DummyArmController는
            # on_state_change를 받지 않음
            return self.arm.execute_task(
                task,
                on_step=on_step,
            )

        return self.arm.execute_task(
            task,
            on_step=on_step,
            on_state_change=on_state_change,
        )

    def _remove_batch_item(
        self,
        item: dict[str, Any],
    ) -> None:
        """스폰된 물체와 처리 좌표를 안전하게 제거합니다."""

        task = item["task"]
        wx = item["wx"]
        wy = item["wy"]

        if not item["removed"]:
            try:
                self.spawner.remove(
                    task.body_id
                )

            except Exception as error:
                self._log(
                    "[pipeline] 객체 제거 오류: "
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

    def _execute_batch(
        self,
        batch_items: list[dict[str, Any]],
        t_detect_ms: float,
        on_step=None,
        on_state_change=None,
    ) -> int:
        """이미 스폰된 물체를 하나씩 순차 수거합니다."""

        completed = 0

        try:
            for item in batch_items:
                if not self._running:
                    break

                task = item["task"]
                det = item["detection"]
                wx = item["wx"]
                wy = item["wy"]

                self._log(
                    f"  수거 시작: "
                    f"{det.category} "
                    f"({wx:.3f}, {wy:.3f})"
                )

                try:
                    started_at = time.time()

                    ok = self._execute_arm_task(
                        task,
                        on_step=on_step,
                        on_state_change=(
                            on_state_change
                        ),
                    )

                    t_execute_ms = (
                        time.time() - started_at
                    ) * 1000

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
                        t_detect_ms=t_detect_ms,
                        t_transform_ms=(
                            item["t_transform_ms"]
                        ),
                        t_execute_ms=t_execute_ms,
                    )

                    self.logger.record(result)

                    self.sorted_coords.append(
                        (
                            wx,
                            wy,
                            time.time(),
                        )
                    )

                    completed += 1

                    result_text = "OK" if ok else f"FAIL ({fail_reason})"
                    self._log(
                        f"  {det.category} "
                        f"-> {task.target_bin} | "
                        f"{result_text}"
                    )

                except Exception as error:
                    self._log(
                        f"  처리 오류: "
                        f"{det.category} - {error}"
                    )

                    self.logger.record_failure(
                        det,
                        "execution_error",
                    )

                    completed += 1

                finally:
                    # 현재 작업이 끝난 물체만 제거
                    self._remove_batch_item(item)
                    p.stepSimulation()

        finally:
            # Stop 또는 예외 발생 시 남은 물체 정리
            for item in batch_items:
                if not item["removed"]:
                    self._remove_batch_item(item)

        return completed

    # =========================================================
    # GUI/CLI 공통 한 사이클
    # =========================================================

    def step_cycle(
        self,
        frame=None,
        on_step=None,
        on_state_change=None,
    ) -> int:
        """SimWorker가 반복 호출하는 한 사이클입니다."""

        if not self._running:
            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)
            return 0

        # 실제 모드인데 프레임이 없으면 검출하지 않음
        if (
            not self._use_dummy
            and frame is None
        ):
            self.last_detections = []

            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)

            return 0

        detections, t_detect_ms, did_detect = (
            self._detect(frame)
        )

        # self.last_detections(GUI bbox 표시용)는 원본 그대로 둠 — 디버깅용으로
        # 뭐가 검출됐는지 다 보여줘야 함. 아래 state 판정에 쓰는 detections만
        # 걸러냄 — 워크스페이스 밖 배경 물체가 섞이면 WAITING의 "첫 물체 검출"이
        # 오탐되고 WAITING_CLEAR의 "작업영역 비움" 판정이 영원히 안 남.
        detections = self._filter_workspace_detections(detections)

        # -----------------------------------------------------
        # 실제 물체 제거 대기
        # -----------------------------------------------------
        if (
            self._state
            == self.STATE_WAITING_CLEAR
        ):
            self._handle_waiting_clear(
                detections,
                did_detect,
            )

            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)
            return 0

        # -----------------------------------------------------
        # 첫 물체 검출 대기
        # -----------------------------------------------------
        if self._state == self.STATE_WAITING:
            if did_detect and detections:
                self._start_collection(
                    detections,
                    t_detect_ms,
                )

            p.stepSimulation()
            time.sleep(config.SIM_TIMESTEP)
            return 0

        # -----------------------------------------------------
        # 추가 물체 수집
        # -----------------------------------------------------
        if (
            self._state
            == self.STATE_COLLECTING
        ):
            confirmed = self._update_collection(
                detections,
                t_detect_ms,
                did_detect,
            )

            if confirmed is None:
                p.stepSimulation()
                time.sleep(config.SIM_TIMESTEP)
                return 0

            (
                confirmed_detections,
                confirmed_t_detect_ms,
            ) = confirmed

            batch_items = self._spawn_batch(
                confirmed_detections
            )

            if not batch_items:
                self._state = self.STATE_WAITING

                p.stepSimulation()
                time.sleep(config.SIM_TIMESTEP)
                return 0

            completed = self._execute_batch(
                batch_items=batch_items,
                t_detect_ms=(
                    confirmed_t_detect_ms
                ),
                on_step=on_step,
                on_state_change=(
                    on_state_change
                ),
            )

            self._state = (
                self.STATE_WAITING_CLEAR
            )

            self._empty_detection_count = 0

            self._log(
                "[pipeline] 배치 수거 완료 "
                "— 실제 물체를 작업영역에서 "
                "치워주세요"
            )

            p.stepSimulation()

            return completed

        # 알 수 없는 상태면 초기화
        self._reset_collection()
        self._state = self.STATE_WAITING

        p.stepSimulation()
        time.sleep(config.SIM_TIMESTEP)
        return 0

    # =========================================================
    # CLI 호환용
    # =========================================================

    def run(
        self,
        max_cycles: int | None = 20,
        camera=None,
    ) -> None:
        """CLI 호환용 실행 함수입니다.

        python -m gui 실행에서는 사용되지 않습니다.
        """

        print(
            "[pipeline] 시작 "
            f"(max_cycles={max_cycles})"
        )

        self.start()

        cycle = 0

        while (
            self._running
            and (
                max_cycles is None
                or cycle < max_cycles
            )
        ):
            frame = (
                camera.read()
                if camera is not None
                else None
            )

            cycle += self.step_cycle(frame)

        self.stop()

        summary = self.logger.summary()

        print("\n[pipeline] 완료!")

        print(
            f"  총 {summary['total']}회, "
            f"성공률 "
            f"{summary['success_rate']:.0%}"
        )

        if summary["fail_reasons"]:
            print(
                "  실패 사유: "
                f"{summary['fail_reasons']}"
            )

        print(f"  CSV: {self.logger.path}")

    # =========================================================
    # 종료
    # =========================================================

    def shutdown(self) -> None:
        self.stop()
        self.scene.disconnect()


def main() -> None:
    """CLI 단독 실행용입니다.

    발표용 GUI 실행은 python -m gui를 사용합니다.
    """

    use_dummy = False

    pipeline = Pipeline(
        use_dummy=use_dummy,
    )

    camera = None

    if not use_dummy:
        from vision.camera import Camera

        camera = Camera()
        camera.open()

    try:
        pipeline.run(
            max_cycles=20,
            camera=camera,
        )

    except KeyboardInterrupt:
        print("\n[pipeline] 사용자 중단")

    finally:
        if camera is not None:
            camera.close()

        pipeline.shutdown()


if __name__ == "__main__":
    main()