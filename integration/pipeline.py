"""
integration/pipeline.py — 메인 진입점
담당: 진선우 | Day 3 ~ Day 4

실행::

    python -m integration.pipeline

전체 시스템을 이 파일 하나로 돌립니다.
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
from tests.dummy_robot import DummyArmController
from tests.dummy_vision import DummyDetector


class Pipeline:
    """전체 시스템 메인 루프."""

    def __init__(self, use_dummy: bool = True) -> None:
        self.use_dummy = use_dummy

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
            from vision.camera import Camera
            from vision.detector import TrashDetector
            from robot.scene import Scene
            from robot.arm_controller import ArmController

            self.scene = Scene()
            self.scene.build()

            self.camera = Camera()
            self.camera.open()

            self.detector = TrashDetector()
            self.arm = ArmController(self.scene.robot_id)

        # 현재 배치에 등록된 좌표
        self.processing_coords: list[tuple[float, float]] = []

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
        """현재 배치에 이미 등록된 좌표 근처인지 확인."""

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

            if not detections:
                p.stepSimulation()
                time.sleep(config.SIM_TIMESTEP)
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

            p.stepSimulation()

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
        pipeline.shutdown()


if __name__ == "__main__":
    main()