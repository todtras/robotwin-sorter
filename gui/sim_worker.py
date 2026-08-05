"""
gui/sim_worker.py — integration.pipeline.Pipeline을 별도 스레드에서 구동하는 워커

Qt 처음이면 알아둘 개념:
- QThread: 별도 스레드에서 코드를 실행하게 해주는 클래스. `self.start()`를 호출하면
  Qt가 새 스레드를 만들고 그 안에서 `run()` 메서드를 자동으로 호출해줍니다.
  (run()을 직접 호출하면 그냥 지금 스레드에서 실행돼버리니 절대 직접 부르지 마세요.)
- Signal: "이런 일이 일어났다"를 다른 객체(주로 다른 스레드에 있는 위젯)에게 알리는
  통로입니다. `self.frame_ready.emit(image)`처럼 emit()으로 신호를 보내면, 이
  시그널에 connect() 해둔 함수(슬롯)가 Qt에 의해 안전하게 GUI 스레드에서 호출됩니다.
  스레드 간 데이터 전달은 항상 이 시그널을 통해서만 하세요.

★★★ 중요: pybullet의 physics client는 한 스레드에서만 호출해야 합니다.
    모든 p.* 호출(그리고 Pipeline 내부의 p.* 호출)은 이 클래스의 run() 안에서만
    일어나야 하고, MainWindow(GUI 스레드)는 절대 pybullet을 직접 부르면 안 됩니다.

설계:
    - integration.pipeline.Pipeline이 로봇+수거함+분류 로직 전체를 소유합니다.
      이 클래스는 그 Pipeline을 워커 스레드 안에서 만들고(run() 시작 시), 매
      반복 Pipeline.step_cycle()을 호출한 뒤 화면만 그립니다.
    - Start / Stop / Reset의 의미 (공장 HMI 감각으로 정한 것):
        Start — 스레드가 없으면 새로 시작, 있으면 그 지점부터 재생 재개
        Stop  — 일시정지. pybullet 월드/누적 상태는 그대로 살아있음
        Reset — 월드를 완전히 버리고 처음부터 다시 생성
      즉 "스레드가 살아있는가"와 "지금 재생 중인가"는 서로 다른 상태입니다.
      전자(_thread_alive)는 Reset으로는 안 꺼지고, 창을 닫을 때만 꺼져야 합니다.
    - use_dummy=True(기본값)면 DummyDetector/DummyArmController로 안전하게 돌아갑니다.
      실제 웹캠+YOLO+로봇 연결은 use_dummy=False로 바꾸면 되는데, 그러려면
      WebcamWorker가 잡은 프레임을 set_latest_frame()으로 받아야 합니다
      (MainWindow가 webcam_worker.raw_frame_ready를 여기 연결해줌).
"""

from __future__ import annotations

import time

import numpy as np
import pybullet as p

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

import config
from integration.pipeline import BATCH_COLLECTION_SEC as DEFAULT_BATCH_COLLECTION_SEC
from integration.pipeline import Pipeline
from integration.pipeline import REQUIRED_EMPTY_DETECTIONS as DEFAULT_REQUIRED_EMPTY_DETECTIONS

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
"""pybullet 시뮬레이션 패널 렌더링 해상도 (웹캠/검출 해상도와는 무관).
★ p.getCameraImage()가 idle 상태 fps의 실질적인 상한. 2026-08-04 실측 당시엔
  320x240이 240x180보다 체감상 무거워서 240x180을 기본값으로 잡았었는데,
  이후(로그 즉시 emit 전환, 워크스페이스 필터링, 모션 큐 무제한화 등) fps가
  전반적으로 좋아져서 320x240으로 올려도 여유가 생김(2026-08-05). Render
  Quality 패널에서 언제든 낮출 수 있음.
★ 로봇팔 이동 애니메이션의 "부드러움"은 이 해상도가 아니라
  robot/arm_controller.py의 STREAM_EVERY_N_STEPS(캡처 밀도)와 MOTION_REPLAY_FPS
  (재생 속도)가 결정함 — 이미 둘 다 2배로 올려둔 상태(개별 프레임 개수 증가).
  이 해상도는 프레임 "개수"가 아니라 프레임 한 장의 화질에만 영향을 줌."""
TARGET_FPS = 60  # 기본값. Settings 다이얼로그에서 바꾸면 인스턴스별 self._target_fps로 대체됨

MOTION_REPLAY_FPS = 34
"""로봇팔 이동 중 on_step()이 프레임을 emit하는 페이싱 목표 fps.
★ 물리 계산 자체는 sleep 없이 원래 속도(사실상 즉시)로 끝나버려서, 페이싱 없이
  캡처하는 족족 emit하면 물리가 빠르게 수렴하는 구간(짧은 이동)은 순식간에
  지나가버림. on_step()이 불릴 때마다 이 fps 기준 "다음 목표 시각"까지
  기다렸다가 캡처+emit해서 이동이 항상 일정한 속도로 "재생"되는 것처럼
  보이게 함 — sleep은 워커 스레드 안에서만 걸리므로 GUI 스레드는 안 막힘.
★ (2026-08-05) 40 -> 25로 낮춤. 320x240 캡처가 평균 ~15ms지만 ±10~15ms로
  들쭉날쭉해서, 40fps(예산 25ms/frame)에선 캡처가 조금만 느려져도 예산을
  넘겨 끊겨 보였음. 25fps(예산 40ms/frame)로 여유를 넉넉히 주면 그 변동폭이
  대부분 예산 안에 흡수됨 — 재생 속도는 느려지지만(트레이드오프) 끊김 없이
  일정하게 보이는 걸 우선함.
★ 2026-08-05 이전엔 on_step()이 프레임을 큐에 모아두기만 하고, step_cycle()이
  리턴한 뒤 바깥 루프에서 이 fps로 재생하는 방식이었음 — 배치(물체 여러 개)가
  다 끝날 때까지 화면이 아예 안 갱신되는 문제가 있어서, on_step() 안에서
  직접 emit+페이싱하는 방식으로 교체(큐 자체를 없앰). 대가로 Stop/Reset은
  여전히 물체 사이(또는 늦어도 한 이동 사이)에서만 반영됨 — 이동 도중
  즉시 중단은 안 됨(그러려면 Pipeline 쪽에 취소 메커니즘이 필요).
★ robot/arm_controller.py의 STREAM_EVERY_N_STEPS와 2배씩 같이 조절할 것.
  캡처 밀도(N_STEPS)와 재생 속도(이 값)를 같은 비율로 올리면 재생 "시간"은
  거의 그대로 유지하면서 프레임 밀도만 올라가 더 매끄럽게 보임."""


class SimWorker(QThread):
    """Pipeline 루프를 도는 워커 스레드.

    Signals (클래스 몸체에 이렇게 선언하면 "이 클래스의 인스턴스는 이런 이벤트를
    내보낼 수 있다"는 뜻이 됩니다. 괄호 안 타입은 emit()할 때 넘길 데이터 타입):
        frame_ready(QImage): 렌더링된 시뮬레이션 프레임 한 장.
        state_changed(dict): {"fps", "step", "sorted", "success_rate"} 상태 값.
        log_message(str): 로그 패널에 표시할 텍스트 한 줄.
    """

    frame_ready = Signal(QImage)
    state_changed = Signal(dict)
    log_message = Signal(str)
    robot_state_changed = Signal(str)

    def __init__(self, use_dummy: bool = True) -> None:
        super().__init__()
        self._use_dummy = use_dummy
        self._thread_alive = False   # 스레드 자체의 생존 여부 (Reset으로는 안 꺼짐)
        self._playing = False        # 재생 중인지 (Start/Stop이 토글)
        self._reset_requested = False
        self._pipeline: Pipeline | None = None
        self._latest_frame = None    # WebcamWorker가 set_latest_frame()으로 채워줌 (real 모드용)
        self.last_detections = []    # Pipeline이 계산한 최신 검출 결과. MainWindow가 웹캠 뷰에
                                      # bbox를 그릴 때 읽어감 (YOLO 재실행 없이 재사용).
        # Settings 다이얼로그로 조절할 값들.
        self._target_fps = TARGET_FPS
        self._camera_distance = config.SIM_CAMERA_DISTANCE
        self._camera_yaw = config.SIM_CAMERA_YAW
        self._camera_pitch = config.SIM_CAMERA_PITCH
        self._frame_width = FRAME_WIDTH
        self._frame_height = FRAME_HEIGHT
        self._conf_threshold = config.CONF_THRESHOLD
        self._batch_collection_sec = DEFAULT_BATCH_COLLECTION_SEC
        self._required_empty_detections = DEFAULT_REQUIRED_EMPTY_DETECTIONS
        # ★ 여기(__init__)는 메인/GUI 스레드에서 실행됩니다. 그래서 여기서
        #   pybullet을 connect하면 안 됩니다 — 실제 연결은 run() 안, 즉 워커
        #   스레드가 시작된 뒤에 해야 합니다.

    # ------------------------------------------------------------------ #
    # MainWindow가 호출하는 제어 메서드 (전부 GUI 스레드에서 호출됨)
    # ------------------------------------------------------------------ #

    def start_simulation(self) -> None:
        """Start 메뉴 액션에 연결할 메서드.

        ★ 이 메서드는 GUI 스레드에서 실행되는데, 여기서 log_message.emit()을
          호출해도 안전합니다 — 받는 쪽(log_panel)도 같은 GUI 스레드에 있어서
          Qt가 큐에 안 넣고 바로 직접 호출해줍니다 (워커 스레드 run() 안에서
          emit할 때와 동작 방식은 같지만, 신경 쓸 스레드 안전 이슈가 없다는 뜻).
        """

        first_start = not self.isRunning()
        self._playing = True
        if first_start:
            self._thread_alive = True
            self.start()
            self.log_message.emit("Start: 시뮬레이션 시작")
        else:
            self.log_message.emit("Start: 재생 재개")

    def stop_simulation(self) -> None:
        """Stop 메뉴 액션에 연결할 메서드."""

        self._playing = False
        self.log_message.emit("Stop: 일시정지")

    def reset_simulation(self) -> None:
        """Reset 메뉴 액션에 연결할 메서드.

        ★ 여기서 바로 pybullet을 재연결하면 안 됩니다. 이 메서드는 GUI 스레드에서
          실행되는데, pybullet은 워커 스레드에서만 호출해야 하기 때문입니다.
          대신 "리셋해줘"라는 요청 플래그만 세워두고, 실제 재생성은 run() 루프
          안에서 다음 반복 시작 시 처리하게 하세요.
        """

        self._reset_requested = True

    def shutdown(self) -> None:
        """창 닫을 때(MainWindow.closeEvent) 호출. 스레드를 완전히 끝냅니다."""

        self._thread_alive = False

    def set_latest_frame(self, frame: np.ndarray) -> None:
        """WebcamWorker 스레드가 새 프레임을 잡을 때마다 DirectConnection으로 직접
        호출됩니다 (GUI 스레드를 거치지 않음). 단순 참조 대입이라 락 없이도
        안전합니다 (CPython의 GIL이 대입 자체의 원자성을 보장).

        use_dummy=True일 때는 아무도 이 값을 읽지 않으니 그냥 저장만 해둡니다.
        """
        self._latest_frame = frame

    def _on_pipeline_log(self, message: str) -> None:
        """Pipeline(log_fn)이 워커 스레드 안에서 직접 호출함(step_cycle() 호출
        스택 안이라 self.run()과 같은 스레드 — cross-thread 아님).

        즉시 emit. 예전엔 화면 재생 프레임 수에 맞춰 emit을 지연시켰었는데,
        _execute_batch()가 move_to() 타임아웃으로 오래 블로킹되는 동안 그
        지연 큐만 계속 쌓이다가(step_cycle()이 안 끝나서 drain도 안 됨) 풀리는
        순간 한꺼번에 쏟아지는 "와르르" 버그가 있었음(2026-08-04). 실패 진단
        시엔 실시간 피드백이 화면-로그 동기화보다 중요해서 즉시 emit으로
        되돌림. FSM 라벨(robot_state_changed)은 여전히 프레임 큐와 같이
        묶어서 지연 emit함 — 그쪽은 상태가 안 바뀌는 구간이라 문제 덜함."""
        self.log_message.emit(message)

    def apply_settings(
        self,
        target_fps: int | None = None,
        distance: float | None = None,
        yaw: float | None = None,
        pitch: float | None = None,
        frame_width: int | None = None,
        frame_height: int | None = None,
        conf_threshold: float | None = None,
        batch_collection_sec: float | None = None,
        required_empty_detections: int | None = None,
    ) -> None:
        """SettingsDialog에서 OK를 눌렀을 때 MainWindow가 호출.

        ★ 이것도 GUI 스레드에서 실행되는 메서드입니다. reset_simulation()과 똑같은
          이유로, 여기서 pybullet을 직접 건드리면 안 되고 인스턴스 변수만 갱신해야
          합니다.
        """
        if target_fps is not None:
            self._target_fps = target_fps
        if distance is not None:
            self._camera_distance = distance
        if yaw is not None:
            self._camera_yaw = yaw
        if pitch is not None:
            self._camera_pitch = pitch
        if frame_width is not None:
            self._frame_width = frame_width
        if frame_height is not None:
            self._frame_height = frame_height
        if conf_threshold is not None:
            self._conf_threshold = conf_threshold
            # detector.conf는 그냥 float 속성이라 GUI 스레드에서 바로 대입해도
            # 안전함 (다른 설정값들과 같은 근거). use_dummy 모드는 DummyDetector라
            # conf 속성 자체가 없으므로 건드리지 않음.
            if self._pipeline is not None and not self._use_dummy:
                self._pipeline.detector.conf = conf_threshold
        if batch_collection_sec is not None:
            self._batch_collection_sec = batch_collection_sec
            if self._pipeline is not None:
                self._pipeline.batch_collection_sec = batch_collection_sec
        if required_empty_detections is not None:
            self._required_empty_detections = required_empty_detections
            if self._pipeline is not None:
                self._pipeline.required_empty_detections = required_empty_detections

        # 카메라 각도/거리/렌더 해상도는 물리 씬과 무관한 "그리는 방식"일 뿐이라
        # reset(물리 재생성)이 필요 없음. _capture_frame()이 매번 최신 값으로
        # view_matrix/proj_matrix를 다시 계산하니 여기서 값만 갱신하면 다음
        # 프레임부터 바로 반영됨. 큐가 없어져서(2026-08-05) 예전엔 여기서
        # _motion_frame_queue를 비워야 했지만 이제는 그럴 필요도 없음 — 다음
        # on_step()/idle 캡처가 바로 새 값으로 찍힘.

    def get_settings(self) -> dict:
        """Settings 다이얼로그를 열 때 현재 값으로 미리 채우기 위해 MainWindow가 호출."""
        return {
            "target_fps": self._target_fps,
            "sim_distance": self._camera_distance,
            "sim_yaw": self._camera_yaw,
            "sim_pitch": self._camera_pitch,
            "frame_width": self._frame_width,
            "frame_height": self._frame_height,
            "conf_threshold": self._conf_threshold,
            "batch_collection_sec": self._batch_collection_sec,
            "required_empty_detections": self._required_empty_detections,
        }

    # ------------------------------------------------------------------ #
    # 아래부터는 전부 워커 스레드 안에서만 실행됨 (self.start() 호출 시 자동 실행)
    # ------------------------------------------------------------------ #

    def run(self) -> None:
        """QThread가 self.start() 호출 시 새 스레드에서 실행하는 진입점.

        ★ 이 메서드(그리고 여기서 부르는 모든 것) 안에서 예외가 나면 QThread는
          그냥 조용히 스레드를 끝내버립니다 — GUI에는 아무 신호도 안 감. 그러면
          "Waiting for simulation..." 화면에서 Start를 눌러도 반응 없는 것처럼
          보이므로, 전체를 try/except로 감싸서 최소한 로그 패널에는 원인을
          남깁니다.
        """
        try:
            self._build_scene()
        except Exception as exc:
            self.log_message.emit(f"시뮬레이션 초기화 실패: {exc}")
            self._thread_alive = False
            return

        step_count = 0
        frames_since_fps = 0
        last_fps_time = time.monotonic()
        last_frame_time = time.monotonic()
        next_motion_deadline = time.monotonic()
        """on_step()이 다음 캡처+emit을 해도 되는 목표 시각(모션 재생 페이싱용).
        run() 시작 시/Reset 시에만 초기화하고, 그 외엔 on_step()이 스스로
        갱신함(_emit_motion_frame 참고) — move_to() 호출 여러 번(한 물체의
        approach/descend/... 전부, 배치면 물체 여러 개까지)에 걸쳐 계속
        이어지는 하나의 페이싱 시계라서 이동 경계마다 끊기지 않고 일정한
        속도로 이어짐."""

        try:
            while self._thread_alive:
                if self._reset_requested:
                    self._teardown_scene()
                    self._build_scene()

                    step_count = 0
                    frames_since_fps = 0
                    last_fps_time = time.monotonic()
                    last_frame_time = time.monotonic()
                    next_motion_deadline = time.monotonic()

                    self.frame_ready.emit(self._capture_frame())
                    self.last_detections = []

                    self._reset_requested = False
                    self._playing = False
                    self.log_message.emit("Reset 완료 : 다시 Start 하세요")

                if not self._playing:
                    self.msleep(30)
                    continue

                # frame=None(웹캠 아직 미연결 등)은 Pipeline.step_cycle()이 안전하게
                # 건너뜀 — ultralytics가 None을 번들 샘플 이미지로 대체해 가짜 검출을
                # 만들어내는 걸 막기 위한 가드가 거기 있음.
                frame = None if self._use_dummy else self._latest_frame

                # 로봇팔이 목표까지 이동하는 동안 STREAM_EVERY_N_STEPS 스텝마다 이
                # 콜백이 불림 — 여기서 캡처하고 곧바로 emit까지 함(큐에 모아뒀다
                # step_cycle() 리턴 후 재생하는 방식 폐지, 2026-08-05: 배치가 여러
                # 물체를 동기 처리하는 동안 화면이 아예 안 갱신되던 문제의 근본
                # 원인이었음). MOTION_REPLAY_FPS 목표 시각까지 sleep해서 페이싱을
                # 유지 — 페이싱 없이 그냥 emit만 하면 물리가 빨리 수렴하는 짧은
                # 이동이 눈 깜빡할 새 지나가버림.
                def _emit_motion_frame() -> None:
                    nonlocal last_frame_time, frames_since_fps, next_motion_deadline

                    now = time.monotonic()
                    if now < next_motion_deadline:
                        time.sleep(next_motion_deadline - now)
                        now = next_motion_deadline
                    # 여기서부터가 "다음 목표 시각"의 기준점 — 이미 목표보다
                    # 늦었으면(캡처 자체가 페이싱 간격보다 오래 걸린 경우) sleep을
                    # 건너뛰고 지금 시각을 새 기준으로 삼음. 밀린 시간을 나중에
                    # "따라잡으려" 하지 않음 -> 밀림이 누적되지 않음.
                    next_motion_deadline = now + 1.0 / MOTION_REPLAY_FPS

                    # 캡처 시점의 FSM 상태를 프레임과 함께 emit -> 화면에 보이는
                    # 것과 라벨이 항상 짝이 맞음.
                    state_name = self._pipeline.arm.state.name
                    self.frame_ready.emit(self._capture_frame())
                    self.robot_state_changed.emit(state_name)

                    frames_since_fps += 1
                    last_frame_time = now

                completed = self._pipeline.step_cycle(frame, on_step=_emit_motion_frame)
                self.last_detections = self._pipeline.last_detections
                step_count += 1
                if completed:
                    summary = self._pipeline.logger.summary()
                    self.log_message.emit(
                        f"분류 완료 (누적 {summary['total']}회, 성공률 {summary['success_rate']:.0%})"
                    )

                now = time.monotonic()

                # 로봇팔이 안 움직이는 idle 구간(WAITING/COLLECTING/WAITING_CLEAR 등,
                # on_step()이 한 번도 안 불린 step_cycle() 호출)에는 위 콜백이 전혀
                # 안 불렸으므로 여기서 target_fps로 따로 캡처해서 화면을 갱신함.
                # 스텝 수가 아니라 실제 경과 시간으로 캡처 여부를 판단 -> msleep 값과
                # 무관하게 항상 목표 fps에 가깝게 프레임이 나옴. self._target_fps를
                # 매번 다시 읽으므로 apply_settings()로 바꾼 값이 다음 반복부터 바로
                # 반영됨.
                if now - last_frame_time >= 1.0 / self._target_fps:
                    self.frame_ready.emit(self._capture_frame())
                    if not self._use_dummy:
                        self.robot_state_changed.emit(self._pipeline.arm.state.name)

                    frames_since_fps += 1
                    last_frame_time = now

                if now - last_fps_time >= 1.0:
                    fps = frames_since_fps / (now - last_fps_time)
                    summary = self._pipeline.logger.summary()
                    self.state_changed.emit({
                        "fps": fps,
                        "step": step_count,
                        "sorted": summary["total"],
                        "success_rate": summary["success_rate"],
                    })
                    frames_since_fps = 0
                    last_fps_time = now
        except Exception as exc:
            self.log_message.emit(f"시뮬레이션 루프 오류로 정지: {exc}")
        finally:
            self._teardown_scene()

    def _build_scene(self) -> None:
        """Pipeline(로봇 + 수거함 + 분류 로직)을 생성합니다. 항상 DIRECT 모드로 붙여서
        네이티브 PyBullet 창 없이 계산만 하고, 화면은 getCameraImage()로 그립니다."""

        # log_fn: Pipeline 내부의 print() 대신 이 콜백으로 로그를 내보내게 해서
        # GUI Log 패널에 뜨게 함(터미널이 아니라).
        self._pipeline = Pipeline(
            use_dummy=self._use_dummy, use_gui=False, log_fn=self._on_pipeline_log
        )
        self._pipeline.start()

        # 아래 값들은 Pipeline 생성 시점에 모듈 기본값으로 초기화되므로, Reset으로
        # Pipeline이 새로 만들어지면 옵션바로 조절해둔 값이 사라지고 기본값으로
        # 되돌아감. 여기서 다시 덮어써서 유지되게 함.
        if not self._use_dummy:
            self._pipeline.detector.conf = self._conf_threshold
        self._pipeline.batch_collection_sec = self._batch_collection_sec
        self._pipeline.required_empty_detections = self._required_empty_detections

        # 투영 행렬(fov/aspect/near/far)과 뷰 행렬(거리/yaw/pitch) 둘 다 여기서
        # 캐싱하지 않고 _capture_frame()에서 매번 최신 값으로 다시 계산함 —
        # aspect가 렌더 해상도(RenderQualityPanel로 실시간 조절)에 의존하고,
        # 순수 행렬 연산이라 매번 다시 계산해도 비용은 무시할 수준이기 때문.

    def _teardown_scene(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline.shutdown()
            self._pipeline = None
        self.last_detections = []

    def _capture_frame(self) -> QImage:
        """현재 씬을 렌더링해서 QImage로 변환합니다.

        최적화 포인트:
            - proj/view 행렬 둘 다 매 프레임 새로 계산합니다 (해상도/카메라 조절
              슬라이더 때문에). 순수 행렬 연산이라 getCameraImage() 자체의
              렌더링 비용에 비하면 무시할 수준입니다.
            - flags=p.ER_NO_SEGMENTATION_MASK: 우리는 segmentation mask(물체별 ID
              픽셀맵)를 안 쓰는데, 이 플래그 없이는 pybullet이 매 프레임 그걸 같이
              계산해서 버림. 꺼주면 그만큼 렌더링 비용이 줄어듦.
            - 알파 채널을 버리고 RGB888로 바꾸는 대신, 4채널 그대로 Format_RGBA8888로
              감쌈. 그러면 [:, :, :3] 슬라이싱과 그로 인한 np.ascontiguousarray() 복사가
              통째로 사라짐 (알파값은 항상 255라 화면에 보이는 색에는 차이 없음).
            - pybullet의 getCameraImage()는 픽셀을 numpy 배열이 아니라 순수 파이썬
              tuple(int 30만+개)로 반환함. np.asarray()보다 np.fromiter()가 이런
              "긴 파이썬 iterable -> numpy 배열" 변환에 더 최적화돼 있어서 더 빠름
              (실측: 6.0ms -> 4.4ms, 약 27% 단축).
        """
        width, height = self._frame_width, self._frame_height
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=width / height, nearVal=0.1, farVal=100.0,
        )
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=list(config.SIM_CAMERA_TARGET),
            distance=self._camera_distance,
            yaw=self._camera_yaw,
            pitch=self._camera_pitch,
            roll=0,
            upAxisIndex=2,
        )
        _, _, rgba, _, _ = p.getCameraImage(
            width, height, view_matrix, proj_matrix,
            renderer=p.ER_TINY_RENDERER,
            flags=p.ER_NO_SEGMENTATION_MASK,
            physicsClientId=self._pipeline.scene.client_id,
        )
        rgba_array = np.fromiter(rgba, dtype=np.uint8, count=width * height * 4)
        rgba_array = rgba_array.reshape((height, width, 4))
        image = QImage(rgba_array.data, width, height, 4 * width, QImage.Format.Format_RGBA8888)
        # ★ QImage가 numpy 버퍼를 빌려서 보는 것뿐이므로, 스레드 경계를 넘기기 전에
        #   반드시 .copy()로 데이터를 복제해야 함 (안 하면 GC 후 깨진 이미지/크래시).
        return image.copy()
