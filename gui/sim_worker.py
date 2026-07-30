"""
gui/sim_worker.py — PyBullet 시뮬레이션을 별도 스레드에서 구동하는 워커

Qt 처음이면 알아둘 개념:
- QThread: 별도 스레드에서 코드를 실행하게 해주는 클래스. `self.start()`를 호출하면
  Qt가 새 스레드를 만들고 그 안에서 `run()` 메서드를 자동으로 호출해줍니다.
  (run()을 직접 호출하면 그냥 지금 스레드에서 실행돼버리니 절대 직접 부르지 마세요.)
- Signal: "이런 일이 일어났다"를 다른 객체(주로 다른 스레드에 있는 위젯)에게 알리는
  통로입니다. `self.frame_ready.emit(image)`처럼 emit()으로 신호를 보내면, 이
  시그널에 connect() 해둔 함수(슬롯)가 Qt에 의해 안전하게 GUI 스레드에서 호출됩니다.
  스레드 간 데이터 전달은 항상 이 시그널을 통해서만 하세요.

★★★ 중요: pybullet의 physics client는 한 스레드에서만 호출해야 합니다.
    모든 p.* 호출은 이 클래스의 run() 안에서만 일어나야 하고, MainWindow(GUI 스레드)는
    절대 pybullet을 직접 부르면 안 됩니다.

설계 방향 (지난 논의 정리):
    - integration/pipeline.py, robot/scene.py 등 팀원 모듈은 아직 건드리지 않습니다.
      대신 이 클래스가 직접 만드는 아주 단순한 자체 씬(바닥 + 낙하하는 큐브 하나)으로
      GUI/스레드/렌더링 파이프라인만 먼저 검증하는 게 목표입니다.
    - Start / Stop / Reset의 의미 (공장 HMI 감각으로 정한 것):
        Start — 스레드가 없으면 새로 시작, 있으면 그 지점부터 재생 재개
        Stop  — 일시정지. pybullet 월드/누적 상태는 그대로 살아있음
        Reset — 월드를 완전히 버리고 처음부터 다시 생성
      즉 "스레드가 살아있는가"와 "지금 재생 중인가"는 서로 다른 상태입니다.
      전자(_thread_alive)는 Reset으로는 안 꺼지고, 창을 닫을 때만 꺼져야 합니다.

이 파일은 스켈레톤입니다. 아래 TODO를 하나씩 채워보세요.
"""

from __future__ import annotations

import time
import numpy as np
import pybullet as p
import pybullet_data

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
TARGET_FPS = 60
FRAME_INTERVAL = 1.0 / TARGET_FPS  # 이 시간 이상 지났을 때만 캡처 (스텝 수 대신 실제 경과 시간 기준)


class SimWorker(QThread):
    """시뮬레이션 루프를 도는 워커 스레드.

    Signals (클래스 몸체에 이렇게 선언하면 "이 클래스의 인스턴스는 이런 이벤트를
    내보낼 수 있다"는 뜻이 됩니다. 괄호 안 타입은 emit()할 때 넘길 데이터 타입):
        frame_ready(QImage): 렌더링된 시뮬레이션 프레임 한 장.
        state_changed(dict): {"fps": ..., "step": ..., "cube_z": ...} 같은 상태 값.
        log_message(str): 로그 패널에 표시할 텍스트 한 줄.
    """

    frame_ready = Signal(QImage)
    state_changed = Signal(dict)
    log_message = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._thread_alive = False   # 스레드 자체의 생존 여부 (Reset으로는 안 꺼짐)
        self._playing = False        # 재생 중인지 (Start/Stop이 토글)
        self._reset_requested = False
        self._client_id: int | None = None
        self._cube_id: int | None = None
        # ★ 여기(__init__)는 메인/GUI 스레드에서 실행됩니다. 그래서 여기서
        #   pybullet을 connect하면 안 됩니다 — 실제 연결은 run() 안, 즉 워커
        #   스레드가 시작된 뒤에 해야 합니다.

    # ------------------------------------------------------------------ #
    # MainWindow가 호출하는 제어 메서드 (전부 GUI 스레드에서 호출됨)
    # ------------------------------------------------------------------ #

    def start_simulation(self) -> None:
        """Start 메뉴 액션에 연결할 메서드."""

        self._playing = True
        if not self.isRunning():
            self._thread_alive = True
            self.start()

    def stop_simulation(self) -> None:
        """Stop 메뉴 액션에 연결할 메서드."""

        self._playing = False

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

    # ------------------------------------------------------------------ #
    # 아래부터는 전부 워커 스레드 안에서만 실행됨 (self.start() 호출 시 자동 실행)
    # ------------------------------------------------------------------ #

    def run(self) -> None:
        """QThread가 self.start() 호출 시 새 스레드에서 실행하는 진입점."""

        self._build_scene()
        step_count = 0
        frames_since_fps = 0
        last_fps_time = time.monotonic()
        last_frame_time = time.monotonic()

        while self._thread_alive:
            if self._reset_requested:
                self._teardown_scene()
                self._build_scene()

                step_count = 0
                frames_since_fps = 0
                last_fps_time = time.monotonic()
                last_frame_time = time.monotonic()

                self.frame_ready.emit(self._capture_frame())

                self._reset_requested = False
                self._playing = False
                self.log_message.emit("Reset 완료")
            if not self._playing:
                self.msleep(30)
                continue

            p.stepSimulation(physicsClientId=self._client_id)
            self.msleep(4)  # 물리 스텝을 대략 실시간 속도로 유지 (240Hz 기준 ~4ms/스텝)
            step_count += 1

            # 스텝 수가 아니라 실제 경과 시간으로 캡처 여부를 판단 -> msleep 값과
            # 무관하게 항상 TARGET_FPS에 가깝게 프레임이 나옴
            now = time.monotonic()
            if now - last_frame_time >= FRAME_INTERVAL:
                self.frame_ready.emit(self._capture_frame())
                frames_since_fps += 1
                last_frame_time = now

            if now - last_fps_time >= 1.0:
                fps = frames_since_fps / (now - last_fps_time)
                cube_pos, _ = p.getBasePositionAndOrientation(self._cube_id, physicsClientId=self._client_id)
                self.state_changed.emit({"fps": fps, "step": step_count, "cube_z": cube_pos[2]})
                frames_since_fps = 0
                last_fps_time = now

    def _build_scene(self) -> None:
        """바닥 + 낙하하는 큐브 하나짜리 최소 씬을 만듭니다. 항상 DIRECT 모드로."""
        
        self._client_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self._client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client_id)
        p.loadURDF("plane.urdf", physicsClientId=self._client_id)
        self._cube_id = p.loadURDF("cube_small.urdf", basePosition=[0, 0, 1.0],
                                   physicsClientId=self._client_id)

        # 카메라가 고정이라 뷰/투영 행렬은 매 프레임 다시 계산할 필요가 없음.
        # 여기서 한 번만 만들어두고 _capture_frame()에서 재사용 (미세하지만 프레임당
        # 절약되는 계산이라 누적하면 fps에 도움이 됨).
        self._view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0],
            distance=1.5,
            yaw=45,
            pitch=-30,
            roll=0,
            upAxisIndex=2,
        )
        self._proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=FRAME_WIDTH / FRAME_HEIGHT,
            nearVal=0.1,
            farVal=100.0,
        )


    def _teardown_scene(self) -> None:
        """TODO: self._client_id가 None이 아니면 p.disconnect(physicsClientId=...)
        호출하고, self._client_id / self._cube_id를 다시 None으로 되돌리세요."""
        if self._client_id is not None:
            p.disconnect(physicsClientId=self._client_id)
            self._client_id = None
            self._cube_id = None

    def _capture_frame(self) -> QImage:
        """현재 씬을 렌더링해서 QImage로 변환합니다.

        최적화 포인트 (기존 대비):
            - view/proj 행렬은 카메라가 고정이라 _build_scene()에서 한 번만 계산해두고
              여기선 재사용만 함 (매 프레임 재계산 안 함).
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
        _, _, rgba, _, _ = p.getCameraImage(
            FRAME_WIDTH, FRAME_HEIGHT, self._view_matrix, self._proj_matrix,
            renderer=p.ER_TINY_RENDERER,
            flags=p.ER_NO_SEGMENTATION_MASK,
            physicsClientId=self._client_id,
        )
        rgba_array = np.fromiter(rgba, dtype=np.uint8, count=FRAME_WIDTH * FRAME_HEIGHT * 4)
        rgba_array = rgba_array.reshape((FRAME_HEIGHT, FRAME_WIDTH, 4))
        image = QImage(rgba_array.data, FRAME_WIDTH, FRAME_HEIGHT, 4 * FRAME_WIDTH, QImage.Format.Format_RGBA8888)
        # ★ QImage가 numpy 버퍼를 빌려서 보는 것뿐이므로, 스레드 경계를 넘기기 전에
        #   반드시 .copy()로 데이터를 복제해야 함 (안 하면 GC 후 깨진 이미지/크래시).
        return image.copy()
