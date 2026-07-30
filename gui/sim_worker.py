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

import random
import time
import numpy as np
import pybullet as p
import pybullet_data

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
TARGET_FPS = 60  # 기본값. Settings 다이얼로그에서 바꾸면 인스턴스별 self._target_fps로 대체됨

NUM_OBJECTS = 6
RESPAWN_INTERVAL_SEC = 10.0  # 이 주기마다 물체를 전부 치우고 새로 떨어뜨림 (무한 반복)
OBJECT_HALF_EXTENT = 0.05
OBJECT_RESTITUTION = 0.8  # 반발력(0=안 튕김, 1=에너지 손실 없이 튕김). 기본값 0이라 명시 필요
OBJECT_COLORS = [
    [0.2, 0.4, 1.0, 1.0],  # blue
    [0.2, 0.8, 0.3, 1.0],  # green
    [0.9, 0.6, 0.1, 1.0],  # orange
    [0.8, 0.2, 0.8, 1.0],  # purple
    [0.9, 0.2, 0.2, 1.0],  # red
]


class SimWorker(QThread):
    """시뮬레이션 루프를 도는 워커 스레드.

    Signals (클래스 몸체에 이렇게 선언하면 "이 클래스의 인스턴스는 이런 이벤트를
    내보낼 수 있다"는 뜻이 됩니다. 괄호 안 타입은 emit()할 때 넘길 데이터 타입):
        frame_ready(QImage): 렌더링된 시뮬레이션 프레임 한 장.
        state_changed(dict): {"fps": ..., "step": ..., "avg_height": ...} 같은 상태 값.
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
        self._object_ids: list[int] = []

        # Settings 다이얼로그로 조절할 값들. 지금은 기존 하드코딩 값과 동일하게
        # 초기화만 해두고, apply_settings()/get_settings()는 아래에 TODO로 남겨둠.
        self._target_fps = TARGET_FPS
        self._camera_distance = 1.5
        self._camera_yaw = 45.0
        self._camera_pitch = -30.0
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

    def apply_settings(
        self,
        target_fps: int | None = None,
        distance: float | None = None,
        yaw: float | None = None,
        pitch: float | None = None,
    ) -> None:
        """SettingsDialog에서 OK를 눌렀을 때 MainWindow가 호출.

        ★ 이것도 GUI 스레드에서 실행되는 메서드입니다. reset_simulation()과 똑같은
          이유로, 여기서 pybullet을 직접 건드리면 안 되고 인스턴스 변수만 갱신해야
          합니다.

        TODO:
            - 넘어온 값 중 None이 아닌 것만 self._target_fps / self._camera_distance /
              self._camera_yaw / self._camera_pitch에 반영하세요.
            - target_fps는 run() 루프가 매 반복 self._target_fps를 직접 읽어서 쓰도록
              바꿔뒀으니(아래 run()/FRAME_INTERVAL 부분 참고) 별도 조치 없이 바로 적용됨.
            - distance/yaw/pitch(카메라 각도)는 _build_scene()에서 view_matrix를 만들
              때만 쓰이므로, 값만 바꿔서는 당장 반영되지 않습니다. self._reset_requested
              = True로 세팅해서 다음 run() 루프에서 _build_scene()이 다시 불리며 새
              각도로 view_matrix가 재계산되게 하세요.
        """
        if target_fps is not None:
            self._target_fps = target_fps
        if distance is not None:
            self._camera_distance = distance
        if yaw is not None:
            self._camera_yaw = yaw
        if pitch is not None:
            self._camera_pitch = pitch

        # 카메라 각도/거리는 물리 씬과 무관한 "그리는 방식"일 뿐이라 reset(물리
        # 재생성)이 필요 없음. _capture_frame()이 매번 최신 값으로 view_matrix를
        # 다시 계산하니 여기서 값만 갱신하면 다음 프레임부터 바로 반영됨.

    def get_settings(self) -> dict:
        """Settings 다이얼로그를 열 때 현재 값으로 미리 채우기 위해 MainWindow가 호출.

        TODO: {"target_fps": self._target_fps, "sim_distance": self._camera_distance,
               "sim_yaw": self._camera_yaw, "sim_pitch": self._camera_pitch} 반환.
        """
        return {
            "target_fps": self._target_fps,
            "sim_distance": self._camera_distance,
            "sim_yaw": self._camera_yaw,
            "sim_pitch": self._camera_pitch,
        }

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
        last_respawn_time = time.monotonic()

        while self._thread_alive:
            if self._reset_requested:
                self._teardown_scene()
                self._build_scene()

                step_count = 0
                frames_since_fps = 0
                last_fps_time = time.monotonic()
                last_frame_time = time.monotonic()
                last_respawn_time = time.monotonic()

                self.frame_ready.emit(self._capture_frame())

                self._reset_requested = False
                self._playing = False
                self.log_message.emit("Reset 완료 : 다시 Start 하세요")
            if not self._playing:
                self.msleep(30)
                continue

            p.stepSimulation(physicsClientId=self._client_id)
            self.msleep(4)  # 물리 스텝을 대략 실시간 속도로 유지 (240Hz 기준 ~4ms/스텝)
            step_count += 1

            now = time.monotonic()

            # 물체들이 바닥에 자리 잡고 나면 화면이 정지된 것처럼 보이니, 일정 주기마다
            # 전부 치우고 새로 색깔/위치를 바꿔 다시 떨어뜨림 (무한 반복).
            if now - last_respawn_time >= RESPAWN_INTERVAL_SEC:
                self._despawn_objects()
                self._spawn_objects()
                last_respawn_time = now

            # 스텝 수가 아니라 실제 경과 시간으로 캡처 여부를 판단 -> msleep 값과
            # 무관하게 항상 목표 fps에 가깝게 프레임이 나옴.
            # self._target_fps를 매번 다시 읽으므로 apply_settings()로 바꾼 값이
            # 다음 반복부터 바로 반영됨 (모듈 상수 FRAME_INTERVAL 대신 사용).
            if now - last_frame_time >= 1.0 / self._target_fps:
                self.frame_ready.emit(self._capture_frame())
                frames_since_fps += 1
                last_frame_time = now

            if now - last_fps_time >= 1.0:
                fps = frames_since_fps / (now - last_fps_time)
                heights = [
                    p.getBasePositionAndOrientation(body_id, physicsClientId=self._client_id)[0][2]
                    for body_id in self._object_ids
                ]
                avg_height = sum(heights) / len(heights) if heights else 0.0
                self.state_changed.emit({"fps": fps, "step": step_count, "avg_height": avg_height})
                frames_since_fps = 0
                last_fps_time = now

    def _build_scene(self) -> None:
        """바닥 + 낙하하는 큐브 하나짜리 최소 씬을 만듭니다. 항상 DIRECT 모드로."""
        
        self._client_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self._client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client_id)
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self._client_id)
        # ★ pybullet은 기본 반발력(restitution)이 0이라 아무것도 안 튕깁니다.
        #   바닥과 물체 양쪽에 restitution을 줘야 실제로 튕기는 게 보입니다
        #   (충돌 시 실제 반발력은 두 값을 조합해서 계산됨).
        p.changeDynamics(plane_id, -1, restitution=OBJECT_RESTITUTION, physicsClientId=self._client_id)
        self._spawn_objects()

        # 투영 행렬(fov/aspect/near/far)은 사용자가 못 바꾸니 한 번만 계산해서 재사용.
        # 뷰 행렬(거리/yaw/pitch)은 CameraControlPanel로 실시간 조절되므로 여기서
        # 캐싱하지 않고 _capture_frame()에서 매번 최신 값으로 다시 계산함.
        self._proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=FRAME_WIDTH / FRAME_HEIGHT,
            nearVal=0.1,
            farVal=100.0,
        )


    def _teardown_scene(self) -> None:
        """TODO: self._client_id가 None이 아니면 p.disconnect(physicsClientId=...)
        호출하고, self._client_id / self._object_ids를 다시 초기 상태로 되돌리세요."""
        if self._client_id is not None:
            p.disconnect(physicsClientId=self._client_id)
            self._client_id = None
            self._object_ids = []

    def _spawn_objects(self) -> None:
        """색깔 있는 물체 NUM_OBJECTS개를 임의 위치/색으로 떨어뜨림."""
        self._object_ids = []
        for _ in range(NUM_OBJECTS):
            color = random.choice(OBJECT_COLORS)
            half = OBJECT_HALF_EXTENT
            position = [
                random.uniform(-0.3, 0.3),
                random.uniform(-0.3, 0.3),
                random.uniform(1.0, 2.5),
            ]
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[half] * 3, rgbaColor=color, physicsClientId=self._client_id
            )
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half] * 3, physicsClientId=self._client_id)
            body_id = p.createMultiBody(
                baseMass=0.2,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=position,
                physicsClientId=self._client_id,
            )
            p.changeDynamics(body_id, -1, restitution=OBJECT_RESTITUTION, physicsClientId=self._client_id)
            self._object_ids.append(body_id)

    def _despawn_objects(self) -> None:
        """_spawn_objects()로 만든 물체를 전부 제거."""
        for body_id in self._object_ids:
            p.removeBody(body_id, physicsClientId=self._client_id)
        self._object_ids = []

    def _capture_frame(self) -> QImage:
        """현재 씬을 렌더링해서 QImage로 변환합니다.

        최적화 포인트 (기존 대비):
            - proj 행렬은 _build_scene()에서 한 번만 계산해두고 재사용 (안 바뀌는 값).
              view 행렬은 카메라 조절 슬라이더 때문에 매 프레임 새로 계산하지만,
              이건 순수 행렬 연산이라 getCameraImage() 자체의 렌더링 비용에 비하면
              무시할 수준입니다.
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
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0],
            distance=self._camera_distance,
            yaw=self._camera_yaw,
            pitch=self._camera_pitch,
            roll=0,
            upAxisIndex=2,
        )
        _, _, rgba, _, _ = p.getCameraImage(
            FRAME_WIDTH, FRAME_HEIGHT, view_matrix, self._proj_matrix,
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
