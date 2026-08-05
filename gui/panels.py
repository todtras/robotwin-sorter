"""
gui/panels.py — 로그 패널 / 상태 패널 위젯

Qt 위젯 기초:
- QWidget: 화면에 뭔가를 그리는 모든 것의 기본 클래스. 보통 이걸 상속해서
  나만의 패널을 만듭니다.
- Layout (QVBoxLayout / QFormLayout 등): 위젯 안에 자식 위젯들을 어떻게 배치할지
  정하는 객체. `layout = QVBoxLayout(self)`처럼 생성자에 self를 넘기면, 그
  레이아웃이 곧바로 self(이 위젯)의 레이아웃으로 등록됩니다. 이후
  `layout.addWidget(자식위젯)`으로 하나씩 추가하면 됩니다.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QSlider,
    QSpinBox,
    QVBoxLayout,
)

import config
from gui.sim_worker import FRAME_HEIGHT as _DEFAULT_FRAME_HEIGHT
from gui.sim_worker import FRAME_WIDTH as _DEFAULT_FRAME_WIDTH
from gui.sim_worker import TARGET_FPS as _DEFAULT_TARGET_FPS
from integration.pipeline import BATCH_COLLECTION_SEC as _DEFAULT_BATCH_COLLECTION_SEC
from integration.pipeline import REQUIRED_EMPTY_DETECTIONS as _DEFAULT_REQUIRED_EMPTY_DETECTIONS

MAX_LOG_LINES = 1000


class LogPanel(QGroupBox):
    """시뮬레이션 이벤트/에러를 텍스트로 쌓아 보여주는 패널."""

    def __init__(self) -> None:
        super().__init__("Log")
        self.text_edit = QPlainTextEdit()
        self.text_edit.setReadOnly(True)
        # 로그는 정렬이 중요해서(타임스탬프, 콜론 등) 고정폭 폰트가 훨씬 읽기 편함.
        # 특정 폰트 이름을 하드코딩하는 대신 시스템의 기본 고정폭 폰트를 물어봄
        # (Windows/Mac/Linux 어디서든 알아서 존재하는 폰트로 매칭됨).
        self.text_edit.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))

        layout = QVBoxLayout(self)
        layout.addWidget(self.text_edit)

    def append_log(self, message: str) -> None:
        """SimWorker/WebcamWorker의 log_message 시그널에 연결될 슬롯.

        ★ 로그가 무한정 쌓이면 메모리 문제가 생기니, 여유가 되면
          self.text_edit.document().blockCount()가 MAX_LOG_LINES를 넘을 때
          오래된 줄부터 지우는 것도 고려해보세요 (QTextCursor로 앞부분을 선택해서
          removeSelectedText() 하면 됩니다). 처음엔 안 해도 동작에는 문제없습니다.
        """
        self.text_edit.appendPlainText(message)


class StatusPanel(QGroupBox):
    """FPS / step / 누적 분류 결과 등 Pipeline 상태 값을 보여주는 패널."""

    def __init__(self) -> None:
        super().__init__("Status")

        self.fps_label = QLabel("-")
        self.step_label = QLabel("-")
        self.sorted_label = QLabel("-")
        self.success_rate_label = QLabel("-")

        layout = QFormLayout(self)
        layout.addRow("FPS", self.fps_label)
        layout.addRow("Step", self.step_label)
        layout.addRow("Sorted", self.sorted_label)
        layout.addRow("Success Rate", self.success_rate_label)

    def update_state(self, state: dict) -> None:
        """SimWorker.state_changed 시그널에 연결될 슬롯.

        state는 {"fps": float, "step": int, "sorted": int, "success_rate": float}
        형태로 들어옵니다 (Pipeline.logger.summary() 기반).
        """

        self.fps_label.setText(f"{state.get('fps', 0):.1f}")
        self.step_label.setText(str(state.get("step", 0)))
        self.sorted_label.setText(str(state.get("sorted", 0)))
        self.success_rate_label.setText(f"{state.get('success_rate', 0):.0%}")


class RenderQualityPanel(QGroupBox):
    """시뮬레이션 패널의 pybullet 렌더 해상도를 고르는 패널 (화질 <-> fps 트레이드오프).

    ★ 웹캠/검출 해상도(config.FRAME_WIDTH/HEIGHT, config.INFERENCE_IMGSZ)와는
      완전히 무관합니다 — 이건 순수하게 "Simulation" 패널을 그리는 데 쓰는
      p.getCameraImage() 해상도만 바꿉니다. gui/sim_worker.py 실측 결과 이
      호출 자체가 idle 상태 fps의 실질적인 상한이라, 낮출수록 fps는 오르고
      화질은 거칠어짐.

    Signals:
        resolution_changed(int, int): (width, height). MainWindow가 이걸 받아서
            sim_worker.apply_settings(frame_width=..., frame_height=...)를 호출.
    """

    resolution_changed = Signal(int, int)

    PRESETS: list[tuple[str, int, int]] = [
        ("160x120", 160, 120),
        ("240x180", 240, 180),
        ("320x240", 320, 240),
        ("480x360", 480, 360),
        ("640x480", 640, 480),
    ]
    """★ "(Fast)"/"(Balanced)"/"(Best quality)" 설명을 뺌 — 콤보박스 자체 너비가
    가장 긴 항목 기준으로 정해지는데, 이 문구들 때문에 옵션바 전체가 필요
    이상으로 넓어짐. 숫자만으로도 뭘 고르는지는 충분히 명확함."""

    def __init__(self) -> None:
        super().__init__("Render Quality")

        self.resolution_combo = QComboBox()
        for label, _w, _h in self.PRESETS:
            self.resolution_combo.addItem(label)
        self.set_resolution(_DEFAULT_FRAME_WIDTH, _DEFAULT_FRAME_HEIGHT)  # sim_worker 기본값과 동기화

        # ★ QFormLayout(라벨|콤보 한 줄)이었다가 세로로 쌓는 방식으로 변경 —
        #   "320x240 (Best quality)" 같은 긴 항목 때문에 콤보박스 자체 너비가
        #   이미 넉넉히 필요한데, 라벨까지 옆에 붙이면 옵션바 전체 폭이 그만큼
        #   더 벌어짐. 라벨을 위로 올리면 그 폭을 안 더해도 됨.
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Sim Resolution"))
        layout.addWidget(self.resolution_combo)

        self.resolution_combo.currentIndexChanged.connect(self._emit_resolution)

    def set_resolution(self, width: int, height: int) -> None:
        """MainWindow가 시작 시 현재 sim_worker 값으로 콤보박스를 맞춰둘 때 호출.

        ★ setCurrentIndex()도 CameraControlPanel.set_values()처럼 currentIndexChanged를
          다시 emit할 수 있음 — 지금 단계에선 무시해도 되는 수준.
        """
        for i, (_label, w, h) in enumerate(self.PRESETS):
            if (w, h) == (width, height):
                self.resolution_combo.setCurrentIndex(i)
                return

    def _emit_resolution(self, index: int) -> None:
        _label, width, height = self.PRESETS[index]
        self.resolution_changed.emit(width, height)


class CameraControlPanel(QGroupBox):
    """시뮬레이션 카메라 거리/yaw/pitch를 실시간으로 조절하는 패널.

    Settings 다이얼로그(모달 팝업)로 만들려다가, 화면에 자리가 넉넉해서 로그/상태
    패널 옆에 상시 노출되는 패널로 바꿈. 그래서 OK/Cancel 없이 슬라이더 값이
    바뀌는 즉시 바로 반영되는 게 자연스러움 (valueChanged 시그널 활용).

    ★ QSlider는 정수(int)만 다룹니다. yaw/pitch는 각도라 정수여도 자연스럽지만,
      distance(0.1 단위, 0.1~10.0)는 "슬라이더 값(int) = 실제값 × 10"으로
      스케일링해서 써야 합니다. 예: 슬라이더 range 1~100, 실제 distance는
      slider.value() / 10.0.
    ★ QSlider는 스핀박스와 달리 지금 값이 얼마인지 숫자로 안 보여줍니다. 그래서
      슬라이더 옆에 값 표시용 QLabel을 하나씩 같이 둡니다.

    Signals:
        params_changed(float, float, float): (distance, yaw, pitch) 중 하나라도
            바뀔 때마다 emit. MainWindow가 이걸 받아서 sim_worker.apply_settings()를
            호출하는 식으로 연결.
    """

    params_changed = Signal(float, float, float)

    DISTANCE_SCALE = 10  # 슬라이더 int 값 <-> 실제 distance(float) 변환 배율

    def __init__(self) -> None:
        super().__init__("Camera Control")

        # 초기값은 config.SIM_CAMERA_*(로봇+수거함 전체가 보이는 기본 앵글)와 맞춤.
        self.distance_slider = QSlider(Qt.Orientation.Horizontal)
        self.distance_slider.setRange(1, 100)  # 실제 0.1 ~ 10.0
        self.distance_slider.setValue(int(config.SIM_CAMERA_DISTANCE * self.DISTANCE_SCALE))
        self.distance_value_label = QLabel(f"{config.SIM_CAMERA_DISTANCE:.1f}")

        self.yaw_slider = QSlider(Qt.Orientation.Horizontal)
        self.yaw_slider.setRange(-180, 180)
        self.yaw_slider.setValue(int(config.SIM_CAMERA_YAW))
        self.yaw_value_label = QLabel(f"{config.SIM_CAMERA_YAW:.0f}")

        self.pitch_slider = QSlider(Qt.Orientation.Horizontal)
        self.pitch_slider.setRange(-90, 90)
        self.pitch_slider.setValue(int(config.SIM_CAMERA_PITCH))
        self.pitch_value_label = QLabel(f"{config.SIM_CAMERA_PITCH:.0f}")

        distance_row = QHBoxLayout()
        distance_row.addWidget(self.distance_slider)
        distance_row.addWidget(self.distance_value_label)
        distance_row.addStretch(1)  # 오른쪽 여백 확보

        yaw_row = QHBoxLayout()
        yaw_row.addWidget(self.yaw_slider)
        yaw_row.addWidget(self.yaw_value_label)
        yaw_row.addStretch(1)

        pitch_row = QHBoxLayout()
        pitch_row.addWidget(self.pitch_slider) 
        pitch_row.addWidget(self.pitch_value_label)
        pitch_row.addStretch(1)

        row = QFormLayout(self)
        row.addRow("Distance", distance_row)
        row.addRow("Yaw", yaw_row)
        row.addRow("Pitch", pitch_row)

        self.distance_slider.valueChanged.connect(self._emit_params)
        self.yaw_slider.valueChanged.connect(self._emit_params)
        self.pitch_slider.valueChanged.connect(self._emit_params)

    def set_values(self, distance: float, yaw: float, pitch: float) -> None:
        """MainWindow가 시작 시(또는 Reset 후) 현재 sim_worker 값으로 슬라이더를
        맞춰둘 때 호출.

        ★ setValue()를 호출하면 valueChanged가 다시 emit되어 _emit_params가 또
          불릴 수 있습니다 (순환은 아니지만 불필요한 apply_settings 호출이 한 번
          더 나갈 수 있음 — 지금 단계에선 무시해도 되는 수준).

        TODO:
            self.distance_slider.setValue(int(distance * self.DISTANCE_SCALE))
            self.yaw_slider.setValue(int(yaw))
            self.pitch_slider.setValue(int(pitch))
            (값 표시 라벨 3개도 같이 setText로 갱신)
        """
        self.distance_slider.setValue(int(distance * self.DISTANCE_SCALE))
        self.yaw_slider.setValue(int(yaw))
        self.pitch_slider.setValue(int(pitch))

    def _emit_params(self) -> None:
        """슬라이더 값이 바뀔 때마다 호출되는 내부 슬롯.

        TODO:
            distance = self.distance_slider.value() / self.DISTANCE_SCALE
            yaw = float(self.yaw_slider.value())
            pitch = float(self.pitch_slider.value())
            (값 표시 라벨 3개도 여기서 같이 setText로 갱신)
            self.params_changed.emit(distance, yaw, pitch)
        """
        distance = self.distance_slider.value() / self.DISTANCE_SCALE
        yaw = float(self.yaw_slider.value())
        pitch = float(self.pitch_slider.value())

        self.distance_value_label.setText(f"{distance:.1f}")
        self.yaw_value_label.setText(f"{yaw:.0f}")
        self.pitch_value_label.setText(f"{pitch:.0f}")

        self.params_changed.emit(distance, yaw, pitch)


class ConfThresholdPanel(QGroupBox):
    """YOLO 검출 confidence 임계값을 실시간으로 조절하는 패널.

    ★ config.CONF_THRESHOLD를 고치고 재시작하는 대신 데모 중 그 자리에서
      튜닝할 수 있게 함 — 카메라 도메인 미스매치 때문에 이 값이 조명/각도에
      따라 자주 움직이는 타겟이라(config.py 주석 참고).

    Signals:
        threshold_changed(float): 0.0~1.0. MainWindow가 받아서
            sim_worker.apply_settings(conf_threshold=...)를 호출.
    """

    threshold_changed = Signal(float)

    SCALE = 100  # 슬라이더 int 값 <-> 실제 threshold(float) 변환 배율

    def __init__(self) -> None:
        super().__init__("Confidence")  # ★ "Confidence Threshold"는 그룹박스 제목 중 가장 길어서
                                          #   옵션바 폭을 그거 하나가 늘려버림 -> 줄임

        self.threshold_slider = QSlider(Qt.Orientation.Horizontal)
        self.threshold_slider.setRange(0, 100)
        self.threshold_slider.setValue(int(config.CONF_THRESHOLD * self.SCALE))
        self.threshold_value_label = QLabel(f"{config.CONF_THRESHOLD:.2f}")

        row = QHBoxLayout()
        row.addWidget(self.threshold_slider)
        row.addWidget(self.threshold_value_label)
        row.addStretch(1)

        layout = QVBoxLayout(self)
        layout.addLayout(row)

        self.threshold_slider.valueChanged.connect(self._emit_threshold)

    def _emit_threshold(self, value: int) -> None:
        threshold = value / self.SCALE
        self.threshold_value_label.setText(f"{threshold:.2f}")
        self.threshold_changed.emit(threshold)


class TargetFpsPanel(QGroupBox):
    """대시보드 목표 fps(SimWorker._target_fps) 조절 패널.

    ★ 로봇팔이 이동 중이면 이 값 대신 MOTION_REPLAY_FPS가 우선 적용됨 —
      이 값은 "아무것도 안 움직이는(idle) 상태"의 캡처 상한만 결정함
      (gui/sim_worker.py의 run() 안 idle 캡처 게이트 참고).

    Signals:
        target_fps_changed(int): MainWindow가 받아서
            sim_worker.apply_settings(target_fps=...)를 호출.
    """

    target_fps_changed = Signal(int)

    def __init__(self) -> None:
        super().__init__("Target FPS")

        self.fps_spin = QSpinBox()
        self.fps_spin.setRange(5, 120)
        self.fps_spin.setValue(_DEFAULT_TARGET_FPS)

        layout = QHBoxLayout(self)
        layout.addWidget(self.fps_spin)
        layout.addStretch(1)

        self.fps_spin.valueChanged.connect(self.target_fps_changed.emit)


class BatchTimingPanel(QGroupBox):
    """배치 수집 대기 시간 / 작업영역 비움 판정 기준을 조절하는 패널.

    ★ integration/pipeline.py의 Pipeline 인스턴스 속성(batch_collection_sec,
      required_empty_detections)을 실시간 조절함. 둘 다 기본값은 모듈 상수
      (BATCH_COLLECTION_SEC/REQUIRED_EMPTY_DETECTIONS)에서 가져옴.

    Signals:
        batch_collection_sec_changed(float): 첫 물체 검출 후 추가 물체를 놓을 수
            있게 기다리는 시간(초).
        required_empty_detections_changed(int): "작업영역 비움" 판정까지 필요한
            연속 빈 검출 횟수. 실제 시간이 아니라 검출 사이클 횟수 기준이라
            (config.DETECT_EVERY_N_FRAMES에 따라 실제 걸리는 시간이 달라짐)
            초 단위 대신 횟수로 노출함.
    """

    batch_collection_sec_changed = Signal(float)
    required_empty_detections_changed = Signal(int)

    COLLECTION_SCALE = 10  # 슬라이더 int 값 <-> 실제 초(float) 변환 배율

    def __init__(self) -> None:
        super().__init__("Batch Timing")

        self.collection_slider = QSlider(Qt.Orientation.Horizontal)
        self.collection_slider.setRange(10, 150)  # 실제 1.0 ~ 15.0초
        self.collection_slider.setValue(
            int(_DEFAULT_BATCH_COLLECTION_SEC * self.COLLECTION_SCALE)
        )
        self.collection_value_label = QLabel(f"{_DEFAULT_BATCH_COLLECTION_SEC:.1f}s")

        collection_row = QHBoxLayout()
        collection_row.addWidget(self.collection_slider)
        collection_row.addWidget(self.collection_value_label)
        collection_row.addStretch(1)

        self.empty_detections_spin = QSpinBox()
        self.empty_detections_spin.setRange(1, 50)
        self.empty_detections_spin.setValue(_DEFAULT_REQUIRED_EMPTY_DETECTIONS)

        empty_row = QHBoxLayout()
        empty_row.addWidget(self.empty_detections_spin)
        empty_row.addStretch(1)

        layout = QFormLayout(self)
        layout.addRow("Collection Wait", collection_row)
        layout.addRow("Empty Detections", empty_row)

        self.collection_slider.valueChanged.connect(self._emit_collection_sec)
        self.empty_detections_spin.valueChanged.connect(
            self.required_empty_detections_changed.emit
        )

    def _emit_collection_sec(self, value: int) -> None:
        seconds = value / self.COLLECTION_SCALE
        self.collection_value_label.setText(f"{seconds:.1f}s")
        self.batch_collection_sec_changed.emit(seconds)


class SettingsPanel(QGroupBox):
    """대시보드 사용 중 자주 조절하는 값들(카메라 각도, 시뮬 해상도, 검출
    confidence 등)을 세로로 길게 모아둔 패널. 화면 가장 오른쪽에 전체 높이로
    배치해서 항상 접근 가능하게 함.

    ★ 하위 패널(target_fps/render_quality/camera_control/conf_threshold/batch_timing)은 각자의
      시그널을 그대로 노출함 — MainWindow가 self.settings_panel.camera_control처럼
      접근해서 연결. 조절 항목을 더 추가하고 싶으면 여기 __init__에 패널
      하나 만들어서 addWidget()만 하면 됨.
    """

    def __init__(self) -> None:
        super().__init__("Settings")

        self.target_fps = TargetFpsPanel()
        self.render_quality = RenderQualityPanel()
        self.camera_control = CameraControlPanel()
        self.conf_threshold = ConfThresholdPanel()
        self.batch_timing = BatchTimingPanel()

        layout = QVBoxLayout(self)
        layout.addWidget(self.target_fps)
        layout.addWidget(self.render_quality)
        layout.addWidget(self.camera_control)
        layout.addWidget(self.conf_threshold)
        layout.addWidget(self.batch_timing)
        layout.addStretch(1)  # 남는 세로 공간은 아래로 몰아서 패널들이 위로 붙게 함
