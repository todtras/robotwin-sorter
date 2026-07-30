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
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QSlider,
    QVBoxLayout,
)

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
    """FPS / step / cube 위치 등 상태 값을 보여주는 패널.

    지금은 sim_worker.py의 데모 씬(낙하 큐브) 값을 그대로 표시하면 됩니다.
    나중에 Pipeline이 연결되면 state dict의 키(FSM 상태, joint 정보 등)만
    바뀌고 이 위젯 구조는 그대로 재사용할 수 있습니다.
    """

    def __init__(self) -> None:
        super().__init__("Status")

        self.fps_label = QLabel("-")
        self.step_label = QLabel("-")
        self.cube_z_label = QLabel("-")

        layout = QFormLayout(self)
        layout.addRow("FPS", self.fps_label)
        layout.addRow("Step", self.step_label)
        layout.addRow("Cube Z", self.cube_z_label)

    def update_state(self, state: dict) -> None:
        """SimWorker.state_changed 시그널에 연결될 슬롯.

        state는 {"fps": float, "step": int, "cube_z": float} 형태로 들어옵니다.
        """

        self.fps_label.setText(f"{state.get('fps', 0):.1f}")
        self.step_label.setText(str(state.get("step", 0)))
        self.cube_z_label.setText(f"{state.get('cube_z', 0):.3f}")


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

        self.distance_slider = QSlider(Qt.Orientation.Horizontal)
        self.distance_slider.setRange(1, 100)  # 실제 0.1 ~ 10.0
        self.distance_slider.setValue(int(1.5 * self.DISTANCE_SCALE))
        self.distance_value_label = QLabel("1.5")

        self.yaw_slider = QSlider(Qt.Orientation.Horizontal)
        self.yaw_slider.setRange(-180, 180)
        self.yaw_slider.setValue(45)
        self.yaw_value_label = QLabel("45") 

        self.pitch_slider = QSlider(Qt.Orientation.Horizontal)
        self.pitch_slider.setRange(-90, 90)
        self.pitch_slider.setValue(-30)
        self.pitch_value_label = QLabel("-30")

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
