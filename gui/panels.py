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

from PySide6.QtWidgets import QFormLayout, QLabel, QPlainTextEdit, QVBoxLayout, QWidget

MAX_LOG_LINES = 1000


class LogPanel(QWidget):
    """시뮬레이션 이벤트/에러를 텍스트로 쌓아 보여주는 패널."""

    def __init__(self) -> None:
        super().__init__()
        self.text_edit = QPlainTextEdit()
        self.text_edit.setReadOnly(True)
        # TODO: QVBoxLayout(self)를 만들고 layout.addWidget(self.text_edit) 호출
        layout = QVBoxLayout(self)
        layout.addWidget(self.text_edit)

    def append_log(self, message: str) -> None:
        """SimWorker/WebcamWorker의 log_message 시그널에 연결될 슬롯.

        TODO: self.text_edit.appendPlainText(message)

        ★ 로그가 무한정 쌓이면 메모리 문제가 생기니, 여유가 되면
          self.text_edit.document().blockCount()가 MAX_LOG_LINES를 넘을 때
          오래된 줄부터 지우는 것도 고려해보세요 (QTextCursor로 앞부분을 선택해서
          removeSelectedText() 하면 됩니다). 처음엔 안 해도 동작에는 문제없습니다.
        """
        self.text_edit.appendPlainText(message)


class StatusPanel(QWidget):
    """FPS / step / cube 위치 등 상태 값을 보여주는 패널.

    지금은 sim_worker.py의 데모 씬(낙하 큐브) 값을 그대로 표시하면 됩니다.
    나중에 Pipeline이 연결되면 state dict의 키(FSM 상태, joint 정보 등)만
    바뀌고 이 위젯 구조는 그대로 재사용할 수 있습니다.
    """

    def __init__(self) -> None:
        super().__init__()
        # TODO: FPS/step/cube_z 각각 보여줄 QLabel을 만들고 (예: self.fps_label = QLabel("-")),
        #       QFormLayout(self)에 layout.addRow("FPS", self.fps_label) 식으로 추가하세요.
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

        TODO: state.get("fps"), state.get("step"), state.get("cube_z")를
              각 QLabel의 setText(...)로 반영하세요. (숫자는 str()이나
              f"{값:.1f}"로 문자열 변환해야 setText에 넘길 수 있습니다)
        """
        self.fps_label.setText(f"{state.get('fps', 0):.1f}")
        self.step_label.setText(str(state.get("step", 0)))
        self.cube_z_label.setText(f"{state.get('cube_z', 0):.3f}")
