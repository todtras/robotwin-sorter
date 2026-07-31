"""
gui/main_window.py — Robot Dashboard 메인 윈도우

Qt 기초:
- QMainWindow: 메뉴바/툴바/상태바가 딸린 기본 창 틀. `self.menuBar()`로 메뉴바를
  얻고, `self.setCentralWidget(widget)`으로 가운데 영역을 채웁니다.
- QAction: 메뉴 항목 하나를 나타내는 객체. `action.triggered.connect(함수)`로
  클릭됐을 때 실행될 함수(슬롯)를 연결합니다.
- Signal ↔ Slot 연결: `어떤_시그널.connect(받을_함수)` 형태로 한 번만 등록해두면,
  이후 그 시그널이 emit()될 때마다 자동으로 받을_함수가 호출됩니다.

★ 이 클래스는 pybullet/cv2를 직접 호출하면 안 됩니다. 모든 작업은 SimWorker /
  WebcamWorker에게 메서드 호출로 명령하고, 결과(frame/state/log)는 시그널로 받아서
  화면에 그리기만 합니다.

Start / Stop / Reset은 시뮬레이션에만 적용됩니다 (웹캠은 창이 열려있는 동안 항상 촬영):
    Start — 정지 상태면 새로 시작, 일시정지 상태면 그 지점부터 재개
    Stop  — 일시정지 (상태 보존)
    Reset — 완전히 새로 시작 (상태 초기화)
"""

from __future__ import annotations

from PySide6.QtCore import QSettings
from PySide6.QtGui import QAction, QImage, QPixmap, Qt
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QSplitter,
    QStyle,
    QVBoxLayout,
)

from gui.panels import CameraControlPanel, LogPanel, StatusPanel
from gui.sim_worker import SimWorker
from gui.webcam_worker import WebcamWorker


class AspectRatioLabel(QLabel):
    """비율을 유지한 채 라벨 크기에 맞춰 이미지를 자동으로 리사이즈하는 QLabel.

    일반 QLabel.setPixmap()은 이미지를 원본 크기 그대로 그리기 때문에, splitter로
    라벨 크기를 바꿔도 이미지는 그대로입니다. 여기서는 원본 pixmap을 따로 저장해두고
    "새 이미지가 들어왔을 때"와 "라벨 크기가 바뀌었을 때"(resizeEvent) 둘 다 같은
    _rescale()로 다시 그려서, splitter를 드래그하는 즉시(새 프레임이 없어도)
    비율을 유지한 채 크기가 따라오게 만듭니다.
    """

    def __init__(self, placeholder_text: str = "") -> None:
        super().__init__(placeholder_text)
        self._original_pixmap: QPixmap | None = None

    def setPixmap(self, pixmap: QPixmap) -> None:
        self._original_pixmap = pixmap
        self._rescale()

    def resizeEvent(self, event) -> None:
        self._rescale()
        super().resizeEvent(event)

    def _rescale(self) -> None:
        if self._original_pixmap is not None:
            super().setPixmap(self._original_pixmap.scaled(
                self.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,  # 매 프레임 스케일링 비용 절감(화질은 약간 거칠어짐)
            ))


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Dashboard")
        self.resize(1280, 800)  # 시작 창 크기 (저장된 레이아웃 없을 때의 기본값)

        # QSettings는 OS별 표준 위치에 자동 저장됩니다 (Windows는 레지스트리).
        # 회사/앱 이름은 실제로 등록된 이름일 필요 없이, 이 앱만의 고유 키 역할만 함.
        self._settings = QSettings("RobotwinSorter", "RobotDashboard")

        self.sim_worker = SimWorker()
        self.webcam_worker = WebcamWorker()

        self._build_toolbar()
        self._build_central_widget()
        self._connect_signals()
        self._restore_layout()

        self.webcam_worker.start_capture()

    def _build_toolbar(self) -> None:
        """Start / Stop / Reset / Exit을 항상 보이는 툴바에 배치.

        메뉴바(QMenuBar.addMenu)는 "Simulation"을 눌러야 펼쳐지는 드롭다운이라,
        매번 클릭 한 번이 더 필요합니다. QToolBar는 액션들이 버튼으로 항상 노출돼
        바로 클릭할 수 있습니다.
        """
        toolbar = self.addToolBar("Simulation")
        toolbar.setMovable(False)
        style = self.style()  # QStyle.StandardPixmap 아이콘은 별도 이미지 파일 없이 사용 가능

        start_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_MediaPlay), "Start", self)
        start_action.triggered.connect(self.sim_worker.start_simulation)
        toolbar.addAction(start_action)

        stop_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_MediaPause), "Stop", self)
        stop_action.triggered.connect(self.sim_worker.stop_simulation)
        toolbar.addAction(stop_action)

        reset_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_BrowserReload), "Reset", self)
        reset_action.triggered.connect(self.sim_worker.reset_simulation)
        toolbar.addAction(reset_action)

        toolbar.addSeparator()

        exit_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_TitleBarCloseButton), "Exit", self)
        exit_action.triggered.connect(self.close)
        toolbar.addAction(exit_action)

    def _build_central_widget(self) -> None:
        """중앙 시뮬레이션 화면 + 웹캠 화면 + 로그/상태 패널 레이아웃."""

        self.sim_view = AspectRatioLabel("Waiting for simulation...")
        self.webcam_view = AspectRatioLabel("Waiting for camera...")
        self.log_panel = LogPanel()
        self.status_panel = StatusPanel()
        self.camera_control_panel = CameraControlPanel()

        sim_group = QGroupBox("Simulation")
        QVBoxLayout(sim_group).addWidget(self.sim_view)

        # 팀원마다 웹캠 장치 번호(config.CAMERA_INDEX)가 다를 수 있어서(외장캠 유무 등),
        # config.py를 직접 고치는 대신 대시보드에서만 즉석으로 바꿔볼 수 있게 함.
        self.camera_index_spin = QSpinBox()
        self.camera_index_spin.setRange(0, 10)
        self.camera_index_spin.setValue(self.webcam_worker.get_camera_index())
        self.reconnect_button = QPushButton("Reconnect")
        self.reconnect_button.clicked.connect(self._reconnect_webcam)

        camera_index_row = QHBoxLayout()
        camera_index_row.addWidget(QLabel("Index"))
        camera_index_row.addWidget(self.camera_index_spin)
        camera_index_row.addWidget(self.reconnect_button)
        camera_index_row.addStretch(1)

        webcam_group = QGroupBox("Webcam")
        webcam_layout = QVBoxLayout(webcam_group)
        webcam_layout.addLayout(camera_index_row)
        webcam_layout.addWidget(self.webcam_view)

        self.sim_webcam_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.sim_webcam_splitter.addWidget(sim_group)
        self.sim_webcam_splitter.addWidget(webcam_group)

        self.log_status_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.log_status_splitter.addWidget(self.camera_control_panel)
        self.log_status_splitter.addWidget(self.log_panel)
        self.log_status_splitter.addWidget(self.status_panel)

        self.main_splitter = QSplitter(Qt.Orientation.Vertical, self)
        self.main_splitter.addWidget(self.sim_webcam_splitter)
        self.main_splitter.addWidget(self.log_status_splitter)
        # 초기 비율(저장된 레이아웃이 없는 첫 실행 기준): 위(시뮬/카메라)는 크게,
        # 아래(로그/상태)는 작게. _restore_layout()이 저장된 값이 있으면 이걸
        # 덮어씁니다.
        self.main_splitter.setSizes([600, 150])
        self.setCentralWidget(self.main_splitter)

    def _restore_layout(self) -> None:
        """이전에 저장해둔 창 크기/splitter 비율이 있으면 복원.

        QByteArray를 그대로 저장/복원하는 saveGeometry()/restoreGeometry(),
        splitter의 saveState()/restoreState()를 씁니다. 저장된 값이 없으면
        (맨 처음 실행) _build_central_widget()에서 이미 정해둔 기본값을 그대로 둠.
        """
        geometry = self._settings.value("window/geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)

        for key, splitter in (
            ("layout/main_splitter", self.main_splitter),
            ("layout/sim_webcam_splitter", self.sim_webcam_splitter),
            ("layout/log_status_splitter", self.log_status_splitter),
        ):
            state = self._settings.value(key)
            if state is not None:
                splitter.restoreState(state)

    def _save_layout(self) -> None:
        """창 닫을 때 지금 창 크기/splitter 비율을 저장. 다음 실행 때 _restore_layout()이
        이 값을 읽어서 그대로 복원함."""
        self._settings.setValue("window/geometry", self.saveGeometry())
        self._settings.setValue("layout/main_splitter", self.main_splitter.saveState())
        self._settings.setValue("layout/sim_webcam_splitter", self.sim_webcam_splitter.saveState())
        self._settings.setValue("layout/log_status_splitter", self.log_status_splitter.saveState())

    def _connect_signals(self) -> None:
        """워커 시그널 ↔ 위젯 슬롯 연결."""

        self.sim_worker.frame_ready.connect(self._on_sim_frame)
        self.sim_worker.state_changed.connect(self.status_panel.update_state)
        self.sim_worker.log_message.connect(self.log_panel.append_log)
        self.webcam_worker.frame_ready.connect(self._on_webcam_frame)
        self.webcam_worker.log_message.connect(self.log_panel.append_log)
        self.camera_control_panel.params_changed.connect(self._on_camera_params_changed)

    def _on_camera_params_changed(self, distance: float, yaw: float, pitch: float) -> None:
        """CameraControlPanel.params_changed 시그널에 연결될 슬롯."""

        self.sim_worker.apply_settings(distance=distance, yaw=yaw, pitch=pitch)

    def _reconnect_webcam(self) -> None:
        """Reconnect 버튼 클릭 시 호출. 새 인덱스로 웹캠 스레드를 재시작.

        ★ cv2.VideoCapture는 스레드가 run() 시작할 때 한 번만 여니까, 인덱스만
          바꿔서는 반영이 안 됩니다. stop_capture() -> wait()(스레드가 실제로
          끝날 때까지 대기) -> set_camera_index() -> start_capture() 순서로
          완전히 재시작해야 합니다. wait()는 GUI 스레드를 잠깐 블로킹하지만,
          "카메라 다시 연결"은 사용자가 버튼을 누른 즉시 반응하는 액션이라
          이 정도 지연은 자연스럽습니다.
        """
        new_index = self.camera_index_spin.value()
        self.webcam_worker.stop_capture()
        self.webcam_worker.wait()
        self.webcam_worker.set_camera_index(new_index)
        self.webcam_worker.start_capture()

    def _on_sim_frame(self, image: QImage) -> None:
        """AspectRatioLabel이 비율 유지 스케일링을 알아서 처리하니 그냥 넘기기만 하면 됨."""

        self.sim_view.setPixmap(QPixmap.fromImage(image))

    def _on_webcam_frame(self, image: QImage) -> None:
        """_on_sim_frame과 동일."""
        self.webcam_view.setPixmap(QPixmap.fromImage(image))

    def closeEvent(self, event) -> None:
        """창 닫을 때 워커 스레드 정리.

        ★ 워커가 살아있는 채로 창을 닫으면 pybullet 클라이언트가 정리되지 않고
          프로세스가 남을 수 있습니다. stop 계열 메서드 호출 후 wait()로 스레드
          종료까지 기다린 다음 super().closeEvent(event)를 호출하세요.
        """
        self._save_layout()
        self.sim_worker.shutdown()
        self.sim_worker.wait()
        self.webcam_worker.stop_capture()
        self.webcam_worker.wait()
        super().closeEvent(event)
