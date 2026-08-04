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
from PySide6.QtGui import QAction, QColor, QImage, QPainter, QPen, QPixmap, Qt
from PySide6.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QStyle,
    QVBoxLayout,
)

import config
from gui.panels import LogPanel, SettingsPanel, StatusPanel
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

    def __init__(
        self,
        placeholder_text: str = "",
        transformation_mode: Qt.TransformationMode = Qt.TransformationMode.FastTransformation,
    ) -> None:
        super().__init__(placeholder_text)
        self._original_pixmap: QPixmap | None = None
        self._transformation_mode = transformation_mode
        """FastTransformation: 매 프레임 스케일링 비용 절감(화질은 거칠어짐) — 웹캠처럼
        원본 해상도가 이미 높은 경우 기본값. SmoothTransformation: 원본이 저해상도라
        업스케일 티가 많이 날 때(예: 160x120 시뮬레이션 렌더) 화질 우선."""

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
                self._transformation_mode,
            ))


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Dashboard")
        self.resize(1280, 800)  # 시작 창 크기 (저장된 레이아웃 없을 때의 기본값)

        # QSettings는 OS별 표준 위치에 자동 저장됩니다 (Windows는 레지스트리).
        # 회사/앱 이름은 실제로 등록된 이름일 필요 없이, 이 앱만의 고유 키 역할만 함.
        self._settings = QSettings("RobotwinSorter", "RobotDashboard")

        # use_dummy=True: DummyDetector/DummyArmController로 안전하게 시작.
        # 실제 YOLO+로봇을 붙이려면 False로 바꾸세요 (모델/캘리브레이션 준비 필요).
        self.sim_worker = SimWorker(use_dummy=False)
        self.webcam_worker = WebcamWorker()

        self._stop_click_count = 0  # ★ 이스터에그: Stop 연속 10번 누르면 팀 소개 표시
        """Start를 누르면 0으로 리셋됨 -> "연속"이라는 의미가 성립함."""

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
        start_action.triggered.connect(self._on_start_clicked)
        toolbar.addAction(start_action)

        stop_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_MediaPause), "Stop", self)
        stop_action.triggered.connect(self._on_stop_clicked)
        toolbar.addAction(stop_action)

        reset_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_BrowserReload), "Reset", self)
        reset_action.triggered.connect(self.sim_worker.reset_simulation)
        toolbar.addAction(reset_action)

        toolbar.addSeparator()

        exit_action = QAction(style.standardIcon(QStyle.StandardPixmap.SP_TitleBarCloseButton), "Exit", self)
        exit_action.triggered.connect(self.close)
        toolbar.addAction(exit_action)

    def _on_start_clicked(self) -> None:
        """Start 버튼 클릭. Stop 연속 클릭 카운트를 리셋함 (연속성이 끊김)."""

        self._stop_click_count = 0
        self.sim_worker.start_simulation()

    def _on_stop_clicked(self) -> None:
        """Stop 버튼 클릭. ★ 이스터에그: 연속 10번 누르면 팀 소개 표시.

        Start를 누르거나 창을 재시작하면 카운트가 리셋되므로 "연속"으로만 발동함.
        """

        self.sim_worker.stop_simulation()

        self._stop_click_count += 1
        if self._stop_click_count >= 10:
            self._stop_click_count = 0
            self._show_team_intro()

    def _show_team_intro(self) -> None:
        QMessageBox.information(
            self,
            "팀 소개",
            "🤖 RoboTwin Sorter — 팀 \"304 Not Found\"\n\n"
            "김태익 — 로봇 제어 & 시뮬레이션\n"
            "윤주연 — 비전 (YOLO 모델)\n"
            "진선우 — 통합 & 파이프라인\n\n"
            "멘토: 진현철 대표님 (세중아이에스)\n\n"
            "Stop을 열 번이나 누르시다니... 그만큼 답답하셨다면 죄송합니다 🙏",
        )

    def _build_central_widget(self) -> None:
        """중앙 시뮬레이션 화면 + 웹캠 화면 + 로그/상태 패널 레이아웃."""

        # 시뮬레이션 화면은 저해상도 렌더(RenderQualityPanel로 조절)를 큰 패널로
        # 확대하는 거라 SmoothTransformation으로 화질을 우선함 (웹캠은 원본이
        # 640x480이라 기본값 유지).
        self.sim_view = AspectRatioLabel(
            "Waiting for simulation...", transformation_mode=Qt.TransformationMode.SmoothTransformation
        )
        self.fsm_label = QLabel("FSM: N/A", parent=self.sim_view)
        self.fsm_label.setStyleSheet("background-color: rgba(0, 0, 0, 128); color: white; padding: 2px;")
        self.fsm_label.move(10, 10)  # sim_view 오른쪽 위에 FSM 상태 표시

        self.webcam_view = AspectRatioLabel("Waiting for camera...")
        self.log_panel = LogPanel()
        self.status_panel = StatusPanel()
        self.settings_panel = SettingsPanel()

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
        self.log_status_splitter.addWidget(self.log_panel)
        self.log_status_splitter.addWidget(self.status_panel)

        self.main_splitter = QSplitter(Qt.Orientation.Vertical)
        self.main_splitter.addWidget(self.sim_webcam_splitter)
        self.main_splitter.addWidget(self.log_status_splitter)
        # 초기 비율(저장된 레이아웃이 없는 첫 실행 기준): 위(시뮬/웹캠)는 크게,
        # 아래(로그/상태)는 작게. _restore_layout()이 저장된 값이 있으면 이걸
        # 덮어씁니다.
        self.main_splitter.setSizes([600, 150])

        # settings_panel(카메라/해상도/confidence 등 조절 패널)을 화면 가장
        # 오른쪽에 전체 높이로 배치 — main_splitter 전체와 나란히 두는 가장
        # 바깥쪽 가로 splitter.
        self.root_splitter = QSplitter(Qt.Orientation.Horizontal, self)
        self.root_splitter.addWidget(self.main_splitter)
        self.root_splitter.addWidget(self.settings_panel)
        self.root_splitter.setSizes([1000, 260])
        self.setCentralWidget(self.root_splitter)

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
            ("layout/root_splitter", self.root_splitter),
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
        self._settings.setValue("layout/root_splitter", self.root_splitter.saveState())
        self._settings.setValue("layout/main_splitter", self.main_splitter.saveState())
        self._settings.setValue("layout/sim_webcam_splitter", self.sim_webcam_splitter.saveState())
        self._settings.setValue("layout/log_status_splitter", self.log_status_splitter.saveState())

    def _connect_signals(self) -> None:
        """워커 시그널 ↔ 위젯 슬롯 연결."""

        self.sim_worker.frame_ready.connect(self._on_sim_frame)
        self.sim_worker.state_changed.connect(self.status_panel.update_state)
        self.sim_worker.log_message.connect(self.log_panel.append_log)
        self.sim_worker.robot_state_changed.connect(self.fsm_label.setText)
        self.webcam_worker.frame_ready.connect(self._on_webcam_frame)
        self.webcam_worker.log_message.connect(self.log_panel.append_log)
        self.settings_panel.target_fps.target_fps_changed.connect(self._on_target_fps_changed)
        self.settings_panel.camera_control.params_changed.connect(self._on_camera_params_changed)
        self.settings_panel.render_quality.resolution_changed.connect(self._on_resolution_changed)
        self.settings_panel.conf_threshold.threshold_changed.connect(self._on_conf_threshold_changed)

        # 웹캠 스레드 -> Sim 스레드로 원본 프레임을 직접 전달 (GUI 스레드 안 거침).
        # SimWorker는 자체 이벤트 루프(exec())를 안 돌리므로 큐잉 연결이 아니라
        # DirectConnection으로 강제해야 콜백이 실제로 호출됨 — set_latest_frame()은
        # 참조 대입만 하는 가벼운 메서드라 다른 스레드에서 직접 불러도 안전함.
        self.webcam_worker.raw_frame_ready.connect(
            self.sim_worker.set_latest_frame, Qt.ConnectionType.DirectConnection
        )

    def _on_target_fps_changed(self, target_fps: int) -> None:
        """TargetFpsPanel.target_fps_changed 시그널에 연결될 슬롯."""

        self.sim_worker.apply_settings(target_fps=target_fps)

    def _on_camera_params_changed(self, distance: float, yaw: float, pitch: float) -> None:
        """CameraControlPanel.params_changed 시그널에 연결될 슬롯."""

        self.sim_worker.apply_settings(distance=distance, yaw=yaw, pitch=pitch)

    def _on_resolution_changed(self, width: int, height: int) -> None:
        """RenderQualityPanel.resolution_changed 시그널에 연결될 슬롯."""

        self.sim_worker.apply_settings(frame_width=width, frame_height=height)

    def _on_conf_threshold_changed(self, threshold: float) -> None:
        """ConfThresholdPanel.threshold_changed 시그널에 연결될 슬롯."""

        self.sim_worker.apply_settings(conf_threshold=threshold)

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
        """웹캠 원본 프레임 위에 SimWorker(Pipeline)가 가장 최근에 계산해둔 검출
        결과를 bbox로 겹쳐 그린 뒤 표시.

        ★ 여기서 YOLO를 다시 돌리지 않음 — SimWorker가 이미 step_cycle() 안에서
          계산해 둔 sim_worker.last_detections를 재사용만 함. bbox 좌표는
          640x480(캡처 해상도) 기준이라 이 QImage에 그대로 그리면 맞고,
          AspectRatioLabel이 그 이후 화면 크기에 맞게 통째로 다시 스케일링해줌.
        ★ image는 WebcamWorker.run()에서 이미 .copy()해서 보낸, 여기서만 참조하는
          QImage라 바로 그려도 됨(다른 곳과 공유되는 버퍼가 아님).
        """
        detections = self.sim_worker.last_detections
        if detections:
            painter = QPainter(image)
            for det in detections:
                x1, y1, x2, y2 = det.bbox
                r, g, b, a = config.CATEGORY_COLORS[det.category]
                painter.setPen(QPen(QColor.fromRgbF(r, g, b, a), 2))
                painter.drawRect(x1, y1, x2 - x1, y2 - y1)
                painter.drawText(x1, max(y1 - 6, 10), f"{det.category} {det.confidence:.2f}")
            painter.end()

        self.webcam_view.setPixmap(QPixmap.fromImage(image))

    def closeEvent(self, event) -> None:
        """창 닫을 때 워커 스레드 정리.

        ★ 워커가 살아있는 채로 창을 닫으면 pybullet 클라이언트가 정리되지 않고
          프로세스가 남을 수 있습니다. stop 계열 메서드 호출 후 wait()로 스레드
          종료까지 기다린 다음 super().closeEvent(event)를 호출하세요.

        ★ sim_worker는 로봇팔이 한창 동작 중(move_to() 타임아웃 최대 5초 x
          최대 6번)이면 wait()가 수십 초까지 블로킹할 수 있습니다. 타임아웃을
          줘서 무한정 멈추지 않게 하고, statusBar 메시지로 "종료 중"임을
          알립니다 (showMessage() 직후 processEvents()를 안 부르면 뒤이은
          wait()가 이벤트 루프를 막아버려서 이 메시지가 화면에 그려지지도
          못한 채 창이 멈춘 것처럼 보입니다).
        """
        self._save_layout()

        self.statusBar().showMessage("시뮬레이션 종료 대기 중...")
        QApplication.processEvents()

        self.sim_worker.shutdown()
        if not self.sim_worker.wait(10000):
            print("[main_window] sim_worker가 10초 내에 안 끝났습니다 — 프로세스 종료로 정리됩니다.")
        self.webcam_worker.stop_capture()
        self.webcam_worker.wait(3000)
        super().closeEvent(event)
