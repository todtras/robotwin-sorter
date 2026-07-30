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

from PySide6.QtGui import QAction, QImage, QPixmap, Qt
from PySide6.QtWidgets import QLabel, QMainWindow, QMessageBox, QSplitter

from gui.panels import LogPanel, StatusPanel
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
                Qt.TransformationMode.SmoothTransformation,
            ))


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Dashboard")
        self.resize(1280, 800)  # 시작 창 크기

        self.sim_worker = SimWorker()
        self.webcam_worker = WebcamWorker()

        self._build_menu()
        self._build_central_widget()
        self._connect_signals()

        # TODO: 웹캠은 시뮬레이션 Start와 무관하게 창을 열자마자 바로 촬영을
        #       시작하는 게 자연스럽습니다: self.webcam_worker.start_capture()
        self.webcam_worker.start_capture()

    def _build_menu(self) -> None:
        """메뉴바 구성: Start / Stop / Reset / Settings / Exit."""

        sim_menu = self.menuBar().addMenu("Simulation")

        start_action = QAction("Start", self)
        start_action.triggered.connect(self.sim_worker.start_simulation)
        sim_menu.addAction(start_action)

        stop_action = QAction("Stop", self)
        stop_action.triggered.connect(self.sim_worker.stop_simulation)
        sim_menu.addAction(stop_action)

        reset_action = QAction("Reset", self)
        reset_action.triggered.connect(self.sim_worker.reset_simulation)
        sim_menu.addAction(reset_action)

        settings_action = QAction("Settings", self)
        settings_action.triggered.connect(lambda: QMessageBox.information(self, "Settings", "준비 중"))
        sim_menu.addAction(settings_action)

        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        sim_menu.addAction(exit_action)

    def _build_central_widget(self) -> None:
        """중앙 시뮬레이션 화면 + 웹캠 화면 + 로그/상태 패널 레이아웃."""

        self.sim_view = AspectRatioLabel("Waiting for simulation...")
        self.webcam_view = AspectRatioLabel("Waiting for camera...")
        self.log_panel = LogPanel()
        self.status_panel = StatusPanel()

        sim_webcam_splitter = QSplitter(Qt.Orientation.Horizontal)
        sim_webcam_splitter.addWidget(self.sim_view)
        sim_webcam_splitter.addWidget(self.webcam_view)

        log_status_splitter = QSplitter(Qt.Orientation.Horizontal)
        log_status_splitter.addWidget(self.log_panel)
        log_status_splitter.addWidget(self.status_panel)

        main_splitter = QSplitter(Qt.Orientation.Vertical, self)
        main_splitter.addWidget(sim_webcam_splitter)
        main_splitter.addWidget(log_status_splitter)
        # 초기 비율: 위(시뮬/카메라)는 크게, 아래(로그/상태)는 작게.
        # setSizes는 "시작할 때"만 이 픽셀 비율로 나누고, 이후엔 사용자가 드래그한
        # 크기가 우선입니다 (창을 다시 열 때마다 이 비율로 리셋됨).
        main_splitter.setSizes([600, 150])
        self.setCentralWidget(main_splitter)

    def _connect_signals(self) -> None:
        """워커 시그널 ↔ 위젯 슬롯 연결."""

        self.sim_worker.frame_ready.connect(self._on_sim_frame)
        self.sim_worker.state_changed.connect(self.status_panel.update_state)
        self.sim_worker.log_message.connect(self.log_panel.append_log)
        self.webcam_worker.frame_ready.connect(self._on_webcam_frame)
        self.webcam_worker.log_message.connect(self.log_panel.append_log)

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
        self.sim_worker.shutdown()
        self.sim_worker.wait()
        self.webcam_worker.stop_capture()
        self.webcam_worker.wait()
        super().closeEvent(event)
