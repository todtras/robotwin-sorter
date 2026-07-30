"""
gui/__main__.py — 대시보드 실행 진입점

실행::
    python -m gui
"""

from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow
from gui.theme import DARK_STYLESHEET


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
