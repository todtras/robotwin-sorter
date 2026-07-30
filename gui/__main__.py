"""
gui/__main__.py — 대시보드 실행 진입점

실행::
    python -m gui
"""

from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
