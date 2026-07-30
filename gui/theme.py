"""
gui/theme.py — 대시보드 다크 테마 스타일시트

QSS(Qt Style Sheets)는 CSS와 문법이 거의 같습니다. `app.setStyleSheet(DARK_STYLESHEET)`
한 번만 호출하면 이 안의 규칙이 모든 위젯에 재귀적으로 적용됩니다(자식 위젯이 따로
스타일시트를 안 가지고 있으면 부모 걸 그대로 물려받음).
"""

from __future__ import annotations

DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #2b2b2b;
    color: #e0e0e0;
}

QToolBar {
    background-color: #232323;
    border: none;
    spacing: 6px;
    padding: 4px;
}

QToolButton {
    background-color: #3a3a3a;
    border: 1px solid #4a4a4a;
    border-radius: 4px;
    padding: 6px 10px;
    color: #e0e0e0;
}

QToolButton:hover {
    background-color: #4a4a4a;
    border: 1px solid #6a9fd8;
}

QToolButton:pressed {
    background-color: #2f2f2f;
}

QGroupBox {
    border: 1px solid #4a4a4a;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 10px;
    font-weight: bold;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: #6a9fd8;
}

QPlainTextEdit {
    background-color: #1e1e1e;
    color: #d0d0d0;
    border: 1px solid #3a3a3a;
    border-radius: 4px;
}

QSlider::groove:horizontal {
    height: 4px;
    background: #4a4a4a;
    border-radius: 2px;
}

QSlider::handle:horizontal {
    background: #6a9fd8;
    width: 14px;
    margin: -6px 0;
    border-radius: 7px;
}

QSplitter::handle {
    background-color: #3a3a3a;
}

QSplitter::handle:hover {
    background-color: #6a9fd8;
}
"""
