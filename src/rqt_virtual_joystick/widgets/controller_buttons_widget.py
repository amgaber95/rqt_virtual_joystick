from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Set, Tuple

from python_qt_binding.QtCore import Qt, QSize, QPoint, pyqtSignal
from python_qt_binding.QtGui import (
    QColor,
    QPainter,
    QPen,
    QBrush,
    QRadialGradient,
    QFont,
)
from python_qt_binding.QtWidgets import QAbstractButton, QGridLayout, QSizePolicy, QWidget


class GameButton(IntEnum):
    A = 0
    B = 1
    X = 2
    Y = 3


LABELS: Dict[GameButton, str] = {
    GameButton.A: "A",
    GameButton.B: "B",
    GameButton.X: "X",
    GameButton.Y: "Y",
}


COLORS: Dict[GameButton, QColor] = {
    GameButton.A: QColor(120, 255, 120),  # Green
    GameButton.B: QColor(255, 100, 100),  # Red
    GameButton.X: QColor(100, 150, 255),  # Blue
    GameButton.Y: QColor(255, 220, 100),  # Yellow
}


BUTTON_LAYOUT: Tuple[Tuple[GameButton, int, int], ...] = (
    (GameButton.Y, 0, 1),
    (GameButton.X, 1, 0),
    (GameButton.B, 1, 2),
    (GameButton.A, 2, 1),
)


@dataclass
class ButtonStyle:
    base: QColor
    border_pen: QPen
    ring_pen: QPen
    font: QFont


def _build_style(base_color: QColor) -> ButtonStyle:
    border_pen = QPen(base_color.darker(150), 2)
    ring_pen = QPen(base_color.lighter(170), 3)
    ring_pen.setJoinStyle(Qt.RoundJoin)

    font = QFont()
    font.setPointSize(15)
    font.setBold(True)

    return ButtonStyle(
        base=base_color,
        border_pen=border_pen,
        ring_pen=ring_pen,
        font=font,
    )


class ControllerButton(QAbstractButton):
    """Controller face button with optional sticky (toggle) mode."""

    def __init__(self, btn_id: GameButton, parent: QWidget | None = None):
        super().__init__(parent)
        self._id = btn_id
        self._style = _build_style(COLORS[btn_id])
        self._sticky = False

        self.setCheckable(True)
        self.setChecked(False)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setAccessibleName(f"Controller button {LABELS[btn_id]}")
        self.setToolTip(LABELS[btn_id])

        # Momentary behaviour uses pressed/released to mirror legacy semantics.
        self.pressed.connect(self._on_pressed)
        self.released.connect(self._on_released)

    # ------------------------------------------------------------------
    # Qt overrides
    # ------------------------------------------------------------------

    def sizeHint(self) -> QSize:  # type: ignore[override]
        return QSize(45, 45)

    def minimumSizeHint(self) -> QSize:  # type: ignore[override]
        return QSize(30, 30)

    def nextCheckState(self) -> None:  # type: ignore[override]
        # Qt toggles on release; skip automatic toggle in momentary mode.
        if self._sticky:
            super().nextCheckState()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        center = QPoint(self.width() // 2, self.height() // 2)
        radius = max(0, min(self.width(), self.height()) // 2 - 2)
        pressed = self.isDown() or self.isChecked()

        self._draw_shadow(painter, center, radius, pressed)
        self._draw_body(painter, center, radius, pressed)
        self._draw_highlight(painter, center, radius, pressed)
        if pressed:
            self._draw_pressed_overlay(painter, center, radius)
        self._draw_focus_ring(painter, center, radius)
        self._draw_text(painter, center, pressed)

    # ------------------------------------------------------------------
    # Painting primitives
    # ------------------------------------------------------------------

    def _draw_shadow(self, painter: QPainter, center: QPoint, radius: int, pressed: bool) -> None:
        if pressed or radius <= 0:
            return
        painter.save()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 60)))
        shadow_offset = 2
        shadow_center = QPoint(center.x() + shadow_offset, center.y() + shadow_offset)
        painter.drawEllipse(shadow_center, radius + 1, radius + 1)
        painter.restore()

    def _draw_body(self, painter: QPainter, center: QPoint, radius: int, pressed: bool) -> None:
        painter.save()
        gradient_center = QPoint(center)
        if pressed:
            gradient_center += QPoint(2, 2)
            gradient = QRadialGradient(gradient_center, radius)
            gradient.setColorAt(0.0, self._style.base.darker(130))
            gradient.setColorAt(0.7, self._style.base.darker(110))
            gradient.setColorAt(1.0, self._style.base.darker(150))
            pen_color = self._style.base.darker(180)
        else:
            gradient_center += QPoint(-3, -3)
            gradient = QRadialGradient(gradient_center, radius)
            gradient.setColorAt(0.0, self._style.base.lighter(150))
            gradient.setColorAt(0.5, self._style.base)
            gradient.setColorAt(1.0, self._style.base.darker(120))
            pen_color = self._style.base.darker(150)

        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(pen_color, self._style.border_pen.width()))
        draw_center = QPoint(center)
        if pressed:
            draw_center += QPoint(1, 1)
        painter.drawEllipse(draw_center, radius, radius)
        painter.restore()

    def _draw_highlight(self, painter: QPainter, center: QPoint, radius: int, pressed: bool) -> None:
        if pressed:
            return
        highlight_radius = radius - 6
        if highlight_radius <= 0:
            return
        painter.save()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(255, 255, 255, 80)))
        highlight_center = QPoint(center.x() - 2, center.y() - 2)
        painter.drawEllipse(highlight_center, highlight_radius, highlight_radius)
        painter.restore()

    def _draw_pressed_overlay(self, painter: QPainter, center: QPoint, radius: int) -> None:
        painter.save()
        draw_center = QPoint(center.x() + 1, center.y() + 1)
        inner_radius = max(0, radius - 5)
        if inner_radius > 0:
            glow = QRadialGradient(draw_center, inner_radius)
            glow.setColorAt(0.0, QColor(255, 255, 255, 160))
            glow.setColorAt(0.5, self._style.base.lighter(150))
            glow.setColorAt(1.0, QColor(self._style.base.red(), self._style.base.green(), self._style.base.blue(), 0))
            painter.setBrush(QBrush(glow))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(draw_center, inner_radius, inner_radius)

        painter.setBrush(Qt.NoBrush)
        painter.setPen(self._style.ring_pen)
        painter.drawEllipse(draw_center, radius, radius)
        painter.restore()

    def _draw_focus_ring(self, painter: QPainter, center: QPoint, radius: int) -> None:
        if not self.hasFocus() or radius <= 0:
            return
        painter.save()
        focus_pen = QPen(QColor(255, 255, 255, 180), 2, Qt.DotLine)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(focus_pen)
        painter.drawEllipse(center, radius + 2, radius + 2)
        painter.restore()

    def _draw_text(self, painter: QPainter, center: QPoint, pressed: bool) -> None:
        painter.save()
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setBrush(Qt.NoBrush)
        painter.setFont(self._style.font)

        label = LABELS[self._id]
        rect = painter.fontMetrics().boundingRect(label)
        offset = QPoint(1, 1) if pressed else QPoint(0, 0)
        text_pos = QPoint(
            center.x() - rect.width() // 2 + offset.x(),
            center.y() + rect.height() // 2 - 2 + offset.y(),
        )
        painter.drawText(text_pos, label)
        painter.restore()

    # ------------------------------------------------------------------
    # Behaviour helpers
    # ------------------------------------------------------------------

    def button_id(self) -> GameButton:
        return self._id

    def reset(self) -> None:
        self.setChecked(False)
        self.setDown(False)

    def set_sticky(self, enabled: bool) -> None:
        self._sticky = bool(enabled)
        if not self._sticky:
            self.setChecked(False)

    def _on_pressed(self) -> None:
        if not self._sticky:
            if not self.isChecked():
                self.setChecked(True)

    def _on_released(self) -> None:
        if not self._sticky:
            if self.isChecked():
                self.setChecked(False)


class ControllerButtonsWidget(QWidget):
    """Group of controller buttons arranged in a diamond layout."""

    button_toggled = pyqtSignal(int, bool)
    button_state_changed = pyqtSignal(int, bool)  # emits toggled state

    def __init__(self, parent: QWidget | None = None, sticky_buttons: bool = False):
        super().__init__(parent)
        self._layout = QGridLayout(self)
        self._layout.setSpacing(2)
        self._layout.setContentsMargins(8, 8, 8, 8)

        self._buttons: Dict[GameButton, ControllerButton] = {}
        self._pressed: Set[GameButton] = set()

        self._init_ui()
        self.set_sticky_buttons(sticky_buttons)

    # ------------------------------------------------------------------
    # Layout & paint
    # ------------------------------------------------------------------

    def _init_ui(self) -> None:
        for row in range(3):
            self._layout.setRowStretch(row, 1)
        for column in range(3):
            self._layout.setColumnStretch(column, 1)

        for btn_id, row, column in BUTTON_LAYOUT:
            button = ControllerButton(btn_id, self)
            button.setText(LABELS[btn_id])
            button.toggled.connect(lambda checked, b=btn_id: self._on_button_toggled(b, checked))

            if btn_id == GameButton.B:
                self._layout.addWidget(button, row, column, Qt.AlignLeft)
            elif btn_id == GameButton.X:
                self._layout.addWidget(button, row, column, Qt.AlignRight)
            elif btn_id == GameButton.Y:
                self._layout.addWidget(button, row, column, Qt.AlignHCenter)
            elif btn_id == GameButton.A:
                self._layout.addWidget(button, row, column, Qt.AlignHCenter)
            self._buttons[btn_id] = button
        self.setFixedSize(200, 200)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        size = min(width, height)
        center = QPoint(width // 2, height // 2)
        radius = max(0, size // 2 - 10)
        self._draw_background(painter, center, radius)

    def _draw_background(self, painter: QPainter, center: QPoint, radius: int) -> None:
        if radius <= 0:
            return

        shadow_offset = 3
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 80)))
        painter.drawEllipse(
            center.x() - radius - shadow_offset + shadow_offset,
            center.y() - radius - shadow_offset + shadow_offset,
            (radius + shadow_offset) * 2,
            (radius + shadow_offset) * 2,
        )

        gradient = QRadialGradient(center, radius)
        gradient.setColorAt(0.0, QColor(60, 60, 60))
        gradient.setColorAt(0.7, QColor(45, 45, 45))
        gradient.setColorAt(1.0, QColor(30, 30, 30))

        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.drawEllipse(center, radius, radius)

        highlight_radius = radius - 5
        if highlight_radius > 0:
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor(100, 100, 100, 100), 1))
            painter.drawEllipse(center, highlight_radius, highlight_radius)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_sticky_buttons(self, enabled: bool) -> None:
        self.set_sticky_mode(enabled)

    def set_sticky_mode(self, enabled: bool) -> None:
        enabled = bool(enabled)
        for button in self._buttons.values():
            button.set_sticky(enabled)
        if not enabled:
            # Reset latched state when leaving sticky mode.
            self.reset_buttons()

    def set_button_checked(self, btn_id: GameButton, checked: bool) -> None:
        button = self._buttons.get(btn_id)
        if button is None:
            raise KeyError(f"Unknown button id: {btn_id}")
        button.setChecked(bool(checked))

    def get_pressed_buttons(self) -> Set[int]:
        return {int(btn_id) for btn_id in self._pressed}

    def reset_buttons(self) -> None:
        self.reset_all()

    def reset_all(self) -> None:
        if not self._pressed:
            for button in self._buttons.values():
                button.reset()
            return

        # copy to avoid mutation during iteration when setChecked emits
        for btn_id in list(self._pressed):
            self.set_button_checked(btn_id, False)
        self._pressed.clear()

    def _on_button_toggled(self, btn_id: GameButton, checked: bool) -> None:
        if checked:
            self._pressed.add(btn_id)
        else:
            self._pressed.discard(btn_id)
        index = int(btn_id)
        self.button_toggled.emit(index, checked)
        self.button_state_changed.emit(index, checked)
