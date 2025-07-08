"""Joystick widget providing the core teleoperation pad."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from python_qt_binding.QtCore import QPointF, Qt, Signal
from python_qt_binding.QtGui import QColor, QPainter, QPen
from python_qt_binding.QtWidgets import QFrame, QSizePolicy, QWidget


__all__ = ['JoystickPosition', 'JoystickWidget']


@dataclass(frozen=True)
class JoystickPosition:
    """Immutable representation of the joystick handle position."""

    x: float
    y: float


class JoystickWidget(QFrame):
    """Interactive joystick pad with normalised axis output."""

    position_changed = Signal(float, float)
    interaction_finished = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._current_position = QPointF(0.0, 0.0)
        self._is_dragging = False

        self.setObjectName('JoystickWidget')
        self.setMinimumSize(160, 160)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFrameShape(QFrame.StyledPanel)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    @property
    def current_position(self) -> JoystickPosition:
        """Return the current normalised joystick position."""

        return JoystickPosition(self._current_position.x(), self._current_position.y())

    def reset(self) -> None:
        """Return the joystick to its neutral position."""

        if self._current_position == QPointF(0.0, 0.0):
            return

        self._current_position = QPointF(0.0, 0.0)
        self.update()
        self.position_changed.emit(0.0, 0.0)
        self.interaction_finished.emit()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        """Render the joystick background and current handle."""

        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        rect = self.rect()
        radius = min(rect.width(), rect.height()) * 0.5 * 0.9
        center = QPointF(rect.center())

        painter.setPen(QPen(QColor(70, 70, 70), 2))
        painter.setBrush(QColor(230, 230, 230))
        painter.drawEllipse(center, radius, radius)

        handle_radius = radius * 0.2
        handle_center = QPointF(
            center.x() + self._current_position.x() * radius,
            center.y() - self._current_position.y() * radius,
        )
        painter.setPen(QPen(QColor(30, 120, 190), 2))
        painter.setBrush(QColor(50, 150, 220, 200))
        painter.drawEllipse(handle_center, handle_radius, handle_radius)

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self._is_dragging = True
            self._update_position(self._event_position(event))
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        if self._is_dragging:
            self._update_position(self._event_position(event))
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton and self._is_dragging:
            self._is_dragging = False
            self.reset()
        super().mouseReleaseEvent(event)

    def leaveEvent(self, event) -> None:  # type: ignore[override]
        if self._is_dragging:
            self._is_dragging = False
            self.reset()
        super().leaveEvent(event)

    @staticmethod
    def _event_position(event) -> QPointF:
        if hasattr(event, 'position'):
            return QPointF(event.position())
        if hasattr(event, 'localPos'):
            return QPointF(event.localPos())
        return QPointF(event.pos())

    def _update_position(self, position: QPointF) -> None:
        rect = self.rect()
        center = QPointF(rect.center())
        radius = float(min(rect.width(), rect.height())) * 0.5
        if radius <= 0.0:
            return

        delta_x = (position.x() - center.x()) / radius
        delta_y = (center.y() - position.y()) / radius

        magnitude = (delta_x ** 2 + delta_y ** 2) ** 0.5
        if magnitude > 1.0:
            delta_x /= magnitude
            delta_y /= magnitude

        next_position = QPointF(delta_x, delta_y)
        if next_position == self._current_position:
            return

        self._current_position = next_position
        self.update()
        self.position_changed.emit(delta_x, delta_y)
