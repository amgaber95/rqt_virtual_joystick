"""Joystick widget providing the core teleoperation pad."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from python_qt_binding.QtCore import QPointF, Qt, QRectF, Signal
from python_qt_binding.QtGui import QColor, QLinearGradient, QPainter, QPen, QRadialGradient
from python_qt_binding.QtWidgets import QFrame, QSizePolicy, QWidget


__all__ = ['JoystickPosition', 'JoystickWidget']


@dataclass(frozen=True)
class JoystickPosition:
    """Immutable representation of the joystick handle position."""

    x: float
    y: float


@dataclass(frozen=True)
class _PadMetrics:
    """Geometric parameters used when rendering the joystick pad."""

    center_x: float
    center_y: float
    outer_radius: float
    inner_radius: float


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

        metrics = self._calculate_pad_metrics()
        self._draw_background(painter, metrics)
        self._draw_axes(painter, metrics)
        self._draw_handle(painter, metrics)

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

    def _calculate_pad_metrics(self) -> _PadMetrics:
        """Compute the geometry values used for drawing."""

        rect = self.rect()
        size = float(min(rect.width(), rect.height()))
        outer_radius = max(0.0, size * 0.5 * 0.92)
        inner_radius = outer_radius * 0.88
        center = QPointF(rect.center())
        return _PadMetrics(center_x=center.x(), center_y=center.y(), outer_radius=outer_radius, inner_radius=inner_radius)

    def _draw_background(self, painter: QPainter, metrics: _PadMetrics) -> None:
        """Paint the circular pad background with subtle gradients."""

        painter.save()
        painter.fillRect(self.rect(), QColor(18, 22, 28))

        center = QPointF(metrics.center_x, metrics.center_y)
        outer_rect = QRectF(
            center.x() - metrics.outer_radius,
            center.y() - metrics.outer_radius,
            metrics.outer_radius * 2,
            metrics.outer_radius * 2,
        )

        outer_gradient = QRadialGradient(center, metrics.outer_radius)
        outer_gradient.setColorAt(0.0, QColor(70, 120, 170, 220))
        outer_gradient.setColorAt(0.45, QColor(40, 60, 90, 180))
        outer_gradient.setColorAt(1.0, QColor(15, 20, 28, 230))

        painter.setPen(QPen(QColor(60, 110, 170), 3))
        painter.setBrush(outer_gradient)
        painter.drawEllipse(outer_rect)

        inner_rect = QRectF(
            center.x() - metrics.inner_radius,
            center.y() - metrics.inner_radius,
            metrics.inner_radius * 2,
            metrics.inner_radius * 2,
        )

        inner_gradient = QLinearGradient(inner_rect.topLeft(), inner_rect.bottomRight())
        inner_gradient.setColorAt(0.0, QColor(26, 32, 42))
        inner_gradient.setColorAt(0.5, QColor(18, 24, 32))
        inner_gradient.setColorAt(1.0, QColor(12, 14, 22))

        painter.setPen(QPen(QColor(35, 70, 120), 2))
        painter.setBrush(inner_gradient)
        painter.drawEllipse(inner_rect)
        painter.restore()

    def _draw_axes(self, painter: QPainter, metrics: _PadMetrics) -> None:
        """Render directional guides and tick marks on the pad."""

        painter.save()
        center_x, center_y = metrics.center_x, metrics.center_y
        radius = metrics.inner_radius

        axis_pen = QPen(QColor(90, 120, 160), 1)
        axis_pen.setStyle(Qt.DotLine)
        painter.setPen(axis_pen)
        painter.drawLine(QPointF(center_x - radius, center_y), QPointF(center_x + radius, center_y))
        painter.drawLine(QPointF(center_x, center_y - radius), QPointF(center_x, center_y + radius))

        tick_pen = QPen(QColor(120, 160, 200), 1)
        painter.setPen(tick_pen)
        for fraction in (-0.5, 0.5):
            offset = radius * fraction
            painter.drawLine(QPointF(center_x + offset, center_y - 6), QPointF(center_x + offset, center_y + 6))
            painter.drawLine(QPointF(center_x - 6, center_y + offset), QPointF(center_x + 6, center_y + offset))

        painter.setPen(QPen(QColor(50, 90, 140), 2))
        painter.drawEllipse(QRectF(center_x - 4, center_y - 4, 8, 8))
        painter.restore()

    def _draw_handle(self, painter: QPainter, metrics: _PadMetrics) -> None:
        """Draw the joystick handle, indicator line, and coordinate bubble."""

        painter.save()
        max_radius = metrics.inner_radius * 0.86
        handle_radius = metrics.inner_radius * 0.16
        handle_center = QPointF(
            metrics.center_x + self._current_position.x() * max_radius,
            metrics.center_y - self._current_position.y() * max_radius,
        )

        painter.setPen(QPen(QColor(120, 160, 220, 180), 1, Qt.DashLine))
        painter.drawLine(
            QPointF(metrics.center_x, metrics.center_y),
            QPointF(handle_center.x(), handle_center.y()),
        )

        handle_gradient = QRadialGradient(handle_center, handle_radius)
        handle_gradient.setColorAt(0.0, QColor(255, 255, 255, 220))
        handle_gradient.setColorAt(0.4, QColor(120, 170, 230, 220))
        handle_gradient.setColorAt(1.0, QColor(40, 90, 150, 230))

        painter.setPen(QPen(QColor(180, 210, 255), 2))
        painter.setBrush(handle_gradient)
        painter.drawEllipse(QRectF(
            handle_center.x() - handle_radius,
            handle_center.y() - handle_radius,
            handle_radius * 2,
            handle_radius * 2,
        ))

        metrics_font = painter.fontMetrics()
        text = f"{self._current_position.x():+0.2f}, {self._current_position.y():+0.2f}"
        padding_x = 8
        padding_y = 4
        text_width = metrics_font.horizontalAdvance(text) + padding_x * 2
        text_height = metrics_font.height() + padding_y * 2

        bubble_x = handle_center.x() + handle_radius + 10
        bubble_y = handle_center.y() - text_height / 2

        max_x = metrics.center_x + metrics.outer_radius - text_width - 6
        min_x = metrics.center_x - metrics.outer_radius + 6
        bubble_x = min(max(bubble_x, min_x), max_x)

        max_y = metrics.center_y + metrics.outer_radius - text_height - 6
        min_y = metrics.center_y - metrics.outer_radius + 6
        bubble_y = min(max(bubble_y, min_y), max_y)

        bubble_rect = QRectF(bubble_x, bubble_y, text_width, text_height)

        painter.setPen(QPen(QColor(70, 120, 180), 1))
        painter.setBrush(QColor(10, 16, 26, 220))
        painter.drawRoundedRect(bubble_rect, 6, 6)

        painter.setPen(QColor(200, 220, 240))
        painter.drawText(
            bubble_rect.adjusted(padding_x, padding_y, -padding_x, -padding_y),
            Qt.AlignCenter,
            text,
        )

        painter.restore()
