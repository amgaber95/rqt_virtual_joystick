import math
from typing import Tuple

from python_qt_binding.QtCore import Qt, QRect, QSize, QTimer, pyqtSignal, QPoint
from python_qt_binding.QtGui import (
    QPainter,
    QPen,
    QBrush,
    QColor,
    QLinearGradient,
    QPainterPath,
    QRadialGradient,
    QFont,
)
from python_qt_binding.QtWidgets import QWidget

from .config_manager import (
    ConfigurationManager,
    RETURN_MODE_BOTH,
    RETURN_MODE_HORIZONTAL,
    RETURN_MODE_NONE,
    RETURN_MODE_VERTICAL,
)


class JoystickWidget(QWidget):
    """Visual joystick widget with mouse and keyboard interaction."""

    position_changed = pyqtSignal(float, float)

    def __init__(self, config_manager: ConfigurationManager, parent=None):
        super().__init__(parent)

        self._config_manager = config_manager
        self._position = (0.0, 0.0)
        self._raw_position = (0.0, 0.0)
        self._is_in_dead_zone = False
        self._is_in_x_dead_zone = False
        self._is_in_y_dead_zone = False
        self._handle_radius = 12
        self._pressed = False

        self.setMinimumSize(200, 200)
        self.setFocusPolicy(Qt.StrongFocus)

        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._emit_position_if_needed)

        self._config_manager.rate_changed.connect(self._on_rate_changed)
        self._config_manager.dead_zone_changed.connect(self._on_dead_zone_changed)
        self._config_manager.expo_changed.connect(self._on_expo_changed)
        self._config_manager.return_mode_changed.connect(self._on_return_mode_changed)
        self._on_rate_changed(self._config_manager.get_publish_rate())

    def _on_rate_changed(self, rate_hz: float):
        if rate_hz <= 0:
            return

        interval_ms = max(1, int(1000.0 / rate_hz))
        if self._update_timer.isActive():
            self._update_timer.stop()
        self._update_timer.setInterval(interval_ms)
        
    def _on_dead_zone_changed(self) -> None:
        # Re-apply dead zone logic to the last raw position
        self._set_position(*self._raw_position, emit_signal=True)
        self.update()

    def _on_expo_changed(self) -> None:
        self._set_position(*self._raw_position, emit_signal=True)
        self.update()

    def _on_return_mode_changed(self, _mode: str) -> None:
        if not self._pressed:
            self._apply_return_to_center(emit_signal=True)

    def sizeHint(self) -> QSize:
        return QSize(250, 250)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        size = min(width, height)
        center_x = width // 2
        center_y = height // 2
        radius = size // 2 - 5

        self._draw_outer_circle(painter, center_x, center_y, radius)
        self._draw_dead_zone(painter, center_x, center_y, radius)
        self._draw_expo_visual(painter, center_x, center_y, radius)
        self._draw_axes(painter, center_x, center_y, radius)
        self._draw_polar_grid(painter, center_x, center_y, radius)
        self._draw_center_marker(painter, center_x, center_y)
        self._draw_handle(painter, center_x, center_y, radius)
        self._draw_handle_info(painter)

    def _draw_outer_circle(self, painter: QPainter, center_x: int, center_y: int, radius: int):
        painter.save()

        gradient = QRadialGradient(center_x, center_y, radius)
        gradient.setColorAt(0.0, QColor(60, 60, 60))
        gradient.setColorAt(0.7, QColor(45, 45, 45))
        gradient.setColorAt(1.0, QColor(30, 30, 30))

        shadow_offset = 3
        shadow_radius = radius + shadow_offset
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 80)))
        shadow_rect = QRect(
            center_x - shadow_radius + shadow_offset,
            center_y - shadow_radius + shadow_offset,
            shadow_radius * 2,
            shadow_radius * 2,
        )
        painter.drawEllipse(shadow_rect)

        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        outer_rect = QRect(center_x - radius, center_y - radius, radius * 2, radius * 2)
        painter.drawEllipse(outer_rect)

        highlight_radius = radius - 5
        painter.setPen(QPen(QColor(100, 100, 100, 100), 1))
        painter.setBrush(Qt.NoBrush)
        highlight_rect = QRect(
            center_x - highlight_radius,
            center_y - highlight_radius,
            highlight_radius * 2,
            highlight_radius * 2,
        )
        painter.drawEllipse(highlight_rect)

        painter.restore()

    def _draw_dead_zone(self, painter: QPainter, center_x: int, center_y: int, radius: int):
        painter.save()

        base_color = QColor(255, 100, 100)

        dead_zone = self._config_manager.get_dead_zone()
        if dead_zone > 0.0:
            dead_zone_radius = int(radius * dead_zone)

            gradient = QRadialGradient(center_x, center_y, max(1, dead_zone_radius))
            gradient.setColorAt(0.0, QColor(base_color.red(), base_color.green(), base_color.blue(), 60))
            gradient.setColorAt(1.0, QColor(base_color.red(), base_color.green(), base_color.blue(), 20))

            painter.setPen(QPen(QColor(base_color.red(), base_color.green(), base_color.blue(), 120), 2))
            painter.setBrush(QBrush(gradient))

            dead_zone_rect = QRect(
                center_x - dead_zone_radius,
                center_y - dead_zone_radius,
                dead_zone_radius * 2,
                dead_zone_radius * 2,
            )
            painter.drawEllipse(dead_zone_rect)

        self._draw_dead_zone_x(painter, center_x, center_y, radius, base_color)
        self._draw_dead_zone_y(painter, center_x, center_y, radius, base_color)

        painter.restore()

    def _draw_expo_visual(self, painter: QPainter, center_x: int, center_y: int, radius: int) -> None:
        expo_x = self._config_manager.get_expo_x()
        expo_y = self._config_manager.get_expo_y()
        if expo_x <= 0.0 and expo_y <= 0.0:
            return

        painter.save()

        clip_path = QPainterPath()
        clip_path.addEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
        painter.setClipPath(clip_path)

        curve_radius = int(radius * 0.85)
        samples = 101

        if expo_x > 0.0:
            expo_factor_x = expo_x / 100.0
            painter.setPen(QPen(QColor(255, 120, 120, 200), 2))
            x_path = QPainterPath()

            for i in range(samples):
                input_val = -1.0 + i * 0.02
                sign = 1.0 if input_val >= 0.0 else -1.0
                abs_val = abs(input_val)
                output_val = sign * (abs_val * (1.0 - expo_factor_x) + (abs_val ** 3) * expo_factor_x)

                px = center_x + int(input_val * curve_radius)
                py = center_y - int(output_val * curve_radius)

                if i == 0:
                    x_path.moveTo(px, py)
                else:
                    x_path.lineTo(px, py)

            painter.drawPath(x_path)

        if expo_y > 0.0:
            expo_factor_y = expo_y / 100.0
            painter.setPen(QPen(QColor(120, 120, 255, 200), 2))
            y_path = QPainterPath()

            for i in range(samples):
                input_val = -1.0 + i * 0.02
                sign = 1.0 if input_val >= 0.0 else -1.0
                abs_val = abs(input_val)
                output_val = sign * (abs_val * (1.0 - expo_factor_y) + (abs_val ** 3) * expo_factor_y)

                px = center_x + int(output_val * curve_radius)
                py = center_y - int(input_val * curve_radius)

                if i == 0:
                    y_path.moveTo(px, py)
                else:
                    y_path.lineTo(px, py)

            painter.drawPath(y_path)

        painter.setPen(QPen(QColor(150, 150, 150, 100), 1, Qt.DotLine))
        painter.drawLine(center_x - curve_radius, center_y, center_x + curve_radius, center_y)
        painter.drawLine(center_x, center_y - curve_radius, center_x, center_y + curve_radius)
        painter.setClipping(False)

        painter.restore()

    def _draw_dead_zone_x(
        self,
        painter: QPainter,
        center_x: int,
        center_y: int,
        radius: int,
        base_color: QColor,
    ) -> None:
        dead_zone_x = self._config_manager.get_dead_zone_x()
        if dead_zone_x <= 0.0:
            return

        x_dead_width = int(radius * dead_zone_x)
        x_gradient = QLinearGradient(
            center_x - x_dead_width,
            center_y,
            center_x + x_dead_width,
            center_y,
        )
        x_gradient.setColorAt(0.0, QColor(base_color.red(), base_color.green(), base_color.blue(), 20))
        x_gradient.setColorAt(0.5, QColor(base_color.red(), base_color.green(), base_color.blue(), 50))
        x_gradient.setColorAt(1.0, QColor(base_color.red(), base_color.green(), base_color.blue(), 20))

        painter.setPen(QPen(QColor(base_color.red(), base_color.green(), base_color.blue(), 100), 1))
        painter.setBrush(QBrush(x_gradient))

        x_dead_rect = QRect(
            center_x - x_dead_width,
            center_y - radius + 5,
            x_dead_width * 2,
            2 * radius - 10,
        )
        painter.drawRect(x_dead_rect)

    def _draw_dead_zone_y(
        self,
        painter: QPainter,
        center_x: int,
        center_y: int,
        radius: int,
        base_color: QColor,
    ) -> None:
        dead_zone_y = self._config_manager.get_dead_zone_y()
        if dead_zone_y <= 0.0:
            return

        y_dead_height = int(radius * dead_zone_y)
        y_gradient = QLinearGradient(
            center_x,
            center_y - y_dead_height,
            center_x,
            center_y + y_dead_height,
        )
        y_gradient.setColorAt(0.0, QColor(base_color.red(), base_color.green(), base_color.blue(), 20))
        y_gradient.setColorAt(0.5, QColor(base_color.red(), base_color.green(), base_color.blue(), 40))
        y_gradient.setColorAt(1.0, QColor(base_color.red(), base_color.green(), base_color.blue(), 20))

        painter.setPen(QPen(QColor(base_color.red(), base_color.green(), base_color.blue(), 100), 1))
        painter.setBrush(QBrush(y_gradient))

        y_dead_rect = QRect(
            center_x - radius + 5,
            center_y - y_dead_height,
            2 * radius - 10,
            y_dead_height * 2,
        )
        painter.drawRect(y_dead_rect)

    def _draw_axes(self, painter: QPainter, center_x: int, center_y: int, radius: int):
        painter.save()

        axis_pen = QPen(QColor(180, 180, 180, 220), 1, Qt.DotLine)
        painter.setPen(axis_pen)
        painter.drawLine(center_x - radius + 10, center_y, center_x + radius - 10, center_y)
        painter.drawLine(center_x, center_y - radius + 10, center_x, center_y + radius - 10)

        tick_pen = QPen(QColor(100, 100, 100, 80), 1)
        painter.setPen(tick_pen)

        tick_positions = [0.25, 0.5, 0.75]
        tick_size = 3
        for pos in tick_positions:
            tick_x = int(radius * pos)
            painter.drawLine(center_x + tick_x, center_y - tick_size, center_x + tick_x, center_y + tick_size)
            painter.drawLine(center_x - tick_x, center_y - tick_size, center_x - tick_x, center_y + tick_size)

            tick_y = int(radius * pos)
            painter.drawLine(center_x - tick_size, center_y + tick_y, center_x + tick_size, center_y + tick_y)
            painter.drawLine(center_x - tick_size, center_y - tick_y, center_x + tick_size, center_y - tick_y)

        painter.restore()

    def _draw_polar_grid(self, painter: QPainter, center_x: int, center_y: int, radius: int):
        painter.save()

        circle_radii = [0.25, 0.5, 0.75]
        circle_pen = QPen(QColor(120, 120, 120, 200), 1, Qt.DotLine)
        painter.setPen(circle_pen)
        painter.setBrush(Qt.NoBrush)

        for ratio in circle_radii:
            circle_radius = int(radius * ratio)
            circle_rect = QRect(
                center_x - circle_radius,
                center_y - circle_radius,
                circle_radius * 2,
                circle_radius * 2,
            )
            painter.drawEllipse(circle_rect)
            
        main_axis_pen = QPen(QColor(140, 140, 140, 180), 1, Qt.DotLine)
        painter.setPen(main_axis_pen)
        for angle in [0, 90, 180, 270]:
            angle_rad = math.radians(angle)
            start_radius = int(radius * 0.01)
            start_x = center_x + int(start_radius * math.cos(angle_rad))
            start_y = center_y - int(start_radius * math.sin(angle_rad))
            end_radius = int(radius * 0.95)
            end_x = center_x + int(end_radius * math.cos(angle_rad))
            end_y = center_y - int(end_radius * math.sin(angle_rad))
            painter.drawLine(start_x, start_y, end_x, end_y)

        diagonal_pen = QPen(QColor(120, 120, 120, 140), 1, Qt.SolidLine)
        painter.setPen(diagonal_pen)
        for angle in [45, 135, 225, 315]:
            angle_rad = math.radians(angle)
            start_radius = int(radius * 0.01)
            start_x = center_x + int(start_radius * math.cos(angle_rad))
            start_y = center_y - int(start_radius * math.sin(angle_rad))
            end_radius = int(radius * 0.9)
            end_x = center_x + int(end_radius * math.cos(angle_rad))
            end_y = center_y - int(end_radius * math.sin(angle_rad))
            painter.drawLine(start_x, start_y, end_x, end_y)

        marker_pen = QPen(QColor(140, 140, 140, 120), 2)
        marker_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(marker_pen)
        marker_radius = radius - 8
        marker_size = 3
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            angle_rad = math.radians(angle)
            marker_x = center_x + int(marker_radius * math.cos(angle_rad))
            marker_y = center_y - int(marker_radius * math.sin(angle_rad))
            painter.setBrush(QBrush(QColor(140, 140, 140, 120)))
            painter.drawEllipse(QPoint(marker_x, marker_y), marker_size, marker_size)

        painter.restore()

    def _draw_center_marker(self, painter: QPainter, center_x: int, center_y: int):
        painter.save()
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.drawEllipse(QPoint(center_x, center_y), 4, 4)

        pen = QPen(QColor(160, 160, 160), 2)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(center_x - 8, center_y, center_x - 5, center_y)
        painter.drawLine(center_x + 5, center_y, center_x + 8, center_y)
        painter.drawLine(center_x, center_y - 8, center_x, center_y - 5)
        painter.drawLine(center_x, center_y + 5, center_x, center_y + 8)
        painter.restore()

    def _draw_handle(self, painter: QPainter, center_x: int, center_y: int, radius: int):
        painter.save()

        x, y = self._position
        handle_x = center_x + int(x * (radius - self._handle_radius))
        handle_y = center_y - int(y * (radius - self._handle_radius))

        connection_pen = QPen(QColor(120, 120, 120, 150), 2, Qt.DashLine)
        painter.setPen(connection_pen)
        painter.drawLine(center_x, center_y, handle_x, handle_y)

        shadow_offset = 2
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 60)))
        painter.drawEllipse(
            QPoint(handle_x + shadow_offset, handle_y + shadow_offset),
            self._handle_radius + 1,
            self._handle_radius + 1,
        )

        if self._is_in_dead_zone:
            base_color = QColor(255, 80, 80)
            glow_color = QColor(255, 120, 120, 100)
        elif self._is_in_x_dead_zone or self._is_in_y_dead_zone:
            base_color = QColor(255, 140, 60)
            glow_color = QColor(255, 180, 100, 100)
        else:
            base_color = QColor(80, 160, 255)
            glow_color = QColor(120, 180, 255, 100)
        glow_radius = self._handle_radius + 4
        glow_gradient = QRadialGradient(handle_x, handle_y, glow_radius)
        glow_gradient.setColorAt(0.0, glow_color)
        glow_gradient.setColorAt(1.0, QColor(glow_color.red(), glow_color.green(), glow_color.blue(), 0))
        painter.setBrush(QBrush(glow_gradient))
        painter.drawEllipse(QPoint(handle_x, handle_y), glow_radius, glow_radius)

        handle_gradient = QRadialGradient(handle_x - 3, handle_y - 3, self._handle_radius)
        light_color = base_color.lighter(150)
        handle_gradient.setColorAt(0.0, light_color)
        handle_gradient.setColorAt(0.5, base_color)
        handle_gradient.setColorAt(1.0, base_color.darker(120))
        painter.setBrush(QBrush(handle_gradient))
        painter.setPen(QPen(base_color.darker(150), 1))
        painter.drawEllipse(QPoint(handle_x, handle_y), self._handle_radius, self._handle_radius)

        highlight_radius = self._handle_radius - 3
        if highlight_radius > 0:
            painter.setBrush(QBrush(QColor(255, 255, 255, 80)))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPoint(handle_x - 2, handle_y - 2), highlight_radius, highlight_radius)

        painter.restore()

    def _draw_handle_info(self, painter: QPainter) -> None:
        painter.save()

        x, y = self._position
        width = self.width()
        height = self.height()
        size = min(width, height)
        center_x = width // 2
        center_y = height // 2
        radius = size // 2 - 5

        handle_x = center_x + int(x * (radius - self._handle_radius))
        handle_y = center_y - int(y * (radius - self._handle_radius))

        font = painter.font()
        font.setPointSize(max(font.pointSize() - 2, 9))
        painter.setFont(font)

        coords_text = f"({x:+.2f}, {y:+.2f})"
        font_metrics = painter.fontMetrics()
        padding = 6
        text_width = font_metrics.horizontalAdvance(coords_text) + padding * 2
        text_height = font_metrics.height() + padding * 2
        offset = self._handle_radius + 8

        if x <= 0.0:
            rect_left = handle_x + offset
            alignment = Qt.AlignLeft | Qt.AlignVCenter
        else:
            rect_left = handle_x - offset - text_width
            alignment = Qt.AlignRight | Qt.AlignVCenter

        rect_top = handle_y - (text_height // 2)
        text_rect = QRect(rect_left, rect_top, text_width, text_height)

        min_margin = 5
        if text_rect.left() < min_margin:
            text_rect.moveLeft(min_margin)
        if text_rect.right() > width - min_margin:
            text_rect.moveRight(width - min_margin)
        if text_rect.top() < min_margin:
            text_rect.moveTop(min_margin)
        if text_rect.bottom() > height - min_margin:
            text_rect.moveBottom(height - min_margin)

        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(text_rect, alignment, coords_text)
        painter.restore()

    def mousePressEvent(self, event):  # noqa: D401 - Qt override
        if event.button() == Qt.LeftButton:
            self._pressed = True
            self._update_position_from_mouse(event.x(), event.y())
            self.setFocus()
            self._start_update_timer()

    def mouseMoveEvent(self, event):  # noqa: D401 - Qt override
        if self._pressed:
            self._update_position_from_mouse(event.x(), event.y())

    def mouseReleaseEvent(self, event):  # noqa: D401 - Qt override
        if event.button() == Qt.LeftButton:
            self._pressed = False
            self._stop_update_timer()
            self._apply_return_to_center(emit_signal=True)

    def keyPressEvent(self, event):  # noqa: D401 - Qt override
        x, y = self._position
        step = 0.1
        changed = False

        if event.key() == Qt.Key_Left:
            x = max(-1.0, x - step)
            changed = True
        elif event.key() == Qt.Key_Right:
            x = min(1.0, x + step)
            changed = True
        elif event.key() == Qt.Key_Up:
            y = min(1.0, y + step)
            changed = True
        elif event.key() == Qt.Key_Down:
            y = max(-1.0, y - step)
            changed = True
        elif event.key() == Qt.Key_Space:
            x, y = 0.0, 0.0
            changed = True
        else:
            super().keyPressEvent(event)
            return

        if changed:
            self._set_position(x, y, emit_signal=True)

    def _update_position_from_mouse(self, mouse_x: int, mouse_y: int):
        center_x = self.width() // 2
        center_y = self.height() // 2
        radius = min(self.width(), self.height()) // 2 - 5
        max_distance = max(1, radius - self._handle_radius)

        dx = mouse_x - center_x
        dy = center_y - mouse_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > max_distance:
            dx = dx * max_distance / distance
            dy = dy * max_distance / distance

        raw_x = dx / max_distance
        raw_y = dy / max_distance
        
        self._set_position(raw_x, raw_y, emit_signal=True)
        
    def _set_position(self, x: float, y: float, emit_signal: bool = False):
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))

        self._raw_position = (x, y)
        processed_x, processed_y = self._apply_dead_zones(x, y)
        final_x, final_y = self._apply_exponential_response(processed_x, processed_y)

        should_emit = emit_signal and (not self._is_in_dead_zone or (final_x, final_y) == (0.0, 0.0))

        if (final_x, final_y) != self._position:
            self._position = (final_x, final_y)
            self.update()
            if should_emit:
                self.position_changed.emit(final_x, final_y)
        elif should_emit:
            self.position_changed.emit(final_x, final_y)

    def _apply_dead_zones(self, x: float, y: float) -> Tuple[float, float]:
        dead_zone, dead_zone_x, dead_zone_y = self._config_manager.get_dead_zones()

        distance = math.sqrt(x * x + y * y)
        in_circular_dead = distance < dead_zone

        if in_circular_dead:
            self._is_in_dead_zone = True
            self._is_in_x_dead_zone = True
            self._is_in_y_dead_zone = True
            return (0.0, 0.0)

        self._is_in_x_dead_zone = abs(x) < dead_zone_x
        self._is_in_y_dead_zone = abs(y) < dead_zone_y
        self._is_in_dead_zone = self._is_in_x_dead_zone and self._is_in_y_dead_zone

        result_x = 0.0 if self._is_in_x_dead_zone else x
        result_y = 0.0 if self._is_in_y_dead_zone else y
        return (result_x, result_y)

    def _apply_exponential_response(self, x: float, y: float) -> Tuple[float, float]:
        expo_x = self._config_manager.get_expo_x()
        expo_y = self._config_manager.get_expo_y()

        adjusted_x = x
        adjusted_y = y

        if expo_x > 0.0 and x != 0.0:
            expo_factor_x = expo_x / 100.0
            sign_x = 1.0 if x > 0.0 else -1.0
            abs_x = abs(x)
            adjusted_x = sign_x * (abs_x * (1.0 - expo_factor_x) + (abs_x ** 3) * expo_factor_x)

        if expo_y > 0.0 and y != 0.0:
            expo_factor_y = expo_y / 100.0
            sign_y = 1.0 if y > 0.0 else -1.0
            abs_y = abs(y)
            adjusted_y = sign_y * (abs_y * (1.0 - expo_factor_y) + (abs_y ** 3) * expo_factor_y)

        return (adjusted_x, adjusted_y)

    def _start_update_timer(self):
        if not self._update_timer.isActive():
            self._update_timer.start()

    def _stop_update_timer(self):
        if self._update_timer.isActive():
            self._update_timer.stop()

    def _emit_position_if_needed(self):
        if self._pressed and not self._is_in_dead_zone:
            self.position_changed.emit(*self._position)

    def _apply_return_to_center(self, emit_signal: bool = False) -> None:
        mode = self._config_manager.get_return_mode()

        x, y = self._position
        if mode == RETURN_MODE_BOTH:
            x, y = 0.0, 0.0
        elif mode == RETURN_MODE_HORIZONTAL:
            x = 0.0
        elif mode == RETURN_MODE_VERTICAL:
            y = 0.0
        elif mode == RETURN_MODE_NONE:
            return

        self._set_position(x, y, emit_signal=emit_signal)

    def get_position(self) -> Tuple[float, float]:
        return self._position

    def set_position(self, x: float, y: float):
        self._set_position(x, y, emit_signal=True)

    def reset_position(self):
        self._set_position(0.0, 0.0, emit_signal=True)
