from __future__ import annotations
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Tuple, Optional

# Qt
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


class ReturnMode(Enum):
    """How the stick returns to center on release."""
    BOTH = auto()
    HORIZONTAL = auto()
    VERTICAL = auto()
    NONE = auto()


@dataclass
class JoystickConfig:
    """configuration consumed by the widget/state."""
    publish_rate_hz: float = 50.0
    dead_zone: float = 0.0           # circular [0..1]
    dead_zone_x: float = 0.0         # axis X [0..1]
    dead_zone_y: float = 0.0         # axis Y [0..1]
    expo_x: float = 0.0              # percent [0..100]
    expo_y: float = 0.0              # percent [0..100]
    return_mode: ReturnMode = ReturnMode.BOTH
    renormalize_after_axis_deadzone: bool = False


@dataclass(frozen=True)
class JoystickSnapshot:
    """Immutable view of current stick state for painting/UI."""
    raw_x: float
    raw_y: float
    x: float            # processed
    y: float            # processed
    in_dead: bool
    in_x_dead: bool
    in_y_dead: bool


def clamp_unit(v: float) -> float:
    """Clamp to [-1, 1]."""
    return max(-1.0, min(1.0, v))


def apply_dead_zones(x: float, y: float, dz: float, dzx: float, dzy: float) -> Tuple[float, float, bool, bool, bool]:
    """
    Return (x', y', in_both_dead, in_x_dead, in_y_dead).
    Applies circular then per-axis dead zones (no renormalization).
    """
    distance = math.sqrt(x * x + y * y)
    in_circular_dead = distance < dz

    if in_circular_dead:
        return (0.0, 0.0, True, True, True)

    in_x_dead = abs(x) < dzx
    in_y_dead = abs(y) < dzy
    in_both_dead = in_x_dead and in_y_dead

    result_x = 0.0 if in_x_dead else x
    result_y = 0.0 if in_y_dead else y
    
    return (result_x, result_y, in_both_dead, in_x_dead, in_y_dead)


def renorm_after_axis_dz(v: float, dz_axis: float) -> float:
    """Optional: preserve full travel after axis dead zone."""
    if abs(v) <= dz_axis:
        return 0.0
    
    sign = 1.0 if v >= 0.0 else -1.0
    abs_v = abs(v)
    
    # Renormalize from [dz_axis, 1.0] to [0.0, 1.0]
    renormed = (abs_v - dz_axis) / (1.0 - dz_axis)
    return sign * renormed


def apply_expo(v: float, expo_pct: float) -> float:
    """Cubic mix between linear and v^3 per expo percentage."""
    if expo_pct <= 0.0 or v == 0.0:
        return v
    
    expo_factor = expo_pct / 100.0
    sign = 1.0 if v >= 0.0 else -1.0
    abs_v = abs(v)
    
    # Mix between linear and cubic
    result = abs_v * (1.0 - expo_factor) + (abs_v ** 3) * expo_factor
    return sign * result


class JoystickState:
    """
    Holds raw/processed values and flags.
    Responsibilities:
      - Ingest raw inputs [-1,1]
      - Recompute processed outputs using config (dead zones, renorm, expo)
      - Provide immutable snapshots for painting
    """

    def __init__(self, cfg: JoystickConfig) -> None:
        self._config = cfg
        self._raw_x = 0.0
        self._raw_y = 0.0
        self._x = 0.0
        self._y = 0.0
        self._in_dead = False
        self._in_x_dead = False
        self._in_y_dead = False

    def set_config(self, cfg: JoystickConfig) -> None:
        self._config = cfg
        self.recompute()

    def get_config(self) -> JoystickConfig:
        return self._config

    def ingest_raw(self, rx: float, ry: float) -> None:
        self._raw_x = clamp_unit(rx)
        self._raw_y = clamp_unit(ry)
        self.recompute()

    def recompute(self) -> None:
        # Apply dead zones
        x, y, in_dead, in_x_dead, in_y_dead = apply_dead_zones(
            self._raw_x, self._raw_y,
            self._config.dead_zone,
            self._config.dead_zone_x,
            self._config.dead_zone_y
        )
        
        # Optional renormalization after axis dead zones
        if self._config.renormalize_after_axis_deadzone:
            if not in_x_dead:
                x = renorm_after_axis_dz(x, self._config.dead_zone_x)
            if not in_y_dead:
                y = renorm_after_axis_dz(y, self._config.dead_zone_y)
        
        # Apply exponential response
        x = apply_expo(x, self._config.expo_x)
        y = apply_expo(y, self._config.expo_y)
        
        # Update state
        self._x = x
        self._y = y
        self._in_dead = in_dead
        self._in_x_dead = in_x_dead
        self._in_y_dead = in_y_dead

    def apply_return(self, mode: ReturnMode) -> None:
        if mode == ReturnMode.BOTH:
            self.ingest_raw(0.0, 0.0)
        elif mode == ReturnMode.HORIZONTAL:
            self.ingest_raw(0.0, self._raw_y)
        elif mode == ReturnMode.VERTICAL:
            self.ingest_raw(self._raw_x, 0.0)
        # ReturnMode.NONE does nothing

    def raw(self) -> Tuple[float, float]:
        return (self._raw_x, self._raw_y)

    def processed(self) -> Tuple[float, float]:
        return (self._x, self._y)

    def flags(self) -> Tuple[bool, bool, bool]:
        """Return (in_dead, in_x_dead, in_y_dead)."""
        return (self._in_dead, self._in_x_dead, self._in_y_dead)

    def snapshot(self) -> JoystickSnapshot:
        return JoystickSnapshot(
            raw_x=self._raw_x,
            raw_y=self._raw_y,
            x=self._x,
            y=self._y,
            in_dead=self._in_dead,
            in_x_dead=self._in_x_dead,
            in_y_dead=self._in_y_dead
        )


# =========================== Widget (View/Controller) =========================

class JoystickWidget(QWidget):
    """
    Joystick widget with its own config & state.
    - Emits processed position at a configurable rate while pressed.
    - Exposes simple config setters/getters (no external manager dependency).
    """

    # Signals
    position_changed = pyqtSignal(float, float)        # processed (after DZ/expo)
    position_raw_changed = pyqtSignal(float, float)    # raw (before DZ/expo)

    def __init__(self, config: Optional[JoystickConfig] = None, parent=None) -> None:
        super().__init__(parent)
        
        self._config = config or JoystickConfig()
        self._state = JoystickState(self._config)
        self._pressed = False
        self._handle_radius = 12
        
        self.setFixedSize(200, 200)
        self.setFocusPolicy(Qt.StrongFocus)
        
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._emit_position_if_needed)
        self._apply_rate()

    # ---- public config api (simple) ----
    def get_config(self) -> JoystickConfig:
        return self._config

    def set_config(self, cfg: JoystickConfig) -> None:
        old_processed = self._state.processed()
        self._config = cfg
        self._state.set_config(cfg)
        self._apply_rate()
        self._emit_if_changed(old_processed, self._state.processed())
        self.update()

    def set_publish_rate(self, hz: float) -> None:
        self._config.publish_rate_hz = max(0.1, hz)
        self._apply_rate()

    def set_dead_zone(self, v: float) -> None:
        old_processed = self._state.processed()
        self._config.dead_zone = max(0.0, min(1.0, v))
        self._reprocess_last_input()
        self._emit_if_changed(old_processed, self._state.processed())

    def set_dead_zone_x(self, v: float) -> None:
        old_processed = self._state.processed()
        self._config.dead_zone_x = max(0.0, min(1.0, v))
        self._reprocess_last_input()
        self._emit_if_changed(old_processed, self._state.processed())

    def set_dead_zone_y(self, v: float) -> None:
        old_processed = self._state.processed()
        self._config.dead_zone_y = max(0.0, min(1.0, v))
        self._reprocess_last_input()
        self._emit_if_changed(old_processed, self._state.processed())

    def set_expo_x(self, pct: float) -> None:
        old_processed = self._state.processed()
        self._config.expo_x = max(0.0, min(100.0, pct))
        self._reprocess_last_input()
        self._emit_if_changed(old_processed, self._state.processed())

    def set_expo_y(self, pct: float) -> None:
        old_processed = self._state.processed()
        self._config.expo_y = max(0.0, min(100.0, pct))
        self._reprocess_last_input()
        self._emit_if_changed(old_processed, self._state.processed())

    def set_return_mode(self, mode: ReturnMode) -> None:
        self._config.return_mode = mode

    def set_renormalize_after_axis_deadzone(self, enabled: bool) -> None:
        old_processed = self._state.processed()
        self._config.renormalize_after_axis_deadzone = enabled
        self._reprocess_last_input()
        self._emit_if_changed(old_processed, self._state.processed())

    # ---- public position api ----
    def get_position(self) -> Tuple[float, float]:
        return self._state.processed()

    def get_raw_position(self) -> Tuple[float, float]:
        return self._state.raw()

    def set_position(self, x: float, y: float) -> None:
        old_processed = self._state.processed()
        old_raw = self._state.raw()
        self._state.ingest_raw(x, y)
        new_processed = self._state.processed()
        new_raw = self._state.raw()
        
        if new_raw != old_raw:
            self.position_raw_changed.emit(*new_raw)
        self._emit_if_changed(old_processed, new_processed)
        self.update()

    def reset_position(self) -> None:
        self.set_position(0.0, 0.0)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()
        size = min(width, height)
        center_x = width // 2
        center_y = height // 2
        radius = size // 2 - 5

        snap = self._state.snapshot()

        self._draw_outer_circle(painter, center_x, center_y, radius)
        self._draw_dead_zone(painter, center_x, center_y, radius)
        self._draw_expo_visual(painter, center_x, center_y, radius)
        self._draw_axes(painter, center_x, center_y, radius)
        self._draw_polar_grid(painter, center_x, center_y, radius)
        self._draw_center_marker(painter, center_x, center_y)
        self._draw_handle(painter, center_x, center_y, radius, snap)
        self._draw_handle_info(painter, center_x, center_y, radius, snap)

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

        dead_zone = self._config.dead_zone
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
        expo_x = self._config.expo_x
        expo_y = self._config.expo_y
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
        dead_zone_x = self._config.dead_zone_x
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
        dead_zone_y = self._config.dead_zone_y
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

    def _draw_handle(self, painter: QPainter, center_x: int, center_y: int, radius: int, snap: JoystickSnapshot):
        painter.save()

        handle_x = center_x + int(snap.x * (radius - self._handle_radius))
        handle_y = center_y - int(snap.y * (radius - self._handle_radius))

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

        if snap.in_dead:
            base_color = QColor(255, 80, 80)
            glow_color = QColor(255, 120, 120, 100)
        elif snap.in_x_dead or snap.in_y_dead:
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

    def _draw_handle_info(self, painter: QPainter, center_x: int, center_y: int, radius: int, snap: JoystickSnapshot) -> None:
        painter.save()

        handle_x = center_x + int(snap.x * (radius - self._handle_radius))
        handle_y = center_y - int(snap.y * (radius - self._handle_radius))

        font = painter.font()
        font.setPointSize(max(font.pointSize() - 2, 9))
        painter.setFont(font)

        coords_text = f"({snap.x:+.2f}, {snap.y:+.2f})"
        font_metrics = painter.fontMetrics()
        padding = 6
        text_width = font_metrics.horizontalAdvance(coords_text) + padding * 2
        text_height = font_metrics.height() + padding * 2
        offset = self._handle_radius + 8

        if snap.x <= 0.0:
            rect_left = handle_x + offset
            alignment = Qt.AlignLeft | Qt.AlignVCenter
        else:
            rect_left = handle_x - offset - text_width
            alignment = Qt.AlignRight | Qt.AlignVCenter

        rect_top = handle_y - (text_height // 2)
        text_rect = QRect(rect_left, rect_top, text_width, text_height)

        min_margin = 5
        widget_width = self.width()
        widget_height = self.height()
        if text_rect.left() < min_margin:
            text_rect.moveLeft(min_margin)
        if text_rect.right() > widget_width - min_margin:
            text_rect.moveRight(widget_width - min_margin)
        if text_rect.top() < min_margin:
            text_rect.moveTop(min_margin)
        if text_rect.bottom() > widget_height - min_margin:
            text_rect.moveBottom(widget_height - min_margin)

        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.drawText(text_rect, alignment, coords_text)
        painter.restore()

    def mousePressEvent(self, e) -> None:
        if e.button() == Qt.LeftButton:
            self._pressed = True
            self._update_from_mouse_xy(e.x(), e.y())
            self.setFocus()
            if self._timer.interval() > 0:
                self._timer.start()

    def mouseMoveEvent(self, e) -> None:
        if self._pressed:
            self._update_from_mouse_xy(e.x(), e.y())

    def mouseReleaseEvent(self, e) -> None:
        if e.button() == Qt.LeftButton:
            self._pressed = False
            self._timer.stop()
            old_processed = self._state.processed()
            self._state.apply_return(self._config.return_mode)
            self._emit_if_changed(old_processed, self._state.processed())
            self.update()

    def keyPressEvent(self, e) -> None:
        x, y = self._state.processed()
        step = 0.05
        changed = False

        if e.key() == Qt.Key_Left:
            x = max(-1.0, x - step)
            changed = True
        elif e.key() == Qt.Key_Right:
            x = min(1.0, x + step)
            changed = True
        elif e.key() == Qt.Key_Up:
            y = min(1.0, y + step)
            changed = True
        elif e.key() == Qt.Key_Down:
            y = max(-1.0, y - step)
            changed = True
        elif e.key() == Qt.Key_Space:
            x, y = 0.0, 0.0
            changed = True
        else:
            super().keyPressEvent(e)
            return

        if changed:
            self.set_position(x, y)

    def reset_position(self):
        self.set_position(0.0, 0.0)

    # ---- internal helpers ----
    def _update_from_mouse_xy(self, mx: int, my: int) -> None:
        """Map widget coords → normalized raw [-1,1], update state, emit signals if changed."""
        center_x = self.width() // 2
        center_y = self.height() // 2
        radius = min(self.width(), self.height()) // 2 - 5
        max_distance = max(1, radius - self._handle_radius)

        dx = mx - center_x
        dy = center_y - my
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > max_distance:
            dx = dx * max_distance / distance
            dy = dy * max_distance / distance

        raw_x = dx / max_distance
        raw_y = dy / max_distance

        old_processed = self._state.processed()
        old_raw = self._state.raw()
        self._state.ingest_raw(raw_x, raw_y)
        new_processed = self._state.processed()
        new_raw = self._state.raw()
        
        if new_raw != old_raw:
            self.position_raw_changed.emit(*new_raw)
        self._emit_if_changed(old_processed, new_processed)
        self.update()

    def _apply_rate(self) -> None:
        if self._config.publish_rate_hz <= 0:
            self._timer.setInterval(0)
        else:
            interval_ms = max(1, int(1000.0 / self._config.publish_rate_hz))
            self._timer.setInterval(interval_ms)

    def _emit_position_if_needed(self) -> None:
        if self._pressed and not self._state.flags()[0]:  # not in_dead
            self.position_changed.emit(*self._state.processed())

    def _reprocess_last_input(self) -> None:
        """Recompute from last raw using current config, repaint & emit if changed."""
        self._state.recompute()
        self.update()

    def _emit_if_changed(self, before_xy: Tuple[float, float], after_xy: Tuple[float, float]) -> None:
        if before_xy != after_xy:
            self.position_changed.emit(*after_xy)

    # ---- JoystickConfigAPI protocol implementation ----
    def get_dead_zone(self) -> float:
        return self._config.dead_zone

    def get_dead_zone_x(self) -> float:
        return self._config.dead_zone_x

    def get_dead_zone_y(self) -> float:
        return self._config.dead_zone_y

    def get_expo_x(self) -> float:
        return self._config.expo_x

    def get_expo_y(self) -> float:
        return self._config.expo_y

    def get_return_mode(self) -> 'ReturnMode':
        return self._config.return_mode

    def get_sticky_buttons(self) -> bool:
        # This widget doesn't handle buttons, but return False for protocol compliance
        return False

    def set_sticky_buttons(self, on: bool) -> None:
        # This widget doesn't handle buttons, but accept the call for protocol compliance
        pass
