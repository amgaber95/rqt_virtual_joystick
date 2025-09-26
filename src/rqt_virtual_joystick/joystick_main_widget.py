"""Main widget for the RQt Virtual Joystick plugin."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from python_qt_binding.QtCore import QObject, Qt, QEvent
from python_qt_binding.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node

from .control_panels import JoyOutputPanel, JoystickConfigPanel, TwistOutputPanel
from .widgets.controller_buttons_widget import ControllerButtonsWidget
from .publishers.joy_publisher import JoyPublisherService
from .widgets.joystick_widget import JoystickWidget, ReturnMode
from .publishers.twist_publisher import TwistPublisherService


def _to_bool(value, default: bool) -> bool:
    """Coerce a QVariant/QSettings value to bool with fallback."""
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    return default


def _to_float(value, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):  # pragma: no cover - defensive
        return default


class _JoyOutputAPIAdapter:
    """Expose JoyPublisherService via the JoyOutputPanel protocol."""

    def __init__(self, service: JoyPublisherService) -> None:
        self._service = service

    def get_enabled(self) -> bool:
        return self._service.is_enabled()

    def set_enabled(self, on: bool) -> None:
        self._service.set_enabled(on)

    def get_topic(self) -> str:
        return self._service.get_topic()

    def set_topic(self, name: str) -> None:
        self._service.set_topic(name)

    def get_rate_hz(self) -> float:
        return self._service.get_rate_hz()

    def set_rate_hz(self, hz: float) -> None:
        self._service.set_rate_hz(hz)


class _TwistOutputAPIAdapter:
    """Expose TwistPublisherService via the TwistOutputPanel protocol."""

    def __init__(self, service: TwistPublisherService) -> None:
        self._service = service

    def get_enabled(self) -> bool:
        return self._service.is_enabled()

    def set_enabled(self, on: bool) -> None:
        self._service.set_enabled(on)

    def get_topic(self) -> str:
        return self._service.get_topic()

    def set_topic(self, name: str) -> None:
        self._service.set_topic(name)

    def get_rate_hz(self) -> float:
        return self._service.get_rate_hz()

    def set_rate_hz(self, hz: float) -> None:
        self._service.set_rate_hz(hz)

    def get_scales(self) -> tuple[float, float]:
        return self._service.get_scales()

    def set_scales(self, linear: float, angular: float) -> None:
        self._service.set_scales(linear, angular)

    def get_use_stamped(self) -> bool:
        return self._service.get_use_stamped()

    def set_use_stamped(self, on: bool) -> None:
        self._service.set_use_stamped(on)

    def get_holonomic(self) -> bool:
        return self._service.get_holonomic()

    def set_holonomic(self, on: bool) -> None:
        self._service.set_holonomic(on)


@dataclass(frozen=True)
class _JoystickSnapshot:
    dead_zone: float
    dead_zone_x: float
    dead_zone_y: float
    expo_x: float
    expo_y: float
    return_mode: ReturnMode
    sticky: bool


class _JoystickConfigAdapter:
    """Bridge JoystickWidget + buttons to the JoystickConfigPanel protocol."""

    def __init__(self, joystick: JoystickWidget, buttons: ControllerButtonsWidget) -> None:
        self._joystick = joystick
        self._buttons = buttons
        self._sticky = False

    # Dead zones ---------------------------------------------------------------
    def get_dead_zone(self) -> float:
        return self._joystick.get_config().dead_zone

    def set_dead_zone(self, v: float) -> None:
        self._joystick.set_dead_zone(v)

    def get_dead_zone_x(self) -> float:
        return self._joystick.get_config().dead_zone_x

    def set_dead_zone_x(self, v: float) -> None:
        self._joystick.set_dead_zone_x(v)

    def get_dead_zone_y(self) -> float:
        return self._joystick.get_config().dead_zone_y

    def set_dead_zone_y(self, v: float) -> None:
        self._joystick.set_dead_zone_y(v)

    # Exponential response -----------------------------------------------------
    def get_expo_x(self) -> float:
        return self._joystick.get_config().expo_x

    def set_expo_x(self, pct: float) -> None:
        self._joystick.set_expo_x(pct)

    def get_expo_y(self) -> float:
        return self._joystick.get_config().expo_y

    def set_expo_y(self, pct: float) -> None:
        self._joystick.set_expo_y(pct)

    # Return mode & sticky buttons --------------------------------------------
    def get_return_mode(self) -> ReturnMode:
        return self._joystick.get_config().return_mode

    def set_return_mode(self, mode: ReturnMode) -> None:
        self._joystick.set_return_mode(mode)

    def get_sticky_buttons(self) -> bool:
        return self._sticky

    def set_sticky_buttons(self, on: bool) -> None:
        self._sticky = bool(on)
        self._buttons.set_sticky_buttons(self._sticky)

    # Snapshot used for persistence -------------------------------------------
    def snapshot(self) -> _JoystickSnapshot:
        cfg = self._joystick.get_config()
        return _JoystickSnapshot(
            dead_zone=cfg.dead_zone,
            dead_zone_x=cfg.dead_zone_x,
            dead_zone_y=cfg.dead_zone_y,
            expo_x=cfg.expo_x,
            expo_y=cfg.expo_y,
            return_mode=cfg.return_mode,
            sticky=self._sticky,
        )


class _HolonomicShiftHandler(QObject):
    """Temporarily force holonomic mode while Shift is held."""

    def __init__(self, twist_service: TwistPublisherService, twist_panel, parent: QWidget) -> None:
        super().__init__(parent)
        self._twist_service = twist_service
        self._twist_panel = twist_panel
        self._last_value = twist_service.get_holonomic()
        self._active = False

        app = QApplication.instance()
        self._app = app
        if self._app is not None:
            self._app.installEventFilter(self)
        parent.destroyed.connect(self._cleanup)

    def eventFilter(self, obj, event):  # noqa: D401 - Qt signature
        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Shift and not self._active:
                self._toggle(True)
        elif event.type() == QEvent.KeyRelease:
            if event.key() == Qt.Key_Shift and self._active:
                self._toggle(False)
        return False

    def _toggle(self, enabled: bool) -> None:
        if enabled:
            self._last_value = self._twist_service.get_holonomic()
            self._twist_service.set_holonomic(True)
            # Update UI to show temporary holonomic state
            self._twist_panel.refresh()
        else:
            self._twist_service.set_holonomic(self._last_value)
            # Restore UI to show actual holonomic state
            self._twist_panel.refresh()
        self._active = enabled
    def _cleanup(self) -> None:
        if self._app is not None:
            self._app.removeEventFilter(self)
        self._app = None


class DynamicTabWidget(QTabWidget):
    """Tab widget whose size hints follow the current page.

    By default QTabWidget reports the max(min/sizeHint) of all pages, which
    prevents shrinking on lightweight tabs. Overriding the hints makes the
    container track the active page instead.
    """

    def minimumSizeHint(self):
        page = self.currentWidget()
        if page is not None:
            try:
                return page.minimumSizeHint()
            except Exception:
                pass
        return super().minimumSizeHint()

    def sizeHint(self):
        page = self.currentWidget()
        if page is not None:
            try:
                return page.sizeHint()
            except Exception:
                pass
        return super().sizeHint()


class JoystickMainWidget(QWidget):
    """Compose joystick controls, publishers, and configuration panels."""

    def __init__(self, ros_node: Node, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._node = ros_node

        self._joy_service = JoyPublisherService(self._node)
        self._twist_service = TwistPublisherService(self._node)
        self._joystick = JoystickWidget()
        self._buttons = ControllerButtonsWidget(sticky_buttons=False)

        self._joy_api = _JoyOutputAPIAdapter(self._joy_service)
        self._twist_api = _TwistOutputAPIAdapter(self._twist_service)
        self._config_api = _JoystickConfigAdapter(self._joystick, self._buttons)

        self._joy_panel = JoyOutputPanel(self._joy_api)
        self._twist_panel = TwistOutputPanel(self._twist_api)
        self._joystick_panel = JoystickConfigPanel(self._config_api)

        self._build_ui()
        self._connect_signals()

        self._shift_handler = _HolonomicShiftHandler(self._twist_service, self._twist_panel, self)

        self.setWindowTitle("Virtual Joystick")
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocusProxy(self._joystick)

    # ------------------------------------------------------------------
    # UI / wiring
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        top_row = QWidget()
        top_layout = QHBoxLayout(top_row)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(12)

        self._joystick.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top_layout.addWidget(self._joystick, 3)

        self._buttons.setMinimumWidth(220)
        self._buttons.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        top_layout.addWidget(self._buttons, 1)

        top_row.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        panels_frame = QFrame()
        panels_frame.setFrameShape(QFrame.NoFrame)
        panels_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        panels_layout = QVBoxLayout(panels_frame)
        panels_layout.setContentsMargins(0, 0, 0, 0)
        panels_layout.setSpacing(8)
        panels_layout.addWidget(self._joy_panel)
        panels_layout.addWidget(self._twist_panel)
        panels_layout.addWidget(self._joystick_panel)
        panels_layout.addStretch(1)

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(12)
        root_layout.addWidget(top_row)
        root_layout.addWidget(panels_frame)
        root_layout.setStretchFactor(top_row, 0)
        root_layout.setStretchFactor(panels_frame, 1)

    def _connect_signals(self) -> None:
        self._joystick.position_changed.connect(self._on_joystick_position)
        self._buttons.button_toggled.connect(self._joy_service.update_button)

    def _on_joystick_position(self, x: float, y: float) -> None:
        self._joy_service.update_axes(x, y)
        self._twist_service.update_from_axes(x, y)

    # ------------------------------------------------------------------
    # Settings persistence
    # ------------------------------------------------------------------

    def save_settings(self, settings) -> None:
        if settings is None:
            return

        settings.set_value("joy/topic", self._joy_service.get_topic())
        settings.set_value("joy/rate_hz", self._joy_service.get_rate_hz())
        settings.set_value("joy/enabled", self._joy_service.is_enabled())

        settings.set_value("twist/topic", self._twist_service.get_topic())
        settings.set_value("twist/rate_hz", self._twist_service.get_rate_hz())
        settings.set_value("twist/enabled", self._twist_service.is_enabled())
        settings.set_value("twist/use_stamped", self._twist_service.get_use_stamped())
        settings.set_value("twist/holonomic", self._twist_service.get_holonomic())
        linear_scale, angular_scale = self._twist_service.get_scales()
        settings.set_value("twist/linear_scale", linear_scale)
        settings.set_value("twist/angular_scale", angular_scale)

        snapshot = self._config_api.snapshot()
        settings.set_value("joystick/dead_zone", snapshot.dead_zone)
        settings.set_value("joystick/dead_zone_x", snapshot.dead_zone_x)
        settings.set_value("joystick/dead_zone_y", snapshot.dead_zone_y)
        settings.set_value("joystick/expo_x", snapshot.expo_x)
        settings.set_value("joystick/expo_y", snapshot.expo_y)
        settings.set_value("joystick/return_mode", snapshot.return_mode.name)
        settings.set_value("joystick/sticky_buttons", snapshot.sticky)

        settings.set_value("panels/joy_collapsed", self._joy_panel.is_collapsed())
        settings.set_value("panels/twist_collapsed", self._twist_panel.is_collapsed())
        settings.set_value("panels/joystick_collapsed", self._joystick_panel.is_collapsed())

        if hasattr(settings, "sync"):
            settings.sync()

    def restore_settings(self, settings) -> None:
        if settings is None:
            self._refresh_panels()
            return

        joy_topic = settings.value("joy/topic", self._joy_service.get_topic())
        joy_rate = _to_float(settings.value("joy/rate_hz", self._joy_service.get_rate_hz()),
                             self._joy_service.get_rate_hz())
        joy_enabled = _to_bool(settings.value("joy/enabled", self._joy_service.is_enabled()),
                               self._joy_service.is_enabled())
        self._joy_service.set_topic(str(joy_topic))
        self._joy_service.set_rate_hz(joy_rate)
        self._joy_service.set_enabled(joy_enabled)

        twist_topic = settings.value("twist/topic", self._twist_service.get_topic())
        twist_rate = _to_float(settings.value("twist/rate_hz", self._twist_service.get_rate_hz()),
                               self._twist_service.get_rate_hz())
        twist_enabled = _to_bool(settings.value("twist/enabled", self._twist_service.is_enabled()),
                                 self._twist_service.is_enabled())
        use_stamped = _to_bool(settings.value("twist/use_stamped", self._twist_service.get_use_stamped()),
                               self._twist_service.get_use_stamped())
        holonomic = _to_bool(settings.value("twist/holonomic", self._twist_service.get_holonomic()),
                             self._twist_service.get_holonomic())
        linear_scale = _to_float(settings.value("twist/linear_scale", 1.0), 1.0)
        angular_scale = _to_float(settings.value("twist/angular_scale", 1.0), 1.0)
        self._twist_service.set_topic(str(twist_topic))
        self._twist_service.set_rate_hz(twist_rate)
        self._twist_service.set_enabled(twist_enabled)
        self._twist_service.set_use_stamped(use_stamped)
        self._twist_service.set_holonomic(holonomic)
        self._twist_service.set_scales(linear_scale, angular_scale)

        dead_zone = _to_float(settings.value("joystick/dead_zone", 0.0), 0.0)
        dead_zone_x = _to_float(settings.value("joystick/dead_zone_x", 0.0), 0.0)
        dead_zone_y = _to_float(settings.value("joystick/dead_zone_y", 0.0), 0.0)
        expo_x = _to_float(settings.value("joystick/expo_x", 0.0), 0.0)
        expo_y = _to_float(settings.value("joystick/expo_y", 0.0), 0.0)
        return_mode_name = settings.value("joystick/return_mode", ReturnMode.BOTH.name)
        sticky = _to_bool(settings.value("joystick/sticky_buttons", False), False)

        self._joystick.set_dead_zone(dead_zone)
        self._joystick.set_dead_zone_x(dead_zone_x)
        self._joystick.set_dead_zone_y(dead_zone_y)
        self._joystick.set_expo_x(expo_x)
        self._joystick.set_expo_y(expo_y)
        try:
            self._config_api.set_return_mode(ReturnMode[str(return_mode_name)])
        except KeyError:  # pragma: no cover - invalid persisted data
            self._config_api.set_return_mode(ReturnMode.BOTH)
        self._config_api.set_sticky_buttons(sticky)

        joy_default = True  # Default collapsed state for panels
        twist_default = True
        joystick_default = True

        joy_collapsed = _to_bool(settings.value("panels/joy_collapsed", joy_default), joy_default)
        twist_collapsed = _to_bool(settings.value("panels/twist_collapsed", twist_default), twist_default)
        joystick_collapsed = _to_bool(settings.value("panels/joystick_collapsed", joystick_default), joystick_default)
        self._joy_panel.set_collapsed(joy_collapsed)
        self._twist_panel.set_collapsed(twist_collapsed)
        self._joystick_panel.set_collapsed(joystick_collapsed)

        # Complete the initial setup of publishers after all settings are applied
        if hasattr(self._twist_service, 'complete_initial_setup'):
            self._twist_service.complete_initial_setup()

        self._refresh_panels()

    def _refresh_panels(self) -> None:
        self._joy_panel.refresh()
        self._twist_panel.refresh()
        self._joystick_panel.refresh()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        self._joy_service.shutdown()
        self._twist_service.shutdown()
        if self._shift_handler is not None:
            self._shift_handler._cleanup()

    def closeEvent(self, event) -> None:  # noqa: D401 - Qt signature
        self.shutdown()
        super().closeEvent(event)
