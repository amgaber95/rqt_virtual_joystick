"""RQt plugin module wiring the joystick widget into the GUI."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional

from python_qt_binding.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
from rqt_gui_py.plugin import Plugin

from .widgets import JoystickPosition, JoystickWidget


__all__ = ['VirtualJoystick']


@dataclass(frozen=True)
class JoystickState:
    """Immutable state snapshot for the virtual joystick view."""

    position: JoystickPosition = JoystickPosition(0.0, 0.0)

    def with_position(self, position: JoystickPosition) -> 'JoystickState':
        """Create a new state reflecting the provided position."""

        return replace(self, position=position)


class VirtualJoystickWidget(QWidget):
    """Composite widget containing the joystick pad and simple controls."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._state = JoystickState()

        self._joystick = JoystickWidget(self)
        self._joystick.position_changed.connect(self._on_position_changed)
        self._joystick.interaction_finished.connect(self._on_interaction_finished)

        self._position_label = QLabel('X: 0.00  Y: 0.00', self)
        self._position_label.setObjectName('JoystickPositionLabel')

        self._reset_button = QPushButton('Reset', self)
        self._reset_button.clicked.connect(self._joystick.reset)

        self._build_layout()
        self.setObjectName('VirtualJoystickWidget')
        self.setWindowTitle('Virtual Joystick')

    def shutdown(self) -> None:
        """Perform shutdown housekeeping for the widget."""

    def _build_layout(self) -> None:
        """Assemble the widget layout."""

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)

        main_layout.addWidget(self._joystick, stretch=1)

        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(8)
        controls_layout.addWidget(self._position_label)
        controls_layout.addStretch(1)
        controls_layout.addWidget(self._reset_button)

        main_layout.addLayout(controls_layout)

    def _on_position_changed(self, x_axis: float, y_axis: float) -> None:
        """Handle joystick motion updates from the pad."""

        position = JoystickPosition(x=x_axis, y=y_axis)
        self._state = self._state.with_position(position)
        self._update_position_readout(position)

    def _on_interaction_finished(self) -> None:
        """Reset the view when the joystick interaction ends."""

        neutral = JoystickPosition(0.0, 0.0)
        self._state = self._state.with_position(neutral)
        self._update_position_readout(neutral)

    def _update_position_readout(self, position: JoystickPosition) -> None:
        """Update the numeric axis readout label."""

        self._position_label.setText(f'X: {position.x:.2f}  Y: {position.y:.2f}')


class VirtualJoystick(Plugin):
    """Entry point exposing the virtual joystick widget to RQt."""

    def __init__(self, context) -> None:
        super().__init__(context)
        self.setObjectName('VirtualJoystick')

        self._widget = VirtualJoystickWidget()
        if context.serial_number() > 1:
            window_title = f'{self._widget.windowTitle()} ({context.serial_number()})'
            self._widget.setWindowTitle(window_title)

        context.add_widget(self._widget)

    def shutdown_plugin(self) -> None:
        """Handle plugin shutdown by delegating to the widget."""

        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings) -> None:
        """Persist plugin configuration (not implemented for the minimal widget)."""

        del plugin_settings, instance_settings

    def restore_settings(self, plugin_settings, instance_settings) -> None:
        """Restore plugin configuration (not implemented for the minimal widget)."""

        del plugin_settings, instance_settings
