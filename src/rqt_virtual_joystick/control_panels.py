from __future__ import annotations

from contextlib import contextmanager
from typing import Optional, Protocol, Tuple

from python_qt_binding.QtCore import Qt, QSize, pyqtSlot
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QSlider,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from .joystick_widget import ReturnMode
from .widgets.segmented_toggle_widget import SegmentedToggle


_QT_MAX_SIZE = 16777215


@contextmanager
def blocked(widget):
    widget.blockSignals(True)
    try:
        yield widget
    finally:
        widget.blockSignals(False)


class SliderRow:
    """Convenience wrapper pairing a slider with a display label."""

    def __init__(self, parent, min_v: int, max_v: int, suffix: str = "") -> None:
        self._slider = QSlider(Qt.Horizontal, parent)
        self._slider.setRange(min_v, max_v)
        self._label = QLabel(parent)
        self._suffix = suffix

    def slider(self) -> QSlider:
        return self._slider

    def label(self) -> QLabel:
        return self._label

    def set(self, value: int) -> None:
        with blocked(self._slider):
            self._slider.setValue(value)
        self._label.setText(f"{value}{self._suffix}")


class JoyOutputAPI(Protocol):
    def get_enabled(self) -> bool: ...
    def set_enabled(self, on: bool) -> None: ...
    def get_topic(self) -> str: ...
    def set_topic(self, name: str) -> None: ...
    def get_rate_hz(self) -> float: ...
    def set_rate_hz(self, hz: float) -> None: ...


class TwistOutputAPI(Protocol):
    def get_enabled(self) -> bool: ...
    def set_enabled(self, on: bool) -> None: ...
    def get_topic(self) -> str: ...
    def set_topic(self, name: str) -> None: ...
    def get_rate_hz(self) -> float: ...
    def set_rate_hz(self, hz: float) -> None: ...
    def get_scales(self) -> Tuple[float, float]: ...
    def set_scales(self, linear: float, angular: float) -> None: ...
    def get_use_stamped(self) -> bool: ...
    def set_use_stamped(self, on: bool) -> None: ...
    def get_holonomic(self) -> bool: ...
    def set_holonomic(self, on: bool) -> None: ...


class JoystickConfigAPI(Protocol):
    def get_dead_zone(self) -> float: ...
    def set_dead_zone(self, v: float) -> None: ...
    def get_dead_zone_x(self) -> float: ...
    def set_dead_zone_x(self, v: float) -> None: ...
    def get_dead_zone_y(self) -> float: ...
    def set_dead_zone_y(self, v: float) -> None: ...
    def get_expo_x(self) -> float: ...
    def set_expo_x(self, pct: float) -> None: ...
    def get_expo_y(self) -> float: ...
    def set_expo_y(self, pct: float) -> None: ...
    def get_return_mode(self) -> ReturnMode: ...
    def set_return_mode(self, mode: ReturnMode) -> None: ...
    def get_sticky_buttons(self) -> bool: ...
    def set_sticky_buttons(self, on: bool) -> None: ...



class _ControlPanel(QFrame):
    LABEL_MIN_WIDTH = 90
    VALUE_PLACEHOLDER_WIDTH = 40

    def __init__(self, title: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._label_min_width = self.LABEL_MIN_WIDTH
        self._value_placeholder_width = self.VALUE_PLACEHOLDER_WIDTH

        self.setObjectName("control-panel")

        self._header_button = QToolButton(self)
        self._header_button.setObjectName("control-panel-toggle")
        self._header_button.setText(title)
        self._header_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self._header_button.setArrowType(Qt.RightArrow)
        self._header_button.setCheckable(True)
        self._header_button.setChecked(False)
        self._header_button.setAutoRaise(True)
        self._header_button.setFocusPolicy(Qt.NoFocus)
        self._header_button.setIconSize(QSize(12, 12))

        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(6)
        header_layout.addWidget(self._header_button)
        header_layout.addStretch(1)

        self._separator = QFrame(self)
        self._separator.setObjectName("control-panel-separator")
        self._separator.setFrameShape(QFrame.HLine)
        self._separator.setFrameShadow(QFrame.Sunken)

        self._body_widget = QWidget(self)
        self._body_layout = QVBoxLayout()
        # self._body_layout.setContentsMargins(0, 0, 0, 0)
        self._body_layout.setSpacing(12)
        self._body_widget.setLayout(self._body_layout)
        # self._body_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        outer_layout = QVBoxLayout()
        outer_layout.setContentsMargins(10, 10, 10, 10)
        outer_layout.setSpacing(6)
        outer_layout.addLayout(header_layout)
        outer_layout.addWidget(self._separator)
        outer_layout.addWidget(self._body_widget)
        self.setLayout(outer_layout)

        self._header_button.toggled.connect(self._on_header_toggled)
        self._apply_frame_style()
        # Start collapsed by default - settings will override if needed
        self.setProperty("collapsed", True)
        self._header_button.setChecked(False)  # Collapsed state
        self._separator.setVisible(False)
        self._body_widget.setVisible(False)
        self._body_widget.setMinimumHeight(0)
        self._body_widget.setMaximumHeight(0)

    def _label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setMinimumWidth(self._label_min_width)
        return label

    def _value_label(self, text: str = "") -> QLabel:
        label = QLabel(text)
        label.setFixedWidth(self._value_placeholder_width)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        return label

    def _placeholder(self) -> QLabel:
        placeholder = QLabel("")
        placeholder.setFixedWidth(self._value_placeholder_width)
        return placeholder

    def _apply_frame_style(self) -> None:
        self.setFrameShape(QFrame.NoFrame)
        self.setStyleSheet(
            """
            QFrame#control-panel {
                background-color: #2d3036;
                border: 1px solid #444952;
                border-radius: 8px;
            }
            QToolButton#control-panel-toggle {
                color: #f4f5f7;
                font-weight: 600;
                padding: 4px 8px;
                background-color: transparent;
            }
            QToolButton#control-panel-toggle:hover {
                color: #ffffff;
                background-color: transparent;
            }
            QToolButton#control-panel-toggle:pressed {
                background-color: transparent;
            }
            QFrame#control-panel QLabel {
                color: #d9dce2;
            }
            QFrame#control-panel-separator {
                border: none;
                background-color: #3a3f48;
                min-height: 1px;
                max-height: 1px;
            }
            """
        )

    def _on_header_toggled(self, expanded: bool) -> None:
        self._header_button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self._separator.setVisible(expanded)
        self._body_widget.setVisible(expanded)
        if expanded:
            self._body_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            self._body_widget.setMaximumHeight(_QT_MAX_SIZE)
            self._body_widget.setMinimumHeight(0)
        else:
            self._body_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self._body_widget.setMaximumHeight(0)
            self._body_widget.setMinimumHeight(0)
        self.setProperty("collapsed", not expanded)
        style = self.style()
        if style is not None:
            style.unpolish(self)
            style.polish(self)
        self.update()
        self.updateGeometry()

    def set_collapsed(self, collapsed: bool) -> None:
        self._header_button.setChecked(not collapsed)

    def is_collapsed(self) -> bool:
        return not self._header_button.isChecked()


class JoyOutputPanel(_ControlPanel):
    def __init__(self, api: JoyOutputAPI, parent: Optional[QWidget] = None):
        super().__init__("Joy Output", parent)
        self._api = api
        self._committed_topic = self._api.get_topic()
        self._build_ui()
        self._wire()
        self.refresh()

    def refresh(self) -> None:
        self._committed_topic = self._api.get_topic()
        with blocked(self._topic_combo):
            self._topic_combo.setCurrentText(self._committed_topic)

        enabled = self._api.get_enabled()
        with blocked(self._publish_toggle):
            self._publish_toggle.setChecked(enabled)

        rate = max(1, int(round(self._api.get_rate_hz())))
        with blocked(self._rate_slider):
            self._rate_slider.setValue(rate)
        self._rate_label.setText(f"{rate} Hz")

    # Backwards compatibility for old code
    def _build_ui(self) -> None:
        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setVerticalSpacing(5)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        layout.addWidget(self._label("Publish:"), row, 0)
        self._publish_toggle = SegmentedToggle(false_label="Disabled", true_label="Enabled")
        layout.addWidget(self._publish_toggle, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Topic:"), row, 0)
        self._topic_combo = QComboBox()
        self._topic_combo.setEditable(True)
        self._topic_combo.addItems(["joy", "teleop/joy", "input/joy"])
        self._topic_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._topic_combo, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Rate:"), row, 0)
        self._rate_slider = QSlider(Qt.Horizontal)
        self._rate_slider.setRange(1, 100)
        self._rate_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._rate_slider, row, 1)
        self._rate_label = self._value_label()
        layout.addWidget(self._rate_label, row, 2)

        self._body_layout.addLayout(layout)

    def _wire(self) -> None:
        self._publish_toggle.toggled.connect(self._api.set_enabled)
        self._topic_combo.activated[str].connect(self._on_topic_activated)
        topic_line_edit = self._topic_combo.lineEdit()
        if topic_line_edit:
            topic_line_edit.returnPressed.connect(self._on_topic_return_pressed)
        self._rate_slider.valueChanged.connect(self._on_rate_changed)

    @pyqtSlot(str)
    def _on_topic_activated(self, topic_name: str) -> None:
        self._commit_topic(topic_name)

    @pyqtSlot()
    def _on_topic_return_pressed(self) -> None:
        self._commit_topic(self._topic_combo.currentText())

    def _commit_topic(self, topic_name: str) -> None:
        try:
            self._api.set_topic(topic_name)
            self._committed_topic = self._api.get_topic()
        except ValueError:
            with blocked(self._topic_combo):
                self._topic_combo.setCurrentText(self._committed_topic)

    @pyqtSlot(int)
    def _on_rate_changed(self, value: int) -> None:
        try:
            self._api.set_rate_hz(float(value))
            self._rate_label.setText(f"{value} Hz")
        except ValueError:
            rate = max(1, int(round(self._api.get_rate_hz())))
            with blocked(self._rate_slider):
                self._rate_slider.setValue(rate)
            self._rate_label.setText(f"{rate} Hz")


class TwistOutputPanel(_ControlPanel):
    def __init__(self, api: TwistOutputAPI, parent: Optional[QWidget] = None):
        super().__init__("Twist Output", parent)
        self._api = api
        self._committed_topic = self._api.get_topic()
        self._build_ui()
        self._wire()
        self.refresh()

    def refresh(self) -> None:
        self._committed_topic = self._api.get_topic()
        with blocked(self._twist_topic_combo):
            self._twist_topic_combo.setCurrentText(self._committed_topic)

        enabled = self._api.get_enabled()
        with blocked(self._twist_publish_toggle):
            self._twist_publish_toggle.setChecked(enabled)

        use_stamped = self._api.get_use_stamped()
        with blocked(self._twist_stamped_checkbox):
            self._twist_stamped_checkbox.setChecked(use_stamped)

        rate = max(0, int(round(self._api.get_rate_hz())))
        with blocked(self._twist_rate_slider):
            self._twist_rate_slider.setValue(rate)
        self._twist_rate_label.setText(f"{rate} Hz")

        linear, angular = self._api.get_scales()
        with blocked(self._twist_linear_spin):
            self._twist_linear_spin.setValue(linear)
        with blocked(self._twist_angular_spin):
            self._twist_angular_spin.setValue(angular)

        holonomic = self._api.get_holonomic()
        with blocked(self._twist_holonomic_checkbox):
            self._twist_holonomic_checkbox.setChecked(holonomic)

    def _build_ui(self) -> None:
        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setVerticalSpacing(5)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        layout.addWidget(self._label("Publish:"), row, 0)
        self._twist_publish_toggle = SegmentedToggle(false_label="Disabled", true_label="Enabled")
        layout.addWidget(self._twist_publish_toggle, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Stamp Header:"), row, 0)
        self._twist_stamped_checkbox = QCheckBox()
        layout.addWidget(self._twist_stamped_checkbox, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Topic:"), row, 0)
        self._twist_topic_combo = QComboBox()
        self._twist_topic_combo.setEditable(True)
        self._twist_topic_combo.addItems(["cmd_vel", "robot/cmd_vel", "teleop/cmd_vel"])
        layout.addWidget(self._twist_topic_combo, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Rate:"), row, 0)
        self._twist_rate_slider = QSlider(Qt.Horizontal)
        self._twist_rate_slider.setRange(0, 100)
        layout.addWidget(self._twist_rate_slider, row, 1)
        self._twist_rate_label = self._value_label()
        layout.addWidget(self._twist_rate_label, row, 2)

        row += 1
        layout.addWidget(self._label("Linear Scale:"), row, 0)
        self._twist_linear_spin = QDoubleSpinBox()
        self._twist_linear_spin.setRange(0.0, 10.0)
        self._twist_linear_spin.setDecimals(3)
        self._twist_linear_spin.setSingleStep(0.1)
        layout.addWidget(self._twist_linear_spin, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Angular Scale:"), row, 0)
        self._twist_angular_spin = QDoubleSpinBox()
        self._twist_angular_spin.setRange(0.0, 10.0)
        self._twist_angular_spin.setDecimals(3)
        self._twist_angular_spin.setSingleStep(0.1)
        layout.addWidget(self._twist_angular_spin, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        holonomic_container = QWidget()
        holonomic_layout = QHBoxLayout()
        holonomic_layout.setContentsMargins(0, 0, 0, 0)
        holonomic_layout.setSpacing(4)
        holonomic_layout.addWidget(self._label("Holonomic:"))
        self._twist_holonomic_checkbox = QCheckBox()
        holonomic_layout.addWidget(self._twist_holonomic_checkbox)
        hint = QLabel("Hold Shift to temporarily enable")
        hint.setStyleSheet("color: #a0a0a0; font-size: 11px;")
        holonomic_layout.addWidget(hint)
        holonomic_layout.addStretch(1)
        holonomic_container.setLayout(holonomic_layout)
        layout.addWidget(holonomic_container, row, 0, 1, 3)

        self._body_layout.addLayout(layout)

    def _wire(self) -> None:
        self._twist_publish_toggle.toggled.connect(self._api.set_enabled)
        self._twist_stamped_checkbox.toggled.connect(self._api.set_use_stamped)
        self._twist_topic_combo.activated[str].connect(self._on_topic_activated)
        topic_line_edit = self._twist_topic_combo.lineEdit()
        if topic_line_edit:
            topic_line_edit.returnPressed.connect(self._on_topic_return_pressed)
        self._twist_rate_slider.valueChanged.connect(self._on_rate_changed)
        self._twist_linear_spin.valueChanged.connect(self._on_linear_changed)
        self._twist_angular_spin.valueChanged.connect(self._on_angular_changed)
        self._twist_holonomic_checkbox.toggled.connect(self._api.set_holonomic)

    @pyqtSlot(str)
    def _on_topic_activated(self, topic: str) -> None:
        self._commit_topic(topic)

    @pyqtSlot()
    def _on_topic_return_pressed(self) -> None:
        self._commit_topic(self._twist_topic_combo.currentText())

    def _commit_topic(self, topic: str) -> None:
        try:
            self._api.set_topic(topic)
            self._committed_topic = self._api.get_topic()
        except ValueError:
            with blocked(self._twist_topic_combo):
                self._twist_topic_combo.setCurrentText(self._committed_topic)

    @pyqtSlot(int)
    def _on_rate_changed(self, value: int) -> None:
        try:
            self._api.set_rate_hz(float(value))
            self._twist_rate_label.setText(f"{value} Hz")
        except ValueError:
            rate = max(0, int(round(self._api.get_rate_hz())))
            with blocked(self._twist_rate_slider):
                self._twist_rate_slider.setValue(rate)
            self._twist_rate_label.setText(f"{rate} Hz")

    @pyqtSlot(float)
    def _on_linear_changed(self, value: float) -> None:
        try:
            _, angular = self._api.get_scales()
            self._api.set_scales(value, angular)
        except ValueError:
            with blocked(self._twist_linear_spin):
                self._twist_linear_spin.setValue(self._api.get_scales()[0])

    @pyqtSlot(float)
    def _on_angular_changed(self, value: float) -> None:
        try:
            linear, _ = self._api.get_scales()
            self._api.set_scales(linear, value)
        except ValueError:
            with blocked(self._twist_angular_spin):
                self._twist_angular_spin.setValue(self._api.get_scales()[1])


class JoystickConfigPanel(_ControlPanel):
    def __init__(self, api: JoystickConfigAPI, parent: Optional[QWidget] = None):
        super().__init__("Joystick", parent)
        self._api = api
        self._build_ui()
        self._wire()
        self.refresh()

    def refresh(self) -> None:
        self._dead_zone_row.set(int(round(self._api.get_dead_zone() * 100)))
        self._dead_zone_x_row.set(int(round(self._api.get_dead_zone_x() * 100)))
        self._dead_zone_y_row.set(int(round(self._api.get_dead_zone_y() * 100)))
        self._expo_x_row.set(int(round(self._api.get_expo_x())))
        self._expo_y_row.set(int(round(self._api.get_expo_y())))

        mode = self._api.get_return_mode()
        index = max(0, self._return_mode_combo.findData(mode))
        with blocked(self._return_mode_combo):
            self._return_mode_combo.setCurrentIndex(index)

        sticky = self._api.get_sticky_buttons()
        with blocked(self._sticky_buttons_checkbox):
            self._sticky_buttons_checkbox.setChecked(sticky)

    def _build_ui(self) -> None:
        layout = QGridLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setVerticalSpacing(5)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        layout.addWidget(self._label("Dead Zone:"), row, 0)
        self._dead_zone_row = SliderRow(self, 0, 90, suffix=" %")
        layout.addWidget(self._dead_zone_row.slider(), row, 1)
        layout.addWidget(self._dead_zone_row.label(), row, 2)

        row += 1
        layout.addWidget(self._label("Dead Zone X:"), row, 0)
        self._dead_zone_x_row = SliderRow(self, 0, 90, suffix=" %")
        layout.addWidget(self._dead_zone_x_row.slider(), row, 1)
        layout.addWidget(self._dead_zone_x_row.label(), row, 2)

        row += 1
        layout.addWidget(self._label("Dead Zone Y:"), row, 0)
        self._dead_zone_y_row = SliderRow(self, 0, 90, suffix=" %")
        layout.addWidget(self._dead_zone_y_row.slider(), row, 1)
        layout.addWidget(self._dead_zone_y_row.label(), row, 2)

        row += 1
        layout.addWidget(self._label("Expo X:"), row, 0)
        self._expo_x_row = SliderRow(self, 0, 100, suffix=" %")
        layout.addWidget(self._expo_x_row.slider(), row, 1)
        layout.addWidget(self._expo_x_row.label(), row, 2)

        row += 1
        layout.addWidget(self._label("Expo Y:"), row, 0)
        self._expo_y_row = SliderRow(self, 0, 100, suffix=" %")
        layout.addWidget(self._expo_y_row.slider(), row, 1)
        layout.addWidget(self._expo_y_row.label(), row, 2)

        row += 1
        layout.addWidget(self._label("Auto Return:"), row, 0)
        self._return_mode_combo = QComboBox()
        self._return_mode_combo.addItem("Both Axes", ReturnMode.BOTH)
        self._return_mode_combo.addItem("X Only", ReturnMode.HORIZONTAL)
        self._return_mode_combo.addItem("Y Only", ReturnMode.VERTICAL)
        self._return_mode_combo.addItem("Disabled", ReturnMode.NONE)
        layout.addWidget(self._return_mode_combo, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Sticky Buttons:"), row, 0)
        self._sticky_buttons_checkbox = QCheckBox()
        layout.addWidget(self._sticky_buttons_checkbox, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        self._body_layout.addLayout(layout)

    def _wire(self) -> None:
        self._dead_zone_row.slider().valueChanged.connect(self._on_dead_zone_changed)
        self._dead_zone_x_row.slider().valueChanged.connect(self._on_dead_zone_x_changed)
        self._dead_zone_y_row.slider().valueChanged.connect(self._on_dead_zone_y_changed)
        self._expo_x_row.slider().valueChanged.connect(self._on_expo_x_changed)
        self._expo_y_row.slider().valueChanged.connect(self._on_expo_y_changed)
        self._return_mode_combo.currentIndexChanged.connect(self._on_return_mode_changed)
        self._sticky_buttons_checkbox.toggled.connect(self._on_sticky_buttons_changed)

    @pyqtSlot(int)
    def _on_dead_zone_changed(self, value: int) -> None:
        self._api.set_dead_zone(value / 100.0)
        self._dead_zone_row.label().setText(f"{value} %")

    @pyqtSlot(int)
    def _on_dead_zone_x_changed(self, value: int) -> None:
        self._api.set_dead_zone_x(value / 100.0)
        self._dead_zone_x_row.label().setText(f"{value} %")

    @pyqtSlot(int)
    def _on_dead_zone_y_changed(self, value: int) -> None:
        self._api.set_dead_zone_y(value / 100.0)
        self._dead_zone_y_row.label().setText(f"{value} %")

    @pyqtSlot(int)
    def _on_expo_x_changed(self, value: int) -> None:
        self._api.set_expo_x(float(value))
        self._expo_x_row.label().setText(f"{value} %")

    @pyqtSlot(int)
    def _on_expo_y_changed(self, value: int) -> None:
        self._api.set_expo_y(float(value))
        self._expo_y_row.label().setText(f"{value} %")

    @pyqtSlot(int)
    def _on_return_mode_changed(self, index: int) -> None:
        mode = self._return_mode_combo.itemData(index)
        if isinstance(mode, ReturnMode):
            self._api.set_return_mode(mode)

    @pyqtSlot(bool)
    def _on_sticky_buttons_changed(self, enabled: bool) -> None:
        self._api.set_sticky_buttons(bool(enabled))


__all__ = [
    "JoyOutputPanel",
    "TwistOutputPanel",
    "JoystickConfigPanel",
]
