"""Reusable control panels for configuring the virtual joystick plugin."""

from __future__ import annotations

from typing import Optional

from python_qt_binding.QtCore import Qt, pyqtSlot
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QSlider,
    QWidget,
)

from .config_manager import (
    ConfigurationManager,
    RETURN_MODE_BOTH,
    RETURN_MODE_HORIZONTAL,
    RETURN_MODE_NONE,
    RETURN_MODE_VERTICAL,
)
from .segmented_toggle import SegmentedToggle


class _ControlPanel(QGroupBox):
    """Shared helpers for control group boxes."""

    LABEL_MIN_WIDTH = 90
    VALUE_PLACEHOLDER_WIDTH = 40

    def __init__(self, title: str, config_manager: ConfigurationManager, parent: Optional[QWidget] = None):
        super().__init__(title, parent)
        self._config_manager = config_manager
        self._label_min_width = self.LABEL_MIN_WIDTH
        self._value_placeholder_width = self.VALUE_PLACEHOLDER_WIDTH
        self._apply_groupbox_style()

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

    def _apply_groupbox_style(self) -> None:
        self.setStyleSheet(
            """
            QGroupBox {
                font-weight: bold;
                border: 1px solid #3A3A3A;
                margin-top: 0.5em;
                padding: 0px;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 8px;
                top: -1px;
                padding: 0px 4px;
            }
            """
        )


class JoyOutputPanel(_ControlPanel):
    """Controls for configuring Joy message publishing."""

    def __init__(self, config_manager: ConfigurationManager, parent: Optional[QWidget] = None):
        super().__init__("Joy Output", config_manager, parent)
        self._committed_topic_name = self._config_manager.get_topic_name()
        self._build_ui()
        self._connect_signals()
        self.refresh_from_config()

    def refresh_from_config(self) -> None:
        self._committed_topic_name = self._config_manager.get_topic_name()

        self._topic_combo.blockSignals(True)
        self._topic_combo.setCurrentText(self._committed_topic_name)
        self._topic_combo.blockSignals(False)

        enabled = self._config_manager.is_publish_enabled()
        self._publish_toggle.blockSignals(True)
        self._publish_toggle.setChecked(enabled)
        self._publish_toggle.blockSignals(False)

        rate = int(self._config_manager.get_publish_rate())
        self._rate_slider.blockSignals(True)
        self._rate_slider.setValue(rate)
        self._rate_slider.blockSignals(False)
        self._rate_label.setText(f"{rate} Hz")

    def _build_ui(self) -> None:
        layout = QGridLayout()
        layout.setVerticalSpacing(8)
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

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self._publish_toggle.toggled.connect(self._config_manager.set_publish_enabled)
        self._topic_combo.activated[str].connect(self._on_topic_activated)
        topic_line_edit = self._topic_combo.lineEdit()
        if topic_line_edit:
            topic_line_edit.returnPressed.connect(self._on_topic_return_pressed)

        self._rate_slider.valueChanged.connect(self._on_rate_changed)

        self._config_manager.publish_enabled_changed.connect(self._on_publish_enabled_changed)
        self._config_manager.topic_changed.connect(self._on_topic_changed)
        self._config_manager.rate_changed.connect(self._on_rate_updated)

    @pyqtSlot(str)
    def _on_topic_activated(self, topic_name: str) -> None:
        self._commit_topic_name(topic_name)

    @pyqtSlot()
    def _on_topic_return_pressed(self) -> None:
        self._commit_topic_name(self._topic_combo.currentText())

    def _commit_topic_name(self, topic_name: str) -> None:
        try:
            self._config_manager.set_topic_name(topic_name)
            self._committed_topic_name = self._config_manager.get_topic_name()
        except ValueError:
            self._restore_committed_topic()

    def _restore_committed_topic(self) -> None:
        self._topic_combo.blockSignals(True)
        self._topic_combo.setCurrentText(self._committed_topic_name)
        self._topic_combo.blockSignals(False)

    @pyqtSlot(int)
    def _on_rate_changed(self, value: int) -> None:
        try:
            self._config_manager.set_publish_rate(float(value))
            self._rate_label.setText(f"{value} Hz")
        except ValueError:
            pass

    @pyqtSlot(bool)
    def _on_publish_enabled_changed(self, enabled: bool) -> None:
        self._publish_toggle.blockSignals(True)
        self._publish_toggle.setChecked(enabled)
        self._publish_toggle.blockSignals(False)

    @pyqtSlot(str)
    def _on_topic_changed(self, topic_name: str) -> None:
        self._committed_topic_name = topic_name
        self._topic_combo.blockSignals(True)
        self._topic_combo.setCurrentText(topic_name)
        self._topic_combo.blockSignals(False)

    @pyqtSlot(float)
    def _on_rate_updated(self, value: float) -> None:
        rate = int(value)
        self._rate_slider.blockSignals(True)
        self._rate_slider.setValue(rate)
        self._rate_slider.blockSignals(False)
        self._rate_label.setText(f"{rate} Hz")


class TwistOutputPanel(_ControlPanel):
    """Controls for configuring Twist message publishing."""

    def __init__(self, config_manager: ConfigurationManager, parent: Optional[QWidget] = None):
        super().__init__("Twist Output", config_manager, parent)
        self._committed_topic_name = self._config_manager.get_twist_topic()
        self._build_ui()
        self._connect_signals()
        self.refresh_from_config()

    def refresh_from_config(self) -> None:
        self._committed_topic_name = self._config_manager.get_twist_topic()

        self._twist_topic_combo.blockSignals(True)
        self._twist_topic_combo.setCurrentText(self._committed_topic_name)
        self._twist_topic_combo.blockSignals(False)

        enabled = self._config_manager.is_twist_publish_enabled()
        self._twist_publish_toggle.blockSignals(True)
        self._twist_publish_toggle.setChecked(enabled)
        self._twist_publish_toggle.blockSignals(False)

        rate = int(self._config_manager.get_twist_publish_rate())
        self._twist_rate_slider.blockSignals(True)
        self._twist_rate_slider.setValue(rate)
        self._twist_rate_slider.blockSignals(False)
        self._twist_rate_label.setText(f"{rate} Hz")

        linear_scale, angular_scale = self._config_manager.get_twist_scales()
        self._twist_linear_spin.blockSignals(True)
        self._twist_linear_spin.setValue(linear_scale)
        self._twist_linear_spin.blockSignals(False)
        self._twist_angular_spin.blockSignals(True)
        self._twist_angular_spin.setValue(angular_scale)
        self._twist_angular_spin.blockSignals(False)

        holonomic = self._config_manager.is_twist_holonomic_enabled()
        self._twist_holonomic_checkbox.blockSignals(True)
        self._twist_holonomic_checkbox.setChecked(holonomic)
        self._twist_holonomic_checkbox.blockSignals(False)

    def _build_ui(self) -> None:
        layout = QGridLayout()
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        layout.addWidget(self._label("Publish:"), row, 0)
        self._twist_publish_toggle = SegmentedToggle(false_label="Disabled", true_label="Enabled")
        layout.addWidget(self._twist_publish_toggle, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Topic:"), row, 0)
        self._twist_topic_combo = QComboBox()
        self._twist_topic_combo.setEditable(True)
        self._twist_topic_combo.addItems(["/cmd_vel", "cmd_vel", "robot/cmd_vel"])
        self._twist_topic_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._twist_topic_combo, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Rate:"), row, 0)
        self._twist_rate_slider = QSlider(Qt.Horizontal)
        self._twist_rate_slider.setRange(1, 100)
        self._twist_rate_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._twist_rate_slider, row, 1)
        self._twist_rate_label = self._value_label()
        layout.addWidget(self._twist_rate_label, row, 2)

        row += 1
        layout.addWidget(self._label("Linear Scale:"), row, 0)
        self._twist_linear_spin = QDoubleSpinBox()
        self._twist_linear_spin.setDecimals(2)
        self._twist_linear_spin.setRange(0.0, 5.0)
        self._twist_linear_spin.setSingleStep(0.1)
        layout.addWidget(self._twist_linear_spin, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Angular Scale:"), row, 0)
        self._twist_angular_spin = QDoubleSpinBox()
        self._twist_angular_spin.setDecimals(2)
        self._twist_angular_spin.setRange(0.0, 5.0)
        self._twist_angular_spin.setSingleStep(0.1)
        layout.addWidget(self._twist_angular_spin, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Holonomic:"), row, 0)
        holonomic_container = QWidget()
        holonomic_layout = QHBoxLayout()
        holonomic_layout.setContentsMargins(0, 0, 0, 0)
        holonomic_layout.setSpacing(6)
        self._twist_holonomic_checkbox = QCheckBox()
        holonomic_layout.addWidget(self._twist_holonomic_checkbox)
        self._twist_holonomic_hint = QLabel("Hold Shift to temporarily enable")
        self._twist_holonomic_hint.setStyleSheet("color: #a0a0a0; font-size: 11px;")
        holonomic_layout.addWidget(self._twist_holonomic_hint)
        holonomic_layout.addStretch(1)
        holonomic_container.setLayout(holonomic_layout)
        layout.addWidget(holonomic_container, row, 1, 1, 2)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self._twist_publish_toggle.toggled.connect(self._config_manager.set_twist_publish_enabled)
        self._twist_topic_combo.activated[str].connect(self._on_topic_activated)
        topic_line_edit = self._twist_topic_combo.lineEdit()
        if topic_line_edit:
            topic_line_edit.returnPressed.connect(self._on_topic_return_pressed)

        self._twist_rate_slider.valueChanged.connect(self._on_rate_changed)
        self._twist_linear_spin.valueChanged.connect(self._on_scale_changed)
        self._twist_angular_spin.valueChanged.connect(self._on_scale_changed)
        self._twist_holonomic_checkbox.toggled.connect(self._config_manager.set_twist_holonomic)

        self._config_manager.twist_publish_enabled_changed.connect(self._on_enabled_changed)
        self._config_manager.twist_topic_changed.connect(self._on_topic_changed)
        self._config_manager.twist_rate_changed.connect(self._on_rate_updated)
        self._config_manager.twist_scales_changed.connect(self._on_scales_updated)
        self._config_manager.twist_holonomic_changed.connect(self._on_holonomic_changed)

    @pyqtSlot(str)
    def _on_topic_activated(self, topic_name: str) -> None:
        self._commit_topic_name(topic_name)

    @pyqtSlot()
    def _on_topic_return_pressed(self) -> None:
        self._commit_topic_name(self._twist_topic_combo.currentText())

    def _commit_topic_name(self, topic_name: str) -> None:
        try:
            self._config_manager.set_twist_topic(topic_name)
            self._committed_topic_name = self._config_manager.get_twist_topic()
        except ValueError:
            self._restore_committed_topic()

    def _restore_committed_topic(self) -> None:
        self._twist_topic_combo.blockSignals(True)
        self._twist_topic_combo.setCurrentText(self._committed_topic_name)
        self._twist_topic_combo.blockSignals(False)

    @pyqtSlot(int)
    def _on_rate_changed(self, value: int) -> None:
        try:
            self._config_manager.set_twist_publish_rate(float(value))
            self._twist_rate_label.setText(f"{value} Hz")
        except ValueError:
            pass

    @pyqtSlot(float)
    def _on_scale_changed(self, _value: float) -> None:
        try:
            self._config_manager.set_twist_scales(
                self._twist_linear_spin.value(),
                self._twist_angular_spin.value(),
            )
        except ValueError:
            pass

    @pyqtSlot(bool)
    def _on_enabled_changed(self, enabled: bool) -> None:
        self._twist_publish_toggle.blockSignals(True)
        self._twist_publish_toggle.setChecked(enabled)
        self._twist_publish_toggle.blockSignals(False)

    @pyqtSlot(str)
    def _on_topic_changed(self, topic_name: str) -> None:
        self._committed_topic_name = topic_name
        self._twist_topic_combo.blockSignals(True)
        self._twist_topic_combo.setCurrentText(topic_name)
        self._twist_topic_combo.blockSignals(False)

    @pyqtSlot(float)
    def _on_rate_updated(self, value: float) -> None:
        rate = int(value)
        self._twist_rate_slider.blockSignals(True)
        self._twist_rate_slider.setValue(rate)
        self._twist_rate_slider.blockSignals(False)
        self._twist_rate_label.setText(f"{rate} Hz")

    @pyqtSlot()
    def _on_scales_updated(self) -> None:
        linear_scale, angular_scale = self._config_manager.get_twist_scales()
        self._twist_linear_spin.blockSignals(True)
        self._twist_linear_spin.setValue(linear_scale)
        self._twist_linear_spin.blockSignals(False)
        self._twist_angular_spin.blockSignals(True)
        self._twist_angular_spin.setValue(angular_scale)
        self._twist_angular_spin.blockSignals(False)

    @pyqtSlot(bool)
    def _on_holonomic_changed(self, enabled: bool) -> None:
        self._twist_holonomic_checkbox.blockSignals(True)
        self._twist_holonomic_checkbox.setChecked(enabled)
        self._twist_holonomic_checkbox.blockSignals(False)


class JoystickConfigPanel(_ControlPanel):
    """Controls for tuning joystick behaviour (dead zone, expo, return mode)."""

    def __init__(self, config_manager: ConfigurationManager, parent: Optional[QWidget] = None):
        super().__init__("Joystick Config", config_manager, parent)
        self._build_ui()
        self._connect_signals()
        self.refresh_from_config()

    def refresh_from_config(self) -> None:
        dead_zone = int(self._config_manager.get_dead_zone() * 100)
        dead_zone_x = int(self._config_manager.get_dead_zone_x() * 100)
        dead_zone_y = int(self._config_manager.get_dead_zone_y() * 100)
        expo_x = int(self._config_manager.get_expo_x())
        expo_y = int(self._config_manager.get_expo_y())
        return_mode = self._config_manager.get_return_mode()
        sticky_enabled = self._config_manager.is_sticky_buttons_enabled()

        self._set_slider_value(self._dead_zone_slider, self._dead_zone_label, dead_zone)
        self._set_slider_value(self._dead_zone_x_slider, self._dead_zone_x_label, dead_zone_x)
        self._set_slider_value(self._dead_zone_y_slider, self._dead_zone_y_label, dead_zone_y)
        self._set_slider_value(self._expo_x_slider, self._expo_x_label, expo_x, suffix=" %")
        self._set_slider_value(self._expo_y_slider, self._expo_y_label, expo_y, suffix=" %")

        index = max(0, self._return_mode_combo.findData(return_mode))
        self._return_mode_combo.blockSignals(True)
        self._return_mode_combo.setCurrentIndex(index)
        self._return_mode_combo.blockSignals(False)

        self._sticky_buttons_checkbox.blockSignals(True)
        self._sticky_buttons_checkbox.setChecked(sticky_enabled)
        self._sticky_buttons_checkbox.blockSignals(False)

    def _build_ui(self) -> None:
        layout = QGridLayout()
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        self._dead_zone_slider = self._create_slider(0, 90)
        self._dead_zone_label = self._value_label()
        self._add_slider_row(layout, row, "Dead Zone:", self._dead_zone_slider, self._dead_zone_label)

        row += 1
        self._dead_zone_x_slider = self._create_slider(0, 90)
        self._dead_zone_x_label = self._value_label()
        self._add_slider_row(layout, row, "Dead Zone X:", self._dead_zone_x_slider, self._dead_zone_x_label)

        row += 1
        self._dead_zone_y_slider = self._create_slider(0, 90)
        self._dead_zone_y_label = self._value_label()
        self._add_slider_row(layout, row, "Dead Zone Y:", self._dead_zone_y_slider, self._dead_zone_y_label)

        row += 1
        self._expo_x_slider = self._create_slider(0, 100)
        self._expo_x_label = self._value_label()
        self._add_slider_row(layout, row, "Expo X:", self._expo_x_slider, self._expo_x_label)

        row += 1
        self._expo_y_slider = self._create_slider(0, 100)
        self._expo_y_label = self._value_label()
        self._add_slider_row(layout, row, "Expo Y:", self._expo_y_slider, self._expo_y_label)

        row += 1
        layout.addWidget(self._label("Auto Return:"), row, 0)
        self._return_mode_combo = QComboBox()
        self._return_mode_combo.addItem("Both Axes", RETURN_MODE_BOTH)
        self._return_mode_combo.addItem("X Only", RETURN_MODE_HORIZONTAL)
        self._return_mode_combo.addItem("Y Only", RETURN_MODE_VERTICAL)
        self._return_mode_combo.addItem("Disabled", RETURN_MODE_NONE)
        layout.addWidget(self._return_mode_combo, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        row += 1
        layout.addWidget(self._label("Sticky Buttons:"), row, 0)
        self._sticky_buttons_checkbox = QCheckBox()
        layout.addWidget(self._sticky_buttons_checkbox, row, 1)
        layout.addWidget(self._placeholder(), row, 2)

        self.setLayout(layout)

    def _connect_signals(self) -> None:
        self._dead_zone_slider.valueChanged.connect(self._on_dead_zone_changed)
        self._dead_zone_x_slider.valueChanged.connect(self._on_dead_zone_x_changed)
        self._dead_zone_y_slider.valueChanged.connect(self._on_dead_zone_y_changed)
        self._expo_x_slider.valueChanged.connect(self._on_expo_x_changed)
        self._expo_y_slider.valueChanged.connect(self._on_expo_y_changed)
        self._return_mode_combo.currentIndexChanged.connect(self._on_return_mode_changed)
        self._sticky_buttons_checkbox.toggled.connect(self._config_manager.set_sticky_buttons)

        self._config_manager.dead_zone_changed.connect(self.refresh_from_config)
        self._config_manager.expo_changed.connect(self.refresh_from_config)
        self._config_manager.return_mode_changed.connect(self._on_return_mode_updated)
        self._config_manager.sticky_buttons_changed.connect(self._on_sticky_buttons_changed)

    def _create_slider(self, minimum: int, maximum: int) -> QSlider:
        slider = QSlider(Qt.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return slider

    def _add_slider_row(self, layout: QGridLayout, row: int, text: str, slider: QSlider, value_label: QLabel) -> None:
        layout.addWidget(self._label(text), row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(value_label, row, 2)

    def _set_slider_value(self, slider: QSlider, value_label: QLabel, value: int, suffix: str = " %") -> None:
        slider.blockSignals(True)
        slider.setValue(value)
        slider.blockSignals(False)
        value_label.setText(f"{value}{suffix}")

    @pyqtSlot(int)
    def _on_dead_zone_changed(self, value: int) -> None:
        try:
            self._config_manager.set_dead_zone(value / 100.0)
            self._dead_zone_label.setText(f"{value} %")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_dead_zone_x_changed(self, value: int) -> None:
        try:
            self._config_manager.set_dead_zone_x(value / 100.0)
            self._dead_zone_x_label.setText(f"{value} %")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_dead_zone_y_changed(self, value: int) -> None:
        try:
            self._config_manager.set_dead_zone_y(value / 100.0)
            self._dead_zone_y_label.setText(f"{value} %")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_expo_x_changed(self, value: int) -> None:
        try:
            self._config_manager.set_expo_x(float(value))
            self._expo_x_label.setText(f"{value} %")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_expo_y_changed(self, value: int) -> None:
        try:
            self._config_manager.set_expo_y(float(value))
            self._expo_y_label.setText(f"{value} %")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_return_mode_changed(self, index: int) -> None:
        mode = self._return_mode_combo.itemData(index)
        try:
            self._config_manager.set_return_mode(mode)
        except ValueError:
            self._on_return_mode_updated(self._config_manager.get_return_mode())

    @pyqtSlot(str)
    def _on_return_mode_updated(self, mode: str) -> None:
        index = max(0, self._return_mode_combo.findData(mode))
        self._return_mode_combo.blockSignals(True)
        self._return_mode_combo.setCurrentIndex(index)
        self._return_mode_combo.blockSignals(False)

    @pyqtSlot(bool)
    def _on_sticky_buttons_changed(self, enabled: bool) -> None:
        self._sticky_buttons_checkbox.blockSignals(True)
        self._sticky_buttons_checkbox.setChecked(enabled)
        self._sticky_buttons_checkbox.blockSignals(False)
