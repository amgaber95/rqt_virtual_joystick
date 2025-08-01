from python_qt_binding.QtCore import Qt, pyqtSlot
from python_qt_binding.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QComboBox,
    QCheckBox,
    QGroupBox,
    QGridLayout,
    QSizePolicy,
)

from .joystick_widget import JoystickWidget
from .controller_buttons_widget import ControllerButtonsWidget
from .config_manager import (
    ConfigurationManager,
    RETURN_MODE_BOTH,
    RETURN_MODE_HORIZONTAL,
    RETURN_MODE_NONE,
    RETURN_MODE_VERTICAL,
)
from .joy_publisher import JoyPublisherService


class JoystickMainWidget(QWidget):
    """Main widget that wires the joystick visualizer with configuration controls."""

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)

        self._ros_node = ros_node
        self._config_manager = ConfigurationManager()
        self._publisher_service = JoyPublisherService(ros_node, self._config_manager)
        self._committed_topic_name = self._config_manager.get_topic_name()

        self.setMaximumWidth(700)  # Increased to accommodate joystick + buttons

        self._init_ui()
        self._connect_signals()

    def _init_ui(self):
        main_layout = QVBoxLayout()

        # Create horizontal layout for joystick and controller buttons
        joystick_layout = QHBoxLayout()
        
        self._joystick_widget = JoystickWidget(self._config_manager)
        joystick_layout.addWidget(self._joystick_widget, 1)
        
        # Add controller buttons widget
        self._controller_buttons_widget = ControllerButtonsWidget()
        joystick_layout.addWidget(self._controller_buttons_widget, 0)
        
        # Add some spacing between joystick and buttons
        # joystick_layout.setSpacing(0)
        
        main_layout.addLayout(joystick_layout, 1)

        controls_widget = self._create_controls()
        main_layout.addWidget(controls_widget, 0)
        main_layout.addStretch(1)

        self.setLayout(main_layout)

    def _create_controls(self) -> QWidget:
        controls_widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(8)
        layout.addWidget(self._create_compact_controls())
        layout.addStretch(1)
        controls_widget.setLayout(layout)
        controls_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        return controls_widget

    def _create_compact_controls(self) -> QGroupBox:
        group = QGroupBox("Controls")
        group.setSizePolicy(group.sizePolicy().horizontalPolicy(), group.sizePolicy().Minimum)
        layout = QGridLayout()
        layout.setVerticalSpacing(8)

        row = 0
        layout.addWidget(QLabel("Topic:"), row, 0)
        self._topic_combo = QComboBox()
        self._topic_combo.setEditable(True)
        self._topic_combo.addItems(["joy", "teleop/joy", "input/joy"])
        self._topic_combo.setCurrentText(self._config_manager.get_topic_name())
        layout.addWidget(self._topic_combo, row, 1, 1, 3)

        self._publish_checkbox = QCheckBox("Publish")
        self._publish_checkbox.setChecked(self._config_manager.is_publish_enabled())
        layout.addWidget(self._publish_checkbox, row, 4)

        row += 1
        layout.addWidget(QLabel("Rate:"), row, 0)
        self._rate_slider = QSlider(Qt.Horizontal)
        self._rate_slider.setRange(1, 100)
        self._rate_slider.setValue(int(self._config_manager.get_publish_rate()))
        layout.addWidget(self._rate_slider, row, 1, 1, 3)

        self._rate_label = QLabel(f"{int(self._config_manager.get_publish_rate())} Hz")
        self._rate_label.setMinimumWidth(30)
        layout.addWidget(self._rate_label, row, 4)
        
        # Add dead zone control
        row += 1
        layout.addWidget(QLabel("Dead Zone:"), row, 0)
        self._dead_zone_slider = QSlider(Qt.Horizontal)
        self._dead_zone_slider.setRange(0, 90)  # 0-90% of full range
        self._dead_zone_slider.setValue(int(self._config_manager.get_dead_zone() * 100))
        layout.addWidget(self._dead_zone_slider, row, 1, 1, 3)

        self._dead_zone_label = QLabel(f"{int(self._config_manager.get_dead_zone() * 100)}%")
        self._dead_zone_label.setMinimumWidth(30)
        layout.addWidget(self._dead_zone_label, row, 4)

        # Add X dead zone control
        row += 1
        layout.addWidget(QLabel("Dead Zone X:"), row, 0)
        self._dead_zone_x_slider = QSlider(Qt.Horizontal)
        self._dead_zone_x_slider.setRange(0, 90)
        self._dead_zone_x_slider.setValue(int(self._config_manager.get_dead_zone_x() * 100))
        layout.addWidget(self._dead_zone_x_slider, row, 1, 1, 3)

        self._dead_zone_x_label = QLabel(f"{int(self._config_manager.get_dead_zone_x() * 100)}%")
        self._dead_zone_x_label.setMinimumWidth(30)
        layout.addWidget(self._dead_zone_x_label, row, 4)

        # Add Y dead zone control
        row += 1
        layout.addWidget(QLabel("Dead Zone Y:"), row, 0)
        self._dead_zone_y_slider = QSlider(Qt.Horizontal)
        self._dead_zone_y_slider.setRange(0, 90)
        self._dead_zone_y_slider.setValue(int(self._config_manager.get_dead_zone_y() * 100))
        layout.addWidget(self._dead_zone_y_slider, row, 1, 1, 3)

        self._dead_zone_y_label = QLabel(f"{int(self._config_manager.get_dead_zone_y() * 100)}%")
        self._dead_zone_y_label.setMinimumWidth(30)
        layout.addWidget(self._dead_zone_y_label, row, 4)

        # Add Expo X control
        row += 1
        layout.addWidget(QLabel("Expo X:"), row, 0)
        self._expo_x_slider = QSlider(Qt.Horizontal)
        self._expo_x_slider.setRange(0, 100)
        self._expo_x_slider.setValue(int(self._config_manager.get_expo_x()))
        layout.addWidget(self._expo_x_slider, row, 1, 1, 3)

        self._expo_x_label = QLabel(f"{int(self._config_manager.get_expo_x())}%")
        self._expo_x_label.setMinimumWidth(30)
        layout.addWidget(self._expo_x_label, row, 4)

        # Add Expo Y control
        row += 1
        layout.addWidget(QLabel("Expo Y:"), row, 0)
        self._expo_y_slider = QSlider(Qt.Horizontal)
        self._expo_y_slider.setRange(0, 100)
        self._expo_y_slider.setValue(int(self._config_manager.get_expo_y()))
        layout.addWidget(self._expo_y_slider, row, 1, 1, 3)

        self._expo_y_label = QLabel(f"{int(self._config_manager.get_expo_y())}%")
        self._expo_y_label.setMinimumWidth(30)
        layout.addWidget(self._expo_y_label, row, 4)

        # Add Auto Return control
        row += 1
        layout.addWidget(QLabel("Auto Return:"), row, 0)
        self._return_mode_combo = QComboBox()
        self._return_mode_combo.addItem("Both Axes", RETURN_MODE_BOTH)
        self._return_mode_combo.addItem("X Only", RETURN_MODE_HORIZONTAL)
        self._return_mode_combo.addItem("Y Only", RETURN_MODE_VERTICAL)
        self._return_mode_combo.addItem("Disabled", RETURN_MODE_NONE)
        current_mode = self._config_manager.get_return_mode()
        index = max(0, self._return_mode_combo.findData(current_mode))
        self._return_mode_combo.setCurrentIndex(index)
        layout.addWidget(self._return_mode_combo, row, 1, 1, 3)

        self._return_mode_label = QLabel(self._return_mode_combo.currentText())
        self._return_mode_label.setMinimumWidth(80)
        layout.addWidget(self._return_mode_label, row, 4)

        group.setLayout(layout)
        return group

    def _connect_signals(self):
        self._topic_combo.activated[str].connect(self._on_topic_activated)

        topic_line_edit = self._topic_combo.lineEdit()
        if topic_line_edit:
            topic_line_edit.returnPressed.connect(self._on_topic_return_pressed)

        self._rate_slider.valueChanged.connect(self._on_rate_changed)
        self._dead_zone_slider.valueChanged.connect(self._on_dead_zone_changed)
        self._dead_zone_x_slider.valueChanged.connect(self._on_dead_zone_x_changed)
        self._dead_zone_y_slider.valueChanged.connect(self._on_dead_zone_y_changed)
        self._expo_x_slider.valueChanged.connect(self._on_expo_x_changed)
        self._expo_y_slider.valueChanged.connect(self._on_expo_y_changed)
        self._publish_checkbox.toggled.connect(self._on_publish_toggled)
        self._return_mode_combo.currentIndexChanged.connect(self._on_return_mode_changed)

        self._config_manager.dead_zone_changed.connect(self._update_control_values)
        self._config_manager.expo_changed.connect(self._update_control_values)
        self._config_manager.publish_enabled_changed.connect(self._on_publish_enabled_changed)
        self._config_manager.return_mode_changed.connect(self._on_return_mode_updated)

        self._joystick_widget.position_changed.connect(self._on_joystick_moved)
        self._controller_buttons_widget.button_pressed.connect(self._on_button_pressed)
        self._publisher_service.publisher_error.connect(self._on_publisher_error)

    @pyqtSlot(str)
    def _on_topic_activated(self, topic_name: str) -> None:
        self._commit_topic_name(topic_name)

    @pyqtSlot()
    def _on_topic_return_pressed(self) -> None:
        self._commit_topic_name(self._topic_combo.currentText())

    def _commit_topic_name(self, topic_name: str) -> None:
        try:
            self._config_manager.set_topic_name(topic_name)
            self._committed_topic_name = topic_name
        except ValueError:
            self._restore_committed_topic()

    def _restore_committed_topic(self) -> None:
        self._topic_combo.setCurrentText(self._committed_topic_name)

    @pyqtSlot(int)
    def _on_rate_changed(self, value: int):
        try:
            self._config_manager.set_publish_rate(float(value))
            self._rate_label.setText(f"{value} Hz")
        except ValueError:
            pass
            
    @pyqtSlot(int)
    def _on_dead_zone_changed(self, value: int):
        try:
            dead_zone_value = value / 100.0  # Convert percentage to 0.0-0.9 range
            self._config_manager.set_dead_zone(dead_zone_value)
            self._dead_zone_label.setText(f"{value}%")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_dead_zone_x_changed(self, value: int):
        try:
            dead_zone_value = value / 100.0
            self._config_manager.set_dead_zone_x(dead_zone_value)
            self._dead_zone_x_label.setText(f"{value}%")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_dead_zone_y_changed(self, value: int):
        try:
            dead_zone_value = value / 100.0
            self._config_manager.set_dead_zone_y(dead_zone_value)
            self._dead_zone_y_label.setText(f"{value}%")
        except ValueError:
            pass

    @pyqtSlot(float, float)
    def _on_joystick_moved(self, x: float, y: float):
        self._publisher_service.update_axes(x, y)

    @pyqtSlot(int, bool)
    def _on_button_pressed(self, button_index: int, pressed: bool):
        """Handle Xbox button press/release events."""
        self._publisher_service.update_button(button_index, pressed)

    @pyqtSlot(str)
    def _on_publisher_error(self, error_msg: str):
        self._ros_node.get_logger().error(f"Publisher error: {error_msg}")

    def save_settings(self, settings):
        self._config_manager.save_settings(settings)

    def restore_settings(self, settings):
        self._config_manager.restore_settings(settings)
        self._update_ui_from_config()

    def _update_ui_from_config(self):
        committed_topic = self._config_manager.get_topic_name()
        self._committed_topic_name = committed_topic
        self._topic_combo.setCurrentText(committed_topic)
        
        publish_rate = int(self._config_manager.get_publish_rate())
        self._rate_slider.setValue(publish_rate)
        self._rate_label.setText(f"{publish_rate} Hz")
        
        self._update_control_values()

    def shutdown(self):
        self._publisher_service.shutdown()
        self._joystick_widget.reset_position()
        self._controller_buttons_widget.reset_buttons()

    @pyqtSlot()
    def _update_control_values(self):
        dead_zone_value = int(self._config_manager.get_dead_zone() * 100)
        dead_zone_x_value = int(self._config_manager.get_dead_zone_x() * 100)
        dead_zone_y_value = int(self._config_manager.get_dead_zone_y() * 100)
        expo_x_value = int(self._config_manager.get_expo_x())
        expo_y_value = int(self._config_manager.get_expo_y())
        return_mode = self._config_manager.get_return_mode()
        publish_enabled = self._config_manager.is_publish_enabled()

        self._dead_zone_slider.blockSignals(True)
        self._dead_zone_slider.setValue(dead_zone_value)
        self._dead_zone_slider.blockSignals(False)
        self._dead_zone_label.setText(f"{dead_zone_value}%")

        self._dead_zone_x_slider.blockSignals(True)
        self._dead_zone_x_slider.setValue(dead_zone_x_value)
        self._dead_zone_x_slider.blockSignals(False)
        self._dead_zone_x_label.setText(f"{dead_zone_x_value}%")

        self._dead_zone_y_slider.blockSignals(True)
        self._dead_zone_y_slider.setValue(dead_zone_y_value)
        self._dead_zone_y_slider.blockSignals(False)
        self._dead_zone_y_label.setText(f"{dead_zone_y_value}%")

        self._expo_x_slider.blockSignals(True)
        self._expo_x_slider.setValue(expo_x_value)
        self._expo_x_slider.blockSignals(False)
        self._expo_x_label.setText(f"{expo_x_value}%")

        self._expo_y_slider.blockSignals(True)
        self._expo_y_slider.setValue(expo_y_value)
        self._expo_y_slider.blockSignals(False)
        self._expo_y_label.setText(f"{expo_y_value}%")

        self._return_mode_combo.blockSignals(True)
        index = max(0, self._return_mode_combo.findData(return_mode))
        self._return_mode_combo.setCurrentIndex(index)
        self._return_mode_combo.blockSignals(False)
        self._return_mode_label.setText(self._return_mode_combo.currentText())

        self._publish_checkbox.blockSignals(True)
        self._publish_checkbox.setChecked(publish_enabled)
        self._publish_checkbox.blockSignals(False)

    @pyqtSlot(int)
    def _on_expo_x_changed(self, value: int):
        try:
            self._config_manager.set_expo_x(float(value))
            self._expo_x_label.setText(f"{value}%")
        except ValueError:
            pass

    @pyqtSlot(int)
    def _on_expo_y_changed(self, value: int):
        try:
            self._config_manager.set_expo_y(float(value))
            self._expo_y_label.setText(f"{value}%")
        except ValueError:
            pass

    @pyqtSlot(bool)
    def _on_publish_toggled(self, enabled: bool):
        self._config_manager.set_publish_enabled(enabled)

    @pyqtSlot(bool)
    def _on_publish_enabled_changed(self, enabled: bool):
        self._publish_checkbox.blockSignals(True)
        self._publish_checkbox.setChecked(enabled)
        self._publish_checkbox.blockSignals(False)

    @pyqtSlot(int)
    def _on_return_mode_changed(self, index: int):
        mode = self._return_mode_combo.itemData(index)
        try:
            self._config_manager.set_return_mode(mode)
        except ValueError:
            self._on_return_mode_updated(self._config_manager.get_return_mode())

    @pyqtSlot(str)
    def _on_return_mode_updated(self, mode: str):
        index = max(0, self._return_mode_combo.findData(mode))
        self._return_mode_combo.blockSignals(True)
        self._return_mode_combo.setCurrentIndex(index)
        self._return_mode_combo.blockSignals(False)
        self._return_mode_label.setText(self._return_mode_combo.currentText())
