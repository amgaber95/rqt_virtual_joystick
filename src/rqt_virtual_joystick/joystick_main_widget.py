from python_qt_binding.QtCore import Qt, pyqtSlot, QEvent, QObject
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
    QDoubleSpinBox,
    QApplication,
)

from .joystick_widget import JoystickWidget
from .controller_buttons_widget import ControllerButtonsWidget
from .segmented_toggle import SegmentedToggle
from .config_manager import (
    ConfigurationManager,
    RETURN_MODE_BOTH,
    RETURN_MODE_HORIZONTAL,
    RETURN_MODE_NONE,
    RETURN_MODE_VERTICAL,
)
from .joy_publisher import JoyPublisherService
from .twist_publisher import TwistPublisherService


class HolonomicKeyHandler(QObject):
    """Temporary holonomic override driven by Shift key state."""

    def __init__(self, root_widget: QWidget, config_manager: ConfigurationManager):
        super().__init__(root_widget)
        self._root_widget = root_widget
        self._config_manager = config_manager
        self._shift_active = False
        self._restore_state = self._config_manager.is_twist_holonomic_enabled()

        app = QApplication.instance()
        self._application = app
        if self._application is not None:
            self._application.installEventFilter(self)

        self._root_widget.destroyed.connect(self._uninstall)
        self._config_manager.twist_holonomic_changed.connect(self._on_holonomic_changed)

    def eventFilter(self, obj, event):
        if not self._is_relevant_object(obj):
            return False

        if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
            if event.key() == Qt.Key_Shift:
                self._activate_shift_override()
        elif event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
            if event.key() == Qt.Key_Shift:
                self._deactivate_shift_override()

        return False

    def _is_relevant_object(self, obj) -> bool:
        if not isinstance(obj, QWidget):
            return False
        return obj is self._root_widget or self._root_widget.isAncestorOf(obj)

    def _activate_shift_override(self) -> None:
        if self._shift_active:
            return

        self._shift_active = True
        self._restore_state = self._config_manager.is_twist_holonomic_enabled()
        if not self._restore_state:
            self._config_manager.set_twist_holonomic(True)

    def _deactivate_shift_override(self) -> None:
        if not self._shift_active:
            return

        self._shift_active = False
        desired_state = self._restore_state
        if self._config_manager.is_twist_holonomic_enabled() != desired_state:
            self._config_manager.set_twist_holonomic(desired_state)

    def _on_holonomic_changed(self, enabled: bool) -> None:
        if not self._shift_active:
            self._restore_state = enabled

    def _uninstall(self) -> None:
        if self._application is not None:
            self._application.removeEventFilter(self)
            self._application = None


class JoystickMainWidget(QWidget):
    """Main widget that wires the joystick visualizer with configuration controls."""

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)

        self._ros_node = ros_node
        self._config_manager = ConfigurationManager()
        self._publisher_service = JoyPublisherService(ros_node, self._config_manager)
        self._twist_publisher_service = TwistPublisherService(ros_node, self._config_manager)
        self._committed_topic_name = self._config_manager.get_topic_name()
        self._committed_twist_topic_name = self._config_manager.get_twist_topic()

        self._label_min_width = 90
        self._value_placeholder_width = 40

        self._twist_publisher_service.update_from_axes(0.0, 0.0)

        self.setMaximumWidth(700)  # Increased to accommodate joystick + buttons

        self._init_ui()
        self._connect_signals()
        self.setFocusPolicy(Qt.StrongFocus)
        self._holonomic_key_handler = HolonomicKeyHandler(self, self._config_manager)

    def _init_ui(self):
        main_layout = QVBoxLayout()

        # Create horizontal layout for joystick and controller buttons
        joystick_layout = QHBoxLayout()
        
        self._joystick_widget = JoystickWidget(self._config_manager)
        joystick_layout.addWidget(self._joystick_widget, 1)
        
        # Add controller buttons widget
        self._controller_buttons_widget = ControllerButtonsWidget(
            sticky_buttons=self._config_manager.is_sticky_buttons_enabled()
        )
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
        layout.addWidget(self._create_publishing_group())
        layout.addWidget(self._create_twist_group())
        layout.addWidget(self._create_joystick_group())
        layout.addStretch(1)
        controls_widget.setLayout(layout)
        controls_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        return controls_widget

    def _create_publishing_group(self) -> QGroupBox:
        group = QGroupBox("Joy Output")
        layout = QGridLayout()
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        self._publish_label = QLabel("Publish:")
        self._publish_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(self._publish_label, row, 0)
        self._publish_toggle = SegmentedToggle(false_label="Disabled", true_label="Enabled")
        self._publish_toggle.setChecked(self._config_manager.is_publish_enabled())
        layout.addWidget(self._publish_toggle, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        row += 1
        topic_label = QLabel("Topic:")
        topic_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(topic_label, row, 0)
        self._topic_combo = QComboBox()
        self._topic_combo.setEditable(True)
        self._topic_combo.addItems(["joy", "teleop/joy", "input/joy"])
        self._topic_combo.setCurrentText(self._config_manager.get_topic_name())
        self._topic_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._topic_combo, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        row += 1
        rate_label = QLabel("Rate:")
        rate_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(rate_label, row, 0)
        self._rate_slider = QSlider(Qt.Horizontal)
        self._rate_slider.setRange(1, 100)
        self._rate_slider.setValue(int(self._config_manager.get_publish_rate()))
        self._rate_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._rate_slider, row, 1)

        self._rate_label = QLabel(f"{int(self._config_manager.get_publish_rate())} Hz")
        self._rate_label.setFixedWidth(self._value_placeholder_width)
        self._rate_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._rate_label, row, 2)

        group.setLayout(layout)
        self._apply_groupbox_style(group)
        return group

    def _create_twist_group(self) -> QGroupBox:
        group = QGroupBox("Twist Output")
        layout = QGridLayout()
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        twist_publish_label = QLabel("Publish:")
        twist_publish_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(twist_publish_label, row, 0)
        self._twist_publish_toggle = SegmentedToggle(false_label="Disabled", true_label="Enabled")
        self._twist_publish_toggle.setChecked(self._config_manager.is_twist_publish_enabled())
        layout.addWidget(self._twist_publish_toggle, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        row += 1
        twist_topic_label = QLabel("Topic:")
        twist_topic_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(twist_topic_label, row, 0)
        self._twist_topic_combo = QComboBox()
        self._twist_topic_combo.setEditable(True)
        self._twist_topic_combo.addItems(["/cmd_vel", "cmd_vel", "robot/cmd_vel"])
        self._twist_topic_combo.setCurrentText(self._config_manager.get_twist_topic())
        self._twist_topic_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._twist_topic_combo, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        row += 1
        twist_rate_label = QLabel("Rate:")
        twist_rate_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(twist_rate_label, row, 0)
        self._twist_rate_slider = QSlider(Qt.Horizontal)
        self._twist_rate_slider.setRange(1, 100)
        self._twist_rate_slider.setValue(int(self._config_manager.get_twist_publish_rate()))
        self._twist_rate_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._twist_rate_slider, row, 1)

        self._twist_rate_label = QLabel(f"{int(self._config_manager.get_twist_publish_rate())} Hz")
        self._twist_rate_label.setFixedWidth(self._value_placeholder_width)
        self._twist_rate_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._twist_rate_label, row, 2)

        row += 1
        linear_label = QLabel("Linear Scale:")
        linear_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(linear_label, row, 0)
        self._twist_linear_spin = QDoubleSpinBox()
        self._twist_linear_spin.setDecimals(2)
        self._twist_linear_spin.setSingleStep(0.1)
        self._twist_linear_spin.setRange(0.0, 5.0)
        self._twist_linear_spin.setValue(self._config_manager.get_twist_scales()[0])
        layout.addWidget(self._twist_linear_spin, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        row += 1
        angular_label = QLabel("Angular Scale:")
        angular_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(angular_label, row, 0)
        self._twist_angular_spin = QDoubleSpinBox()
        self._twist_angular_spin.setDecimals(2)
        self._twist_angular_spin.setSingleStep(0.1)
        self._twist_angular_spin.setRange(0.0, 5.0)
        self._twist_angular_spin.setValue(self._config_manager.get_twist_scales()[1])
        layout.addWidget(self._twist_angular_spin, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        row += 1
        holonomic_label = QLabel("Holonomic:")
        holonomic_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(holonomic_label, row, 0)
        self._twist_holonomic_checkbox = QCheckBox()
        self._twist_holonomic_checkbox.setChecked(self._config_manager.is_twist_holonomic_enabled())
        layout.addWidget(self._twist_holonomic_checkbox, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        group.setLayout(layout)
        self._apply_groupbox_style(group)
        return group

    def _create_joystick_group(self) -> QGroupBox:
        group = QGroupBox("Joystick Config")
        layout = QGridLayout()
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)

        row = 0
        dead_zone_label = QLabel("Dead Zone:")
        dead_zone_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(dead_zone_label, row, 0)
        self._dead_zone_slider = QSlider(Qt.Horizontal)
        self._dead_zone_slider.setRange(0, 90)
        self._dead_zone_slider.setValue(int(self._config_manager.get_dead_zone() * 100))
        self._dead_zone_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._dead_zone_slider, row, 1)

        self._dead_zone_label = QLabel(f"{int(self._config_manager.get_dead_zone() * 100)}%")
        self._dead_zone_label.setFixedWidth(self._value_placeholder_width)
        self._dead_zone_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._dead_zone_label, row, 2)

        row += 1
        dead_zone_x_label = QLabel("Dead Zone X:")
        dead_zone_x_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(dead_zone_x_label, row, 0)
        self._dead_zone_x_slider = QSlider(Qt.Horizontal)
        self._dead_zone_x_slider.setRange(0, 90)
        self._dead_zone_x_slider.setValue(int(self._config_manager.get_dead_zone_x() * 100))
        self._dead_zone_x_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._dead_zone_x_slider, row, 1)

        self._dead_zone_x_label = QLabel(f"{int(self._config_manager.get_dead_zone_x() * 100)}%")
        self._dead_zone_x_label.setFixedWidth(self._value_placeholder_width)
        self._dead_zone_x_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._dead_zone_x_label, row, 2)

        row += 1
        dead_zone_y_label = QLabel("Dead Zone Y:")
        dead_zone_y_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(dead_zone_y_label, row, 0)
        self._dead_zone_y_slider = QSlider(Qt.Horizontal)
        self._dead_zone_y_slider.setRange(0, 90)
        self._dead_zone_y_slider.setValue(int(self._config_manager.get_dead_zone_y() * 100))
        self._dead_zone_y_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._dead_zone_y_slider, row, 1)

        self._dead_zone_y_label = QLabel(f"{int(self._config_manager.get_dead_zone_y() * 100)}%")
        self._dead_zone_y_label.setFixedWidth(self._value_placeholder_width)
        self._dead_zone_y_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._dead_zone_y_label, row, 2)

        row += 1
        expo_x_label = QLabel("Expo X:")
        expo_x_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(expo_x_label, row, 0)
        self._expo_x_slider = QSlider(Qt.Horizontal)
        self._expo_x_slider.setRange(0, 100)
        self._expo_x_slider.setValue(int(self._config_manager.get_expo_x()))
        self._expo_x_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._expo_x_slider, row, 1)

        self._expo_x_label = QLabel(f"{int(self._config_manager.get_expo_x())}%")
        self._expo_x_label.setFixedWidth(self._value_placeholder_width)
        self._expo_x_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._expo_x_label, row, 2)

        row += 1
        expo_y_label = QLabel("Expo Y:")
        expo_y_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(expo_y_label, row, 0)
        self._expo_y_slider = QSlider(Qt.Horizontal)
        self._expo_y_slider.setRange(0, 100)
        self._expo_y_slider.setValue(int(self._config_manager.get_expo_y()))
        self._expo_y_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._expo_y_slider, row, 1)

        self._expo_y_label = QLabel(f"{int(self._config_manager.get_expo_y())}%")
        self._expo_y_label.setFixedWidth(self._value_placeholder_width)
        self._expo_y_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._expo_y_label, row, 2)

        row += 1
        auto_return_label = QLabel("Auto Return:")
        auto_return_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(auto_return_label, row, 0)
        self._return_mode_combo = QComboBox()
        self._return_mode_combo.addItem("Both Axes", RETURN_MODE_BOTH)
        self._return_mode_combo.addItem("X Only", RETURN_MODE_HORIZONTAL)
        self._return_mode_combo.addItem("Y Only", RETURN_MODE_VERTICAL)
        self._return_mode_combo.addItem("Disabled", RETURN_MODE_NONE)
        current_mode = self._config_manager.get_return_mode()
        index = max(0, self._return_mode_combo.findData(current_mode))
        self._return_mode_combo.setCurrentIndex(index)
        layout.addWidget(self._return_mode_combo, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        row += 1
        self._sticky_buttons_label = QLabel("Sticky Buttons:")
        self._sticky_buttons_label.setMinimumWidth(self._label_min_width)
        layout.addWidget(self._sticky_buttons_label, row, 0)
        self._sticky_buttons_checkbox = QCheckBox()
        self._sticky_buttons_checkbox.setChecked(self._config_manager.is_sticky_buttons_enabled())
        layout.addWidget(self._sticky_buttons_checkbox, row, 1)
        layout.addWidget(self._build_placeholder(), row, 2)

        group.setLayout(layout)
        self._apply_groupbox_style(group)
        return group

    @staticmethod
    def _apply_groupbox_style(group: QGroupBox) -> None:
        group.setStyleSheet("""
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
        """)

    def _build_placeholder(self) -> QLabel:
        placeholder = QLabel("")
        placeholder.setFixedWidth(self._value_placeholder_width)
        return placeholder

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
        self._publish_toggle.toggled.connect(self._on_publish_toggled)
        self._return_mode_combo.currentIndexChanged.connect(self._on_return_mode_changed)
        self._sticky_buttons_checkbox.toggled.connect(self._on_sticky_buttons_toggled)
        self._twist_publish_toggle.toggled.connect(self._on_twist_publish_toggled)

        self._twist_topic_combo.activated[str].connect(self._on_twist_topic_activated)
        twist_topic_line_edit = self._twist_topic_combo.lineEdit()
        if twist_topic_line_edit:
            twist_topic_line_edit.returnPressed.connect(self._on_twist_topic_return_pressed)

        self._twist_rate_slider.valueChanged.connect(self._on_twist_rate_changed)
        self._twist_linear_spin.valueChanged.connect(self._on_twist_scale_changed)
        self._twist_angular_spin.valueChanged.connect(self._on_twist_scale_changed)
        self._twist_holonomic_checkbox.toggled.connect(self._on_twist_holonomic_toggled)

        self._config_manager.dead_zone_changed.connect(self._update_control_values)
        self._config_manager.expo_changed.connect(self._update_control_values)
        self._config_manager.publish_enabled_changed.connect(self._on_publish_enabled_changed)
        self._config_manager.return_mode_changed.connect(self._on_return_mode_updated)
        self._config_manager.sticky_buttons_changed.connect(self._on_sticky_buttons_changed)
        self._config_manager.twist_publish_enabled_changed.connect(self._on_twist_publish_enabled_changed)
        self._config_manager.twist_topic_changed.connect(self._on_twist_topic_changed)
        self._config_manager.twist_rate_changed.connect(self._on_twist_rate_updated)
        self._config_manager.twist_scales_changed.connect(self._on_twist_scales_updated)
        self._config_manager.twist_holonomic_changed.connect(self._on_twist_holonomic_changed)

        self._joystick_widget.position_changed.connect(self._on_joystick_moved)
        self._controller_buttons_widget.button_pressed.connect(self._on_button_pressed)
        self._publisher_service.publisher_error.connect(self._on_publisher_error)
        self._twist_publisher_service.publisher_error.connect(self._on_publisher_error)

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
        self._twist_publisher_service.update_from_axes(x, y)

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

        twist_topic = self._config_manager.get_twist_topic()
        self._committed_twist_topic_name = twist_topic
        self._twist_topic_combo.setCurrentText(twist_topic)

        twist_rate = int(self._config_manager.get_twist_publish_rate())
        self._twist_rate_slider.setValue(twist_rate)
        self._twist_rate_label.setText(f"{twist_rate} Hz")
        
        self._update_control_values()

    def shutdown(self):
        self._publisher_service.shutdown()
        self._twist_publisher_service.shutdown()
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
        sticky_enabled = self._config_manager.is_sticky_buttons_enabled()
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

        self._publish_toggle.blockSignals(True)
        self._publish_toggle.setChecked(publish_enabled)
        self._publish_toggle.blockSignals(False)

        twist_enabled = self._config_manager.is_twist_publish_enabled()
        self._twist_publish_toggle.blockSignals(True)
        self._twist_publish_toggle.setChecked(twist_enabled)
        self._twist_publish_toggle.blockSignals(False)

        twist_topic = self._config_manager.get_twist_topic()
        self._twist_topic_combo.blockSignals(True)
        self._twist_topic_combo.setCurrentText(twist_topic)
        self._twist_topic_combo.blockSignals(False)

        twist_rate = int(self._config_manager.get_twist_publish_rate())
        self._twist_rate_slider.blockSignals(True)
        self._twist_rate_slider.setValue(twist_rate)
        self._twist_rate_slider.blockSignals(False)
        self._twist_rate_label.setText(f"{twist_rate} Hz")

        linear_scale, angular_scale = self._config_manager.get_twist_scales()
        self._twist_linear_spin.blockSignals(True)
        self._twist_linear_spin.setValue(linear_scale)
        self._twist_linear_spin.blockSignals(False)
        self._twist_angular_spin.blockSignals(True)
        self._twist_angular_spin.setValue(angular_scale)
        self._twist_angular_spin.blockSignals(False)

        holonomic_enabled = self._config_manager.is_twist_holonomic_enabled()
        self._twist_holonomic_checkbox.blockSignals(True)
        self._twist_holonomic_checkbox.setChecked(holonomic_enabled)
        self._twist_holonomic_checkbox.blockSignals(False)

        self._sticky_buttons_checkbox.blockSignals(True)
        self._sticky_buttons_checkbox.setChecked(sticky_enabled)
        self._sticky_buttons_checkbox.blockSignals(False)
        self._controller_buttons_widget.set_sticky_buttons(sticky_enabled)

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
        self._publish_toggle.blockSignals(True)
        self._publish_toggle.setChecked(enabled)
        self._publish_toggle.blockSignals(False)

    @pyqtSlot(bool)
    def _on_twist_publish_toggled(self, enabled: bool):
        self._config_manager.set_twist_publish_enabled(enabled)

    @pyqtSlot(bool)
    def _on_sticky_buttons_toggled(self, enabled: bool):
        self._config_manager.set_sticky_buttons(enabled)

    @pyqtSlot(bool)
    def _on_sticky_buttons_changed(self, enabled: bool):
        self._sticky_buttons_checkbox.blockSignals(True)
        self._sticky_buttons_checkbox.setChecked(enabled)
        self._sticky_buttons_checkbox.blockSignals(False)
        self._controller_buttons_widget.set_sticky_buttons(enabled)

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

    def _commit_twist_topic(self, topic_name: str) -> None:
        try:
            self._config_manager.set_twist_topic(topic_name)
            self._committed_twist_topic_name = topic_name
        except ValueError:
            self._twist_topic_combo.blockSignals(True)
            self._twist_topic_combo.setCurrentText(self._committed_twist_topic_name)
            self._twist_topic_combo.blockSignals(False)

    @pyqtSlot(str)
    def _on_twist_topic_activated(self, topic_name: str) -> None:
        self._commit_twist_topic(topic_name)

    @pyqtSlot()
    def _on_twist_topic_return_pressed(self) -> None:
        self._commit_twist_topic(self._twist_topic_combo.currentText())

    @pyqtSlot(int)
    def _on_twist_rate_changed(self, value: int):
        try:
            self._config_manager.set_twist_publish_rate(float(value))
            self._twist_rate_label.setText(f"{value} Hz")
        except ValueError:
            pass

    @pyqtSlot(float)
    def _on_twist_rate_updated(self, value: float):
        self._twist_rate_slider.blockSignals(True)
        self._twist_rate_slider.setValue(int(value))
        self._twist_rate_slider.blockSignals(False)
        self._twist_rate_label.setText(f"{int(value)} Hz")

    @pyqtSlot(float)
    def _on_twist_scale_changed(self, _value: float):
        try:
            self._config_manager.set_twist_scales(
                self._twist_linear_spin.value(),
                self._twist_angular_spin.value(),
            )
        except ValueError:
            pass

    @pyqtSlot()
    def _on_twist_scales_updated(self):
        linear_scale, angular_scale = self._config_manager.get_twist_scales()
        self._twist_linear_spin.blockSignals(True)
        self._twist_linear_spin.setValue(linear_scale)
        self._twist_linear_spin.blockSignals(False)
        self._twist_angular_spin.blockSignals(True)
        self._twist_angular_spin.setValue(angular_scale)
        self._twist_angular_spin.blockSignals(False)

    @pyqtSlot(bool)
    def _on_twist_publish_enabled_changed(self, enabled: bool):
        self._twist_publish_toggle.blockSignals(True)
        self._twist_publish_toggle.setChecked(enabled)
        self._twist_publish_toggle.blockSignals(False)

    @pyqtSlot(bool)
    def _on_twist_holonomic_toggled(self, enabled: bool):
        self._config_manager.set_twist_holonomic(enabled)

    @pyqtSlot(bool)
    def _on_twist_holonomic_changed(self, enabled: bool):
        self._twist_holonomic_checkbox.blockSignals(True)
        self._twist_holonomic_checkbox.setChecked(enabled)
        self._twist_holonomic_checkbox.blockSignals(False)

    @pyqtSlot(str)
    def _on_twist_topic_changed(self, topic_name: str):
        self._committed_twist_topic_name = topic_name
        self._twist_topic_combo.blockSignals(True)
        self._twist_topic_combo.setCurrentText(topic_name)
        self._twist_topic_combo.blockSignals(False)
