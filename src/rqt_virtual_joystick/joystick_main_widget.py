from python_qt_binding.QtCore import Qt, pyqtSlot
from python_qt_binding.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QSlider,
    QComboBox,
    QGroupBox,
    QGridLayout,
)

from .joystick_widget import JoystickWidget
from .config_manager import ConfigurationManager
from .joy_publisher import JoyPublisherService


class JoystickMainWidget(QWidget):
    """Main widget that wires the joystick visualizer with configuration controls."""

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)

        self._ros_node = ros_node
        self._config_manager = ConfigurationManager()
        self._publisher_service = JoyPublisherService(ros_node, self._config_manager)
        self._committed_topic_name = self._config_manager.get_topic_name()

        self.setMaximumWidth(400)

        self._init_ui()
        self._connect_signals()

    def _init_ui(self):
        main_layout = QVBoxLayout()

        self._joystick_widget = JoystickWidget(self._config_manager)
        main_layout.addWidget(self._joystick_widget, 1)

        controls_widget = self._create_controls()
        main_layout.addWidget(controls_widget)

        self.setLayout(main_layout)

    def _create_controls(self) -> QWidget:
        controls_widget = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(8)
        layout.addWidget(self._create_compact_controls())
        controls_widget.setLayout(layout)
        return controls_widget

    def _create_compact_controls(self) -> QGroupBox:
        group = QGroupBox("Controls")
        layout = QGridLayout()
        layout.setVerticalSpacing(4)

        row = 0
        layout.addWidget(QLabel("Topic:"), row, 0)
        self._topic_combo = QComboBox()
        self._topic_combo.setEditable(True)
        self._topic_combo.addItems(["joy", "teleop/joy", "input/joy"])
        self._topic_combo.setCurrentText(self._config_manager.get_topic_name())
        layout.addWidget(self._topic_combo, row, 1, 1, 4)

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

        self._config_manager.dead_zone_changed.connect(self._update_dead_zone_controls)

        self._joystick_widget.position_changed.connect(self._on_joystick_moved)
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

    @pyqtSlot(float, float)
    def _on_joystick_moved(self, x: float, y: float):
        self._publisher_service.update_axes(x, y)

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
        
        self._update_dead_zone_controls()

    def shutdown(self):
        self._publisher_service.shutdown()
        self._joystick_widget.reset_position()

    @pyqtSlot()
    def _update_dead_zone_controls(self):
        dead_zone_value = int(self._config_manager.get_dead_zone() * 100)
        dead_zone_x_value = int(self._config_manager.get_dead_zone_x() * 100)

        self._dead_zone_slider.blockSignals(True)
        self._dead_zone_slider.setValue(dead_zone_value)
        self._dead_zone_slider.blockSignals(False)
        self._dead_zone_label.setText(f"{dead_zone_value}%")

        self._dead_zone_x_slider.blockSignals(True)
        self._dead_zone_x_slider.setValue(dead_zone_x_value)
        self._dead_zone_x_slider.blockSignals(False)
        self._dead_zone_x_label.setText(f"{dead_zone_x_value}%")
