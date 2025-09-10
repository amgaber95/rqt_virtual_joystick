from weakref import ref

import sip

from python_qt_binding.QtCore import Qt, pyqtSlot, QEvent, QObject
from python_qt_binding.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QSizePolicy,
    QApplication,
)

from .joystick_widget import JoystickWidget
from .controller_buttons_widget import ControllerButtonsWidget
from .config_manager import ConfigurationManager
from .joy_publisher import JoyPublisherService
from .twist_publisher import TwistPublisherService
from .control_panels import JoyOutputPanel, TwistOutputPanel, JoystickConfigPanel


class HolonomicKeyHandler(QObject):
    """Temporary holonomic override driven by Shift key state."""

    def __init__(self, root_widget: QWidget, config_manager: ConfigurationManager):
        super().__init__(root_widget)
        self._root_widget_ref = ref(root_widget)
        self._config_manager = config_manager
        self._shift_active = False
        self._restore_state = self._config_manager.is_twist_holonomic_enabled()

        app = QApplication.instance()
        self._application = app
        self._installed_filter = False
        if self._application is not None:
            self._application.installEventFilter(self)
            self._installed_filter = True

        root_widget.destroyed.connect(self._on_root_destroyed)
        self._config_manager.twist_holonomic_changed.connect(self._on_holonomic_changed)

    def eventFilter(self, obj, event):
        root_widget = self._root_widget()
        if self._is_deleted(root_widget):
            self._uninstall()
            return False

        if self._is_deleted(obj):
            return False

        if not self._is_relevant_object(root_widget, obj):
            return False

        if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
            if event.key() == Qt.Key_Shift:
                self._activate_shift_override()
        elif event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
            if event.key() == Qt.Key_Shift:
                self._deactivate_shift_override()

        return False

    def _is_relevant_object(self, root_widget: QWidget, obj) -> bool:
        if self._is_deleted(root_widget):
            return False

        if not isinstance(obj, QWidget) or self._is_deleted(obj):
            return False
        try:
            return obj is root_widget or root_widget.isAncestorOf(obj)
        except RuntimeError:
            return False

    @staticmethod
    def _is_deleted(qobj) -> bool:
        if qobj is None:
            return True
        try:
            return sip.isdeleted(qobj)
        except (RuntimeError, ReferenceError):
            return True
        except TypeError:
            return False

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
        if self._installed_filter and self._application is not None:
            self._application.removeEventFilter(self)
            self._installed_filter = False
        self._application = None
        self._root_widget_ref = None
        try:
            self._config_manager.twist_holonomic_changed.disconnect(self._on_holonomic_changed)
        except TypeError:
            pass

    def _on_root_destroyed(self, _obj=None):
        self._uninstall()

    def _root_widget(self):
        return self._root_widget_ref() if self._root_widget_ref is not None else None


class JoystickMainWidget(QWidget):
    """Main widget that wires the joystick visualizer with configuration controls."""

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)

        self._ros_node = ros_node
        self._config_manager = ConfigurationManager()
        self._publisher_service = JoyPublisherService(ros_node, self._config_manager)
        self._twist_publisher_service = TwistPublisherService(ros_node, self._config_manager)
        self._twist_publisher_service.update_from_axes(0.0, 0.0)

        self.setMaximumWidth(700)  # Increased to accommodate joystick + buttons

        self._init_ui()
        self._connect_signals()
        self.setFocusPolicy(Qt.StrongFocus)
        self._holonomic_key_handler = HolonomicKeyHandler(self, self._config_manager)
        self._controller_buttons_widget.set_sticky_buttons(
            self._config_manager.is_sticky_buttons_enabled()
        )

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
        controls_widget = QWidget()
        controls_layout = QVBoxLayout()
        controls_layout.setSpacing(8)

        self._publishing_panel = JoyOutputPanel(self._config_manager)
        self._twist_panel = TwistOutputPanel(self._config_manager)
        self._joystick_config_panel = JoystickConfigPanel(self._config_manager)

        controls_layout.addWidget(self._publishing_panel)
        controls_layout.addWidget(self._twist_panel)
        controls_layout.addWidget(self._joystick_config_panel)
        controls_layout.addStretch(1)

        controls_widget.setLayout(controls_layout)
        controls_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

        main_layout.addWidget(controls_widget, 0)
        main_layout.addStretch(1)

        self.setLayout(main_layout)

    def _connect_signals(self):
        self._joystick_widget.position_changed.connect(self._on_joystick_moved)
        self._controller_buttons_widget.button_pressed.connect(self._on_button_pressed)
        self._publisher_service.publisher_error.connect(self._on_publisher_error)
        self._twist_publisher_service.publisher_error.connect(self._on_publisher_error)
        self._config_manager.sticky_buttons_changed.connect(self._on_sticky_buttons_changed)

    @pyqtSlot(float, float)
    def _on_joystick_moved(self, x: float, y: float):
        self._publisher_service.update_axes(x, y)
        self._twist_publisher_service.update_from_axes(x, y)

    @pyqtSlot(int, bool)
    def _on_button_pressed(self, button_index: int, pressed: bool):
        """Handle face button press/release events."""
        self._publisher_service.update_button(button_index, pressed)

    @pyqtSlot(str)
    def _on_publisher_error(self, error_msg: str):
        self._ros_node.get_logger().error(f"Publisher error: {error_msg}")

    def save_settings(self, settings):
        self._config_manager.save_settings(settings)

    def restore_settings(self, settings):
        self._config_manager.restore_settings(settings)
        self._publishing_panel.refresh_from_config()
        self._twist_panel.refresh_from_config()
        self._joystick_config_panel.refresh_from_config()
        self._controller_buttons_widget.set_sticky_buttons(
            self._config_manager.is_sticky_buttons_enabled()
        )

    def shutdown(self):
        self._publisher_service.shutdown()
        self._twist_publisher_service.shutdown()
        self._joystick_widget.reset_position()
        self._controller_buttons_widget.reset_buttons()

    @pyqtSlot(bool)
    def _on_sticky_buttons_changed(self, enabled: bool):
        self._controller_buttons_widget.set_sticky_buttons(enabled)
