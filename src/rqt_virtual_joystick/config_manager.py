"""Configuration utilities for the virtual joystick plugin."""

from dataclasses import dataclass

from python_qt_binding.QtCore import QObject, pyqtSignal


RETURN_MODE_BOTH = "both"
RETURN_MODE_HORIZONTAL = "horizontal"
RETURN_MODE_VERTICAL = "vertical"
RETURN_MODE_NONE = "none"


@dataclass
class JoystickConfig:
    """Container for the user editable joystick configuration."""

    topic_name: str = "joy"
    publish_rate: float = 20.0  # Hz
    dead_zone: float = 0.05  # Circular dead zone (0.0-0.9)
    dead_zone_x: float = 0.0  # X-axis dead zone (0.0-0.9)
    dead_zone_y: float = 0.0  # Y-axis dead zone (0.0-0.9)
    expo_x: float = 0.0  # X-axis exponential response (0-100%)
    expo_y: float = 0.0  # Y-axis exponential response (0-100%)
    publish_enabled: bool = True
    sticky_buttons: bool = False
    return_mode: str = RETURN_MODE_BOTH
    twist_topic: str = "/cmd_vel"
    twist_publish_rate: float = 20.0
    twist_publish_enabled: bool = False
    twist_linear_scale: float = 0.5
    twist_angular_scale: float = 1.0
    twist_holonomic: bool = False

    def __post_init__(self):
        self.validate()

    def validate(self):
        if not self.topic_name or not self.topic_name.strip():
            raise ValueError("Topic name cannot be empty")

        if not 1.0 <= self.publish_rate <= 100.0:
            raise ValueError("Publish rate must be between 1.0 and 100.0 Hz")
            
        if not 0.0 <= self.dead_zone <= 0.9:
            raise ValueError("Dead zone must be between 0.0 and 0.9")

        if not 0.0 <= self.dead_zone_x <= 0.9:
            raise ValueError("X dead zone must be between 0.0 and 0.9")

        if not 0.0 <= self.dead_zone_y <= 0.9:
            raise ValueError("Y dead zone must be between 0.0 and 0.9")

        if not 0.0 <= self.expo_x <= 100.0:
            raise ValueError("X expo must be between 0.0 and 100.0%")

        if not 0.0 <= self.expo_y <= 100.0:
            raise ValueError("Y expo must be between 0.0 and 100.0%")

        if not 0.0 <= self.expo_y <= 100.0:
            raise ValueError("Y expo must be between 0.0 and 100.0%")

        valid_modes = [
            RETURN_MODE_BOTH,
            RETURN_MODE_HORIZONTAL,
            RETURN_MODE_VERTICAL,
            RETURN_MODE_NONE,
        ]
        if self.return_mode not in valid_modes:
            raise ValueError(f"Return mode must be one of {valid_modes}")

        if not self.twist_topic or not self.twist_topic.strip():
            raise ValueError("Twist topic name cannot be empty")

        if not 1.0 <= self.twist_publish_rate <= 100.0:
            raise ValueError("Twist publish rate must be between 1.0 and 100.0 Hz")

        for value, label in [
            (self.twist_linear_scale, "linear"),
            (self.twist_angular_scale, "angular"),
        ]:
            if value < 0.0:
                raise ValueError(f"Twist {label} scale must be non-negative")

class ConfigurationManager(QObject):
    """Tracks configuration values and emits change notifications."""

    config_changed = pyqtSignal()
    topic_changed = pyqtSignal(str)
    rate_changed = pyqtSignal(float)
    dead_zone_changed = pyqtSignal()
    expo_changed = pyqtSignal()
    publish_enabled_changed = pyqtSignal(bool)
    sticky_buttons_changed = pyqtSignal(bool)
    return_mode_changed = pyqtSignal(str)
    twist_topic_changed = pyqtSignal(str)
    twist_rate_changed = pyqtSignal(float)
    twist_publish_enabled_changed = pyqtSignal(bool)
    twist_scales_changed = pyqtSignal()
    twist_holonomic_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._config = JoystickConfig()

    def get_topic_name(self) -> str:
        return self._config.topic_name

    def set_topic_name(self, topic_name: str):
        if not topic_name or not topic_name.strip():
            raise ValueError("Topic name cannot be empty")

        topic_name = topic_name.strip()
        if topic_name != self._config.topic_name:
            self._config.topic_name = topic_name
            self.topic_changed.emit(topic_name)
            self.config_changed.emit()

    def get_publish_rate(self) -> float:
        return self._config.publish_rate

    def set_publish_rate(self, rate: float):
        if not 1.0 <= rate <= 100.0:
            raise ValueError("Publish rate must be between 1.0 and 100.0 Hz")

        if rate != self._config.publish_rate:
            self._config.publish_rate = rate
            self.rate_changed.emit(rate)
            self.config_changed.emit()
            
    def get_dead_zone(self) -> float:
        return self._config.dead_zone

    def set_dead_zone(self, dead_zone: float):
        if not 0.0 <= dead_zone <= 0.9:
            raise ValueError("Dead zone must be between 0.0 and 0.9")

        if dead_zone != self._config.dead_zone:
            self._config.dead_zone = dead_zone
            self.dead_zone_changed.emit()
            self.config_changed.emit()

    def get_dead_zone_x(self) -> float:
        return self._config.dead_zone_x

    def set_dead_zone_x(self, dead_zone_x: float):
        if not 0.0 <= dead_zone_x <= 0.9:
            raise ValueError("X dead zone must be between 0.0 and 0.9")

        if dead_zone_x != self._config.dead_zone_x:
            self._config.dead_zone_x = dead_zone_x
            self.dead_zone_changed.emit()
            self.config_changed.emit()

    def get_dead_zone_y(self) -> float:
        return self._config.dead_zone_y

    def set_dead_zone_y(self, dead_zone_y: float):
        if not 0.0 <= dead_zone_y <= 0.9:
            raise ValueError("Y dead zone must be between 0.0 and 0.9")

        if dead_zone_y != self._config.dead_zone_y:
            self._config.dead_zone_y = dead_zone_y
            self.dead_zone_changed.emit()
            self.config_changed.emit()

    def get_dead_zones(self) -> tuple[float, float, float]:
        return (
            self._config.dead_zone,
            self._config.dead_zone_x,
            self._config.dead_zone_y,
        )

    def get_expo_x(self) -> float:
        return self._config.expo_x

    def set_expo_x(self, expo_x: float):
        if not 0.0 <= expo_x <= 100.0:
            raise ValueError("X expo must be between 0.0 and 100.0%")

        if expo_x != self._config.expo_x:
            self._config.expo_x = expo_x
            self.expo_changed.emit()
            self.config_changed.emit()

    def get_expo_y(self) -> float:
        return self._config.expo_y

    def set_expo_y(self, expo_y: float):
        if not 0.0 <= expo_y <= 100.0:
            raise ValueError("Y expo must be between 0.0 and 100.0%")

        if expo_y != self._config.expo_y:
            self._config.expo_y = expo_y
            self.expo_changed.emit()
            self.config_changed.emit()

    def is_publish_enabled(self) -> bool:
        return self._config.publish_enabled

    def set_publish_enabled(self, enabled: bool):
        enabled = bool(enabled)
        if enabled != self._config.publish_enabled:
            self._config.publish_enabled = enabled
            self.publish_enabled_changed.emit(enabled)
            self.config_changed.emit()

    def is_sticky_buttons_enabled(self) -> bool:
        return self._config.sticky_buttons

    def set_sticky_buttons(self, enabled: bool):
        enabled = bool(enabled)
        if enabled != self._config.sticky_buttons:
            self._config.sticky_buttons = enabled
            self.sticky_buttons_changed.emit(enabled)
            self.config_changed.emit()

    def get_return_mode(self) -> str:
        return self._config.return_mode

    def set_return_mode(self, return_mode: str):
        valid_modes = [
            RETURN_MODE_BOTH,
            RETURN_MODE_HORIZONTAL,
            RETURN_MODE_VERTICAL,
            RETURN_MODE_NONE,
        ]
        if return_mode not in valid_modes:
            raise ValueError("Invalid return mode")

        if return_mode != self._config.return_mode:
            self._config.return_mode = return_mode
            self.return_mode_changed.emit(return_mode)
            self.config_changed.emit()

    def get_twist_topic(self) -> str:
        return self._config.twist_topic

    def set_twist_topic(self, topic_name: str):
        if not topic_name or not topic_name.strip():
            raise ValueError("Twist topic name cannot be empty")

        topic_name = topic_name.strip()
        if topic_name != self._config.twist_topic:
            self._config.twist_topic = topic_name
            self.twist_topic_changed.emit(topic_name)
            self.config_changed.emit()

    def get_twist_publish_rate(self) -> float:
        return self._config.twist_publish_rate

    def set_twist_publish_rate(self, rate: float):
        if not 1.0 <= rate <= 100.0:
            raise ValueError("Twist publish rate must be between 1.0 and 100.0 Hz")

        if rate != self._config.twist_publish_rate:
            self._config.twist_publish_rate = rate
            self.twist_rate_changed.emit(rate)
            self.config_changed.emit()

    def is_twist_publish_enabled(self) -> bool:
        return self._config.twist_publish_enabled

    def set_twist_publish_enabled(self, enabled: bool):
        enabled = bool(enabled)
        if enabled != self._config.twist_publish_enabled:
            self._config.twist_publish_enabled = enabled
            self.twist_publish_enabled_changed.emit(enabled)
            self.config_changed.emit()

    def get_twist_scales(self) -> tuple[float, float]:
        return (
            self._config.twist_linear_scale,
            self._config.twist_angular_scale,
        )

    def set_twist_scales(self, linear: float, angular: float):
        for value, label in [(linear, "linear"), (angular, "angular")]:
            if value < 0.0:
                raise ValueError(f"Twist {label} scale must be non-negative")

        if (
            linear != self._config.twist_linear_scale
            or angular != self._config.twist_angular_scale
        ):
            self._config.twist_linear_scale = linear
            self._config.twist_angular_scale = angular
            self.twist_scales_changed.emit()
            self.config_changed.emit()

    def is_twist_holonomic_enabled(self) -> bool:
        return self._config.twist_holonomic

    def set_twist_holonomic(self, enabled: bool):
        enabled = bool(enabled)
        if enabled != self._config.twist_holonomic:
            self._config.twist_holonomic = enabled
            self.twist_holonomic_changed.emit(enabled)
            self.config_changed.emit()

    def save_settings(self, settings) -> None:
        settings.set_value('topic_name', self._config.topic_name)
        settings.set_value('publish_rate', self._config.publish_rate)
        settings.set_value('dead_zone', self._config.dead_zone)
        settings.set_value('dead_zone_x', self._config.dead_zone_x)
        settings.set_value('dead_zone_y', self._config.dead_zone_y)
        settings.set_value('expo_x', self._config.expo_x)
        settings.set_value('expo_y', self._config.expo_y)
        settings.set_value('publish_enabled', self._config.publish_enabled)
        settings.set_value('return_mode', self._config.return_mode)
        settings.set_value('sticky_buttons', self._config.sticky_buttons)
        settings.set_value('twist_topic', self._config.twist_topic)
        settings.set_value('twist_publish_rate', self._config.twist_publish_rate)
        settings.set_value('twist_publish_enabled', self._config.twist_publish_enabled)
        settings.set_value('twist_linear_scale', self._config.twist_linear_scale)
        settings.set_value('twist_angular_scale', self._config.twist_angular_scale)
        settings.set_value('twist_holonomic', self._config.twist_holonomic)

    def restore_settings(self, settings) -> None:
        def safe_convert(value, converter, default):
            try:
                return converter(value) if value is not None else default
            except (ValueError, TypeError):
                return default

        try:
            self.set_topic_name(settings.value('topic_name', 'joy'))
        except ValueError:
            self.set_topic_name('joy')

        publish_rate = safe_convert(settings.value('publish_rate'), float, 20.0)
        try:
            self.set_publish_rate(publish_rate)
        except ValueError:
            self.set_publish_rate(20.0)
            
        dead_zone_value = settings.value('dead_zone')
        dead_zone = safe_convert(dead_zone_value, float, 0.05)
        try:
            self.set_dead_zone(dead_zone)
        except ValueError:
            self.set_dead_zone(0.05)

        dead_zone_x_value = settings.value('dead_zone_x')
        dead_zone_x = safe_convert(dead_zone_x_value, float, 0.0)
        try:
            self.set_dead_zone_x(dead_zone_x)
        except ValueError:
            self.set_dead_zone_x(0.0)

        dead_zone_y_value = settings.value('dead_zone_y')
        dead_zone_y = safe_convert(dead_zone_y_value, float, 0.0)
        try:
            self.set_dead_zone_y(dead_zone_y)
        except ValueError:
            self.set_dead_zone_y(0.0)

        expo_x_value = settings.value('expo_x')
        expo_x = safe_convert(expo_x_value, float, 0.0)
        try:
            self.set_expo_x(expo_x)
        except ValueError:
            self.set_expo_x(0.0)

        expo_y_value = settings.value('expo_y')
        expo_y = safe_convert(expo_y_value, float, 0.0)
        try:
            self.set_expo_y(expo_y)
        except ValueError:
            self.set_expo_y(0.0)

        publish_enabled_value = settings.value('publish_enabled')
        if isinstance(publish_enabled_value, str):
            publish_enabled = publish_enabled_value.lower() in ('1', 'true', 'yes', 'on')
        elif publish_enabled_value is None:
            publish_enabled = True
        else:
            publish_enabled = bool(publish_enabled_value)
        self.set_publish_enabled(publish_enabled)

        return_mode = settings.value('return_mode', RETURN_MODE_BOTH)
        try:
            self.set_return_mode(return_mode)
        except ValueError:
            self.set_return_mode(RETURN_MODE_BOTH)

        sticky_value = settings.value('sticky_buttons')
        if isinstance(sticky_value, str):
            sticky_enabled = sticky_value.lower() in ('1', 'true', 'yes', 'on')
        elif sticky_value is None:
            sticky_enabled = False
        else:
            sticky_enabled = bool(sticky_value)
        self.set_sticky_buttons(sticky_enabled)

        try:
            self.set_twist_topic(settings.value('twist_topic', '/cmd_vel'))
        except ValueError:
            self.set_twist_topic('/cmd_vel')

        twist_rate = safe_convert(settings.value('twist_publish_rate'), float, 20.0)
        try:
            self.set_twist_publish_rate(twist_rate)
        except ValueError:
            self.set_twist_publish_rate(20.0)

        twist_enabled_value = settings.value('twist_publish_enabled')
        if isinstance(twist_enabled_value, str):
            twist_enabled = twist_enabled_value.lower() in ('1', 'true', 'yes', 'on')
        elif twist_enabled_value is None:
            twist_enabled = False
        else:
            twist_enabled = bool(twist_enabled_value)
        self.set_twist_publish_enabled(twist_enabled)

        linear_scale = safe_convert(settings.value('twist_linear_scale'), float, 0.5)
        angular_scale = safe_convert(settings.value('twist_angular_scale'), float, 1.0)
        try:
            self.set_twist_scales(linear_scale, angular_scale)
        except ValueError:
            self.set_twist_scales(0.5, 1.0)

        holonomic_value = settings.value('twist_holonomic')
        if isinstance(holonomic_value, str):
            holonomic_enabled = holonomic_value.lower() in ('1', 'true', 'yes', 'on')
        elif holonomic_value is None:
            holonomic_enabled = False
        else:
            holonomic_enabled = bool(holonomic_value)
        self.set_twist_holonomic(holonomic_enabled)
