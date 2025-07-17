"""Configuration utilities for the virtual joystick plugin."""

from dataclasses import dataclass

from python_qt_binding.QtCore import QObject, pyqtSignal


@dataclass
class JoystickConfig:
    """Container for the user editable joystick configuration."""

    topic_name: str = "joy"
    publish_rate: float = 20.0  # Hz
    dead_zone: float = 0.05  # Circular dead zone (0.0-0.9)

    def __post_init__(self):
        self.validate()

    def validate(self):
        if not self.topic_name or not self.topic_name.strip():
            raise ValueError("Topic name cannot be empty")

        if not 1.0 <= self.publish_rate <= 100.0:
            raise ValueError("Publish rate must be between 1.0 and 100.0 Hz")
            
        if not 0.0 <= self.dead_zone <= 0.9:
            raise ValueError("Dead zone must be between 0.0 and 0.9")


class ConfigurationManager(QObject):
    """Tracks configuration values and emits change notifications."""

    config_changed = pyqtSignal()
    topic_changed = pyqtSignal(str)
    rate_changed = pyqtSignal(float)
    dead_zone_changed = pyqtSignal()

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

    def save_settings(self, settings) -> None:
        settings.set_value('topic_name', self._config.topic_name)
        settings.set_value('publish_rate', self._config.publish_rate)
        settings.set_value('dead_zone', self._config.dead_zone)

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
        if dead_zone_value is None:
            dead_zone_value = settings.value('radial_dead_zone')  # backward compatibility
        dead_zone = safe_convert(dead_zone_value, float, 0.05)
        try:
            self.set_dead_zone(dead_zone)
        except ValueError:
            self.set_dead_zone(0.05)
