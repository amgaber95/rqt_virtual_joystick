from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Joy
from python_qt_binding.QtCore import QObject, QTimer


DEFAULT_BUTTON_COUNT = 12
DEFAULT_AXIS_COUNT = 6


@dataclass
class JoyPublishSettings:
    topic: str = "joy"
    rate_hz: float = 20.0
    enabled: bool = True
    axis_count: int = DEFAULT_AXIS_COUNT
    button_count: int = DEFAULT_BUTTON_COUNT
    qos_reliability: QoSReliabilityPolicy = QoSReliabilityPolicy.RELIABLE
    qos_history: QoSHistoryPolicy = QoSHistoryPolicy.KEEP_LAST
    qos_depth: int = 10


class JoyPublisherService(QObject):
    """Minimal service that publishes sensor_msgs/Joy at a fixed rate."""

    def __init__(self, ros_node: Node, settings: Optional[JoyPublishSettings] = None) -> None:
        super().__init__()
        self._node = ros_node
        self._settings = settings or JoyPublishSettings()

        self._publisher: Optional[rclpy.publisher.Publisher] = None

        # Current state buffers
        self._axes: List[float] = [0.0] * max(1, int(self._settings.axis_count))
        self._buttons: List[int] = [0] * max(1, int(self._settings.button_count))

        # Timer for periodic publishing
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timeout)

        # Init publisher & rate
        self._create_or_recreate_publisher(force=True)
        self._apply_rate()

        if self._settings.enabled:
            self._timer.start()

    # ---------------- Public control API ----------------

    def set_enabled(self, enabled: bool) -> None:
        self._settings.enabled = bool(enabled)
        if not self._settings.enabled:
            self._timer.stop()
        else:
            self._on_timeout()  # publish once immediately
            self._timer.start()

    def is_enabled(self) -> bool:
        return self._settings.enabled

    def get_enabled(self) -> bool:
        """Protocol-compatible alias for is_enabled()."""
        return self.is_enabled()

    def set_topic(self, topic: str) -> None:
        topic = (topic or "").strip()
        if not topic:
            self._node.get_logger().warn("Ignoring empty Joy topic")
            return
        if topic == self._settings.topic:
            return
        self._settings.topic = topic
        self._create_or_recreate_publisher()

    def get_topic(self) -> str:
        return self._settings.topic

    def set_rate_hz(self, rate_hz: float) -> None:
        self._settings.rate_hz = max(1.0, float(rate_hz))
        self._apply_rate()

    def get_rate_hz(self) -> float:
        return self._settings.rate_hz

    def set_qos(self,
                reliability: QoSReliabilityPolicy,
                history: QoSHistoryPolicy,
                depth: int) -> None:
        self._settings.qos_reliability = reliability
        self._settings.qos_history = history
        self._settings.qos_depth = int(max(1, depth))
        self._create_or_recreate_publisher()

    def set_sizes(self, axis_count: int, button_count: int) -> None:
        """Resize internal buffers (keeps current values where possible)."""
        axis_count = max(1, int(axis_count))
        button_count = max(1, int(button_count))

        if axis_count != len(self._axes):
            old = self._axes
            self._axes = (old[:axis_count] + [0.0] * max(0, axis_count - len(old)))[:axis_count]

        if button_count != len(self._buttons):
            old = self._buttons
            self._buttons = (old[:button_count] + [0] * max(0, button_count - len(old)))[:button_count]

        self._settings.axis_count = axis_count
        self._settings.button_count = button_count

    # ---------------- State updates ----------------

    def update_axes(self, x: float, y: float, additional_axes: Optional[List[float]] = None) -> None:
        """Update axes 0 and 1 (normalized [-1, 1]) and optionally more axes."""
        if len(self._axes) >= 1:
            self._axes[0] = max(-1.0, min(1.0, float(x)))
        if len(self._axes) >= 2:
            self._axes[1] = max(-1.0, min(1.0, float(y)))

        if additional_axes:
            for i, v in enumerate(additional_axes, start=2):
                if i < len(self._axes):
                    self._axes[i] = max(-1.0, min(1.0, float(v)))
                else:
                    break

        if self._settings.enabled and self._publisher and not self._timer.isActive():
            self._timer.start()

    def update_button(self, button_index: int, pressed: bool) -> None:
        """Set one button (expands array as needed if index is beyond current size)."""
        if button_index < 0:
            return
        if button_index >= len(self._buttons):
            # Expand to fit; keep zeros for new buttons
            self._buttons.extend([0] * (button_index + 1 - len(self._buttons)))
            self._settings.button_count = len(self._buttons)

        self._buttons[button_index] = 1 if pressed else 0

        if self._settings.enabled and self._publisher and not self._timer.isActive():
            self._timer.start()

    # ---------------- Optional external loop control ----------------

    def start(self) -> None:
        if self._settings.enabled and self._publisher and not self._timer.isActive():
            self._timer.start()

    def stop(self) -> None:
        if self._timer.isActive():
            self._timer.stop()

    def is_publishing(self) -> bool:
        return self._timer.isActive()

    def shutdown(self) -> None:
        self.stop()
        if self._publisher:
            try:
                self._node.destroy_publisher(self._publisher)
            except Exception as exc:
                self._node.get_logger().warn(f"Failed to destroy Joy publisher: {exc}")
        self._publisher = None
        self._node.get_logger().info("JoyPublisherService shut down")

    # ---------------- Internals ----------------

    def _apply_rate(self) -> None:
        interval_ms = int(1000.0 / max(1.0, self._settings.rate_hz))
        self._timer.setInterval(interval_ms)

    def _qos(self) -> QoSProfile:
        return QoSProfile(
            reliability=self._settings.qos_reliability,
            history=self._settings.qos_history,
            depth=self._settings.qos_depth,
        )

    def _create_or_recreate_publisher(self, *, force: bool = False) -> None:
        """(Re)create publisher when topic or QoS changes."""
        was_active = self._timer.isActive()
        if was_active:
            self._timer.stop()

        # Destroy old publisher
        if self._publisher:
            try:
                self._node.destroy_publisher(self._publisher)
            except Exception as exc:
                self._node.get_logger().warn(f"Failed to destroy old Joy publisher: {exc}")
            self._publisher = None

        # Create new publisher
        try:
            self._publisher = self._node.create_publisher(Joy, self._settings.topic, self._qos())
            self._node.get_logger().info(f"Created Joy publisher on '{self._settings.topic}'")
        except Exception as exc:
            self._publisher = None
            self._node.get_logger().error(f"Failed to create Joy publisher for '{self._settings.topic}': {exc}")

        # Resume timer if needed
        if was_active and self._settings.enabled and self._publisher:
            self._timer.start()

    def _on_timeout(self) -> None:
        if not self._settings.enabled or not self._publisher:
            return
        try:
            msg = Joy()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.axes = list(self._axes)
            msg.buttons = list(self._buttons)
            self._publisher.publish(msg)
        except Exception as exc:
            self._node.get_logger().error(f"Failed to publish Joy: {exc}")
