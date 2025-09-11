from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from python_qt_binding.QtCore import QObject, QTimer, pyqtSignal

from .config_manager import ConfigurationManager


class TwistPublisherService(QObject):
    """Handles publishing of geometry_msgs/Twist messages derived from joystick input."""

    publisher_error = pyqtSignal(str)
    message_published = pyqtSignal()

    def __init__(self, ros_node: Node, config_manager: ConfigurationManager):
        super().__init__()
        self._node = ros_node
        self._config_manager = config_manager
        self._publisher: Optional[rclpy.publisher.Publisher] = None
        self._enabled = self._config_manager.is_twist_publish_enabled()

        self._current_twist = Twist()
        self._last_axes = (0.0, 0.0)

        self._publish_timer = QTimer(self)
        self._publish_timer.timeout.connect(self._publish_twist)

        self._connect_config_signals()
        self._create_publisher()
        self._update_timer_rate()

    def _connect_config_signals(self) -> None:
        self._config_manager.twist_topic_changed.connect(self._on_topic_changed)
        self._config_manager.twist_rate_changed.connect(self._on_rate_changed)
        self._config_manager.twist_publish_enabled_changed.connect(self._on_enabled_changed)
        self._config_manager.twist_scales_changed.connect(self._on_scales_changed)
        self._config_manager.twist_holonomic_changed.connect(self._on_holonomic_changed)

    def _dispose_publisher(self, publisher: Optional[rclpy.publisher.Publisher]) -> None:
        """Destroy a ROS publisher, ignoring cleanup errors to avoid crashes."""
        if publisher is None:
            return

        try:
            self._node.destroy_publisher(publisher)
        except Exception as exc:
            self._node.get_logger().warn(
                f"Failed to destroy Twist publisher instance: {exc}"
            )

    def _create_publisher(self) -> bool:
        topic_name = self._config_manager.get_twist_topic()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        try:
            new_publisher = self._node.create_publisher(Twist, topic_name, qos_profile)
        except Exception as exc:
            error_msg = f"Failed to create Twist publisher for topic '{topic_name}': {exc}"
            self._node.get_logger().error(error_msg)
            self.publisher_error.emit(error_msg)
            return False

        old_publisher = self._publisher
        self._publisher = new_publisher
        self._node.get_logger().info(f"Created Twist publisher for topic: {topic_name}")

        if old_publisher is not None and old_publisher is not new_publisher:
            self._dispose_publisher(old_publisher)

        return True

    def _update_timer_rate(self) -> None:
        rate_hz = self._config_manager.get_twist_publish_rate()
        interval_ms = int(1000.0 / rate_hz)

        if self._publish_timer.isActive():
            self._publish_timer.stop()

        self._publish_timer.setInterval(interval_ms)
        if self._publisher and self._enabled:
            self._publish_timer.start()

    def update_from_axes(self, x: float, y: float) -> None:
        self._last_axes = (x, y)
        linear_scale, angular_scale = self._config_manager.get_twist_scales()
        holonomic = self._config_manager.is_twist_holonomic_enabled()

        linear_x = y * linear_scale
        self._current_twist.linear.x = linear_x
        self._current_twist.linear.y = -x * linear_scale if holonomic else 0.0
        self._current_twist.linear.z = 0.0

        self._current_twist.angular.x = 0.0
        self._current_twist.angular.z = 0.0

        # Positive joystick X (right) should result in negative rotation.
        angular_command = -x * angular_scale

        # When driving backward, flip turn direction so stick right always means "move right".
        if linear_x < 0.0 and not holonomic:
            angular_command *= -1.0

        self._current_twist.angular.z = 0.0 if holonomic else angular_command

        if self._enabled and self._publisher and not self._publish_timer.isActive():
            self._publish_timer.start()

    def stop(self) -> None:
        if self._publish_timer.isActive():
            self._publish_timer.stop()

    def _publish_twist(self) -> None:
        if not self._publisher or not self._enabled:
            return

        try:
            self._current_twist.linear.x = float(self._current_twist.linear.x)
            self._current_twist.linear.y = float(self._current_twist.linear.y)
            self._current_twist.linear.z = 0.0
            self._current_twist.angular.x = 0.0
            if not self._config_manager.is_twist_holonomic_enabled():
                self._current_twist.angular.y = 0.0
            self._publisher.publish(self._current_twist)
            self.message_published.emit()
        except Exception as exc:
            error_msg = f"Failed to publish Twist message: {exc}"
            self._node.get_logger().error(error_msg)
            self.publisher_error.emit(error_msg)

    def _on_topic_changed(self, _topic: str) -> None:
        was_publishing = self._publish_timer.isActive()
        if was_publishing:
            self._publish_timer.stop()
        if self._create_publisher() and was_publishing:
            self._publish_timer.start()

    def _on_rate_changed(self, _rate: float) -> None:
        self._update_timer_rate()

    def _on_enabled_changed(self, enabled: bool) -> None:
        self._enabled = enabled
        if not enabled:
            self.stop()
        else:
            if self._publisher and not self._publish_timer.isActive():
                self._publish_timer.start()

    def _on_scales_changed(self) -> None:
        self.update_from_axes(*self._last_axes)

    def _on_holonomic_changed(self, _enabled: bool) -> None:
        self.update_from_axes(*self._last_axes)
        if self._enabled and self._publisher and not self._publish_timer.isActive():
            self._publish_timer.start()

    def shutdown(self) -> None:
        self.stop()
        if self._publisher is not None:
            self._dispose_publisher(self._publisher)
            self._publisher = None
