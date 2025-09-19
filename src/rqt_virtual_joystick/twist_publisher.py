from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from python_qt_binding.QtCore import QObject, QTimer, pyqtSignal

from .config_manager import ConfigurationManager


class TwistPublisherService(QObject):
    """Publish geometry_msgs/Twist or TwistStamped derived from joystick input."""

    publisher_error = pyqtSignal(str)
    message_published = pyqtSignal()

    def __init__(self, ros_node: Node, config_manager: ConfigurationManager):
        super().__init__()
        self._node = ros_node
        self._config_manager = config_manager

        self._publisher: Optional[rclpy.publisher.Publisher] = None
        self._publisher_topic: Optional[str] = None
        self._publisher_is_stamped: Optional[bool] = None

        self._enabled = self._config_manager.is_twist_publish_enabled()
        self._use_stamped = self._config_manager.is_twist_use_stamped_enabled()
        self._linear_scale, self._angular_scale = self._config_manager.get_twist_scales()
        self._holonomic = self._config_manager.is_twist_holonomic_enabled()

        self._current_twist = Twist()
        self._last_axes = (0.0, 0.0)

        self._publish_timer = QTimer(self)
        self._publish_timer.timeout.connect(self._publish_twist)

        self._rebuild_timer = QTimer(self)
        self._rebuild_timer.setSingleShot(True)
        self._rebuild_timer.timeout.connect(self._build_publisher)
        self._resume_after_rebuild = False

        self._connect_config_signals()
        self._queue_publisher_rebuild()
        self._update_timer_rate(resume=False)

    def _can_publish(self) -> bool:
        """Check if publishing is currently possible and enabled."""
        return bool(self._publisher and self._enabled)

    def _start_publishing_if_ready(self, *, force: bool = False) -> None:
        """Start the publish timer if conditions are met."""
        if not self._can_publish():
            return
        if force or not self._publish_timer.isActive():
            self._publish_timer.start()

    def _stop_publishing(self) -> None:
        """Stop the publish timer."""
        if self._publish_timer.isActive():
            self._publish_timer.stop()

    def _refresh_scales(self) -> None:
        """Update linear and angular scales from configuration."""
        self._linear_scale, self._angular_scale = self._config_manager.get_twist_scales()

    def _connect_config_signals(self) -> None:
        """Connect configuration manager signals to handler methods."""
        self._config_manager.twist_topic_changed.connect(self._on_topic_changed)
        self._config_manager.twist_rate_changed.connect(self._on_rate_changed)
        self._config_manager.twist_publish_enabled_changed.connect(self._on_enabled_changed)
        self._config_manager.twist_scales_changed.connect(self._on_scales_changed)
        self._config_manager.twist_holonomic_changed.connect(self._on_holonomic_changed)
        self._config_manager.twist_use_stamped_changed.connect(self._on_use_stamped_changed)

    def _log_info(self, message: str) -> None:
        """Log an info message."""
        self._node.get_logger().info(message)

    def _log_warning(self, message: str) -> None:
        """Log a warning message."""
        self._node.get_logger().warn(message)

    def _log_error(self, message: str) -> None:
        """Log an error message."""
        self._node.get_logger().error(message)

    def _handle_publisher_creation_error(self, topic_name: str, exc: Exception) -> None:
        """Handle errors during publisher creation."""
        error_msg = f"Failed to create Twist publisher for topic '{topic_name}': {exc}"
        self._log_error(error_msg)
        self.publisher_error.emit(error_msg)

    def _handle_publish_error(self, exc: Exception) -> None:
        """Handle errors during message publishing."""
        error_msg = f"Failed to publish Twist message: {exc}"
        self._log_error(error_msg)
        self.publisher_error.emit(error_msg)

    def _create_qos_profile(self) -> QoSProfile:
        """Create the QoS profile for the Twist publisher."""
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

    def _get_message_type(self):
        """Get the appropriate message type based on configuration."""
        return TwistStamped if self._use_stamped else Twist

    def _dispose_publisher(self, publisher: Optional[rclpy.publisher.Publisher]) -> None:
        """Safely dispose of a publisher instance."""
        if publisher is None:
            return
        try:
            self._node.destroy_publisher(publisher)
        except Exception as exc:  # pragma: no cover - defensive logging
            self._log_warning(f"Failed to destroy Twist publisher instance: {exc}")

    def _unregister_publisher(self) -> None:
        """Clean up the current publisher and reset state."""
        if self._publisher is not None:
            self._dispose_publisher(self._publisher)
        self._publisher = None
        self._publisher_topic = None
        self._publisher_is_stamped = None

    def _queue_publisher_rebuild(self, delay_ms: int = 0, resume: bool = False) -> None:
        """Schedule a publisher rebuild with optional delay and resume flag."""
        if resume:
            self._resume_after_rebuild = True
        self._rebuild_timer.stop()
        self._rebuild_timer.start(max(0, delay_ms))

    def _build_publisher(self) -> None:
        """Build and configure the ROS publisher."""
        topic_name = self._config_manager.get_twist_topic().strip()
        if not topic_name:
            self._unregister_publisher()
            self._resume_after_rebuild = False
            return

        message_type = self._get_message_type()
        qos_profile = self._create_qos_profile()

        try:
            publisher = self._node.create_publisher(message_type, topic_name, qos_profile)
        except Exception as exc:
            self._handle_publisher_creation_error(topic_name, exc)
            self._resume_after_rebuild = False
            return

        self._unregister_publisher()
        self._publisher = publisher
        self._publisher_topic = topic_name
        self._publisher_is_stamped = self._use_stamped

        msg_label = "TwistStamped" if self._use_stamped else "Twist"
        self._log_info(f"Created {msg_label} publisher for topic: {topic_name}")

        self._update_timer_rate(resume=self._resume_after_rebuild)
        self._resume_after_rebuild = False

    def _update_timer_rate(self, resume: Optional[bool] = None) -> None:
        """Update the publish timer rate based on configuration.""" 
        rate_hz = self._config_manager.get_twist_publish_rate()
        interval_ms = int(1000.0 / rate_hz)

        was_active = self._publish_timer.isActive() if resume is None else resume

        self._stop_publishing()
        self._publish_timer.setInterval(interval_ms)
        
        if was_active:
            self._start_publishing_if_ready(force=True)

    def _calculate_linear_velocity(self, x: float, y: float) -> tuple[float, float, float]:
        """Calculate linear velocity components from joystick axes."""
        linear_x = y * self._linear_scale
        linear_y = -x * self._linear_scale if self._holonomic else 0.0
        linear_z = 0.0
        return linear_x, linear_y, linear_z

    def _calculate_angular_velocity(self, x: float, y: float, linear_x: float) -> tuple[float, float, float]:
        """Calculate angular velocity components from joystick axes and linear velocity."""
        angular_x = 0.0
        angular_y = 0.0
        
        if self._holonomic:
            angular_z = 0.0
        else:
            angular_z = -x * self._angular_scale
            # Reverse angular velocity when moving backwards for car-like behavior
            if linear_x < 0.0:
                angular_z *= -1.0
        
        return angular_x, angular_y, angular_z

    def update_from_axes(self, x: float, y: float) -> None:
        """Update twist message from joystick axes input."""
        self._last_axes = (x, y)

        linear_x, linear_y, linear_z = self._calculate_linear_velocity(x, y)
        angular_x, angular_y, angular_z = self._calculate_angular_velocity(x, y, linear_x)

        self._current_twist.linear.x = linear_x
        self._current_twist.linear.y = linear_y
        self._current_twist.linear.z = linear_z
        self._current_twist.angular.x = angular_x
        self._current_twist.angular.y = angular_y
        self._current_twist.angular.z = angular_z

        self._start_publishing_if_ready()

    def stop(self) -> None:
        """Stop publishing Twist messages."""
        self._stop_publishing()

    def _create_twist_message(self) -> Twist:
        """Create and normalize a Twist message from current state."""
        twist = Twist()
        twist.linear.x = float(self._current_twist.linear.x)
        twist.linear.y = float(self._current_twist.linear.y)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self._current_twist.angular.z)
        return twist

    def _create_twist_stamped_message(self) -> TwistStamped:
        """Create a TwistStamped message from current state."""
        stamped = TwistStamped()
        stamped.header.stamp = self._node.get_clock().now().to_msg()
        stamped.twist = self._create_twist_message()
        return stamped

    def _publish_twist(self) -> None:
        if not self._publisher or not self._enabled:
            return

        published = False

        try:
            if self._use_stamped and self._publisher_is_stamped:
                message = self._create_twist_stamped_message()
                self._publisher.publish(message)
                published = True
            elif not self._use_stamped and not self._publisher_is_stamped:
                message = self._create_twist_message()
                self._publisher.publish(message)
                published = True
        except Exception as exc:  # pragma: no cover - defensive logging
            self._handle_publish_error(exc)
            return

        if published:
            self.message_published.emit()

    def _on_topic_changed(self, _topic: str) -> None:
        was_publishing = self._publish_timer.isActive()
        self._unregister_publisher()
        self._queue_publisher_rebuild(resume=was_publishing)

    def _on_rate_changed(self, _rate: float) -> None:
        self._update_timer_rate()

    def _on_enabled_changed(self, enabled: bool) -> None:
        self._enabled = bool(enabled)
        if not self._enabled:
            self.stop()
        else:
            self._start_publishing_if_ready()

    def _on_scales_changed(self) -> None:
        self._refresh_scales()
        self.update_from_axes(*self._last_axes)

    def _on_holonomic_changed(self, enabled: bool) -> None:
        self._holonomic = bool(enabled)
        self.update_from_axes(*self._last_axes)
        self._start_publishing_if_ready()

    def _on_use_stamped_changed(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._use_stamped:
            return

        was_publishing = self._publish_timer.isActive()
        self._use_stamped = enabled
        self._unregister_publisher()
        self._queue_publisher_rebuild(delay_ms=100, resume=was_publishing)

    def shutdown(self) -> None:
        self.stop()
        self._rebuild_timer.stop()
        self._unregister_publisher()
