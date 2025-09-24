from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped
from python_qt_binding.QtCore import QObject, QTimer


@dataclass
class TwistPublishSettings:
    topic: str = "cmd_vel"
    rate_hz: float = 30.0
    enabled: bool = True
    use_stamped: bool = False
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    holonomic: bool = False
    qos_reliability: QoSReliabilityPolicy = QoSReliabilityPolicy.RELIABLE
    qos_history: QoSHistoryPolicy = QoSHistoryPolicy.KEEP_LAST
    qos_depth: int = 1


class TwistPublisherService(QObject):
    """Minimal service that publishes Twist/TwistStamped from joystick-like axes."""

    def __init__(self, ros_node: Node, settings: Optional[TwistPublishSettings] = None) -> None:
        super().__init__()
        self._node = ros_node
        self._settings = settings or TwistPublishSettings()

        self._publisher: Optional[rclpy.publisher.Publisher] = None
        self._is_stamped_publisher: Optional[bool] = None  # tracks type of current publisher

        # Current computed twist and last axes (x=left/right, y=forward/back)
        self._twist = Twist()
        self._last_axes: Tuple[float, float] = (0.0, 0.0)

        # Timer for periodic publishing
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timeout)

        # Defer publisher creation until after initial configuration
        # This prevents double-creation during settings restoration
        self._initial_setup_complete = False

        if self._settings.enabled:
            self._timer.start()

    # ---------------- Public control API ----------------

    def set_enabled(self, enabled: bool) -> None:
        self._settings.enabled = bool(enabled)
        if not self._settings.enabled:
            self._timer.stop()
        else:
            # publish once immediately, then start timer
            self._on_timeout()
            self._timer.start()

    def complete_initial_setup(self) -> None:
        """Complete the initial setup after all configuration is loaded."""
        if not self._initial_setup_complete:
            self._create_or_recreate_publisher(force=True)
            self._apply_rate()
            self._initial_setup_complete = True

    def is_enabled(self) -> bool:
        return self._settings.enabled

    def get_enabled(self) -> bool:
        """Protocol-compatible alias for is_enabled()."""
        return self.is_enabled()

    def set_topic(self, topic: str) -> None:
        topic = (topic or "").strip()
        if not topic:
            self._node.get_logger().warn("Ignoring empty Twist topic")
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

    def set_use_stamped(self, on: bool) -> None:
        on = bool(on)
        if on == self._settings.use_stamped:
            return
        
        # Store the old setting temporarily
        old_stamped = self._settings.use_stamped
        self._settings.use_stamped = on
        
        # If there's a message type change, use a delayed rebuild to avoid conflicts
        if self._publisher is not None and old_stamped != on:
            was_active = self._timer.isActive()
            if was_active:
                self._timer.stop()
                
            # Destroy the old publisher completely
            try:
                self._node.destroy_publisher(self._publisher)
                self._publisher = None
                self._is_stamped_publisher = None
                
                # Use a single-shot timer to rebuild after a short delay
                rebuild_timer = QTimer(self)
                rebuild_timer.setSingleShot(True)
                rebuild_timer.timeout.connect(lambda: self._delayed_rebuild(was_active))
                rebuild_timer.start(100)  # 100ms delay
                
            except Exception as exc:
                self._node.get_logger().error(f"Failed to destroy publisher during type change: {exc}")
                self._create_or_recreate_publisher()
        else:
            self._create_or_recreate_publisher()
    
    def _delayed_rebuild(self, resume_publishing: bool) -> None:
        """Rebuild publisher after delay and optionally resume publishing."""
        self._create_or_recreate_publisher(force=True)
        if resume_publishing and self._settings.enabled and self._publisher:
            self._timer.start()

    def get_use_stamped(self) -> bool:
        return self._settings.use_stamped

    def set_scales(self, linear: float, angular: float) -> None:
        self._settings.linear_scale = float(linear)
        self._settings.angular_scale = float(angular)
        # Recompute with last axes so next publish uses updated scales
        self.update_from_axes(*self._last_axes)

    def get_scales(self) -> Tuple[float, float]:
        return self._settings.linear_scale, self._settings.angular_scale

    def set_holonomic(self, on: bool) -> None:
        self._settings.holonomic = bool(on)
        self.update_from_axes(*self._last_axes)

    def get_holonomic(self) -> bool:
        return self._settings.holonomic

    def set_qos(self,
                reliability: QoSReliabilityPolicy,
                history: QoSHistoryPolicy,
                depth: int) -> None:
        self._settings.qos_reliability = reliability
        self._settings.qos_history = history
        self._settings.qos_depth = int(max(1, depth))
        self._create_or_recreate_publisher()

    # Joystick-like input (normalized -1..1)
    def update_from_axes(self, x: float, y: float) -> None:
        self._last_axes = (float(x), float(y))
        linear_x = y * self._settings.linear_scale
        linear_y = (-x * self._settings.linear_scale) if self._settings.holonomic else 0.0

        # Car-like steering on Z when non-holonomic
        if self._settings.holonomic:
            angular_z = 0.0
        else:
            angular_z = -x * self._settings.angular_scale
            if linear_x < 0.0:
                angular_z *= -1.0  # reverse steering when backing up

        self._twist.linear.x = float(linear_x)
        self._twist.linear.y = float(linear_y)
        self._twist.linear.z = 0.0
        self._twist.angular.x = 0.0
        self._twist.angular.y = 0.0
        self._twist.angular.z = float(angular_z)

        if self._settings.enabled and self._publisher and not self._timer.isActive():
            self._timer.start()

    # Optional external control
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
                self._node.get_logger().warn(f"Failed to destroy Twist publisher: {exc}")
        self._publisher = None
        self._node.get_logger().info("TwistPublisherService shut down")

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
        """(Re)create publisher when topic/type/QoS changes."""
        # Skip during initial setup phase to prevent double-creation
        if not force and not getattr(self, '_initial_setup_complete', False):
            return
            
        target_is_stamped = self._settings.use_stamped
        if (not force and
            self._publisher is not None and
            self._is_stamped_publisher == target_is_stamped):
            # Only topic/QoS may have changed; still recreate for safety
            pass

        # Preserve active state
        was_active = self._timer.isActive()
        if was_active:
            self._timer.stop()

        # Destroy old publisher with extra safety
        if self._publisher:
            try:
                self._node.destroy_publisher(self._publisher)
                # Give ROS time to clean up internal references
                import time
                time.sleep(0.001)  # 1ms delay
            except Exception as exc:
                self._node.get_logger().warn(f"Failed to destroy old Twist publisher: {exc}")
            finally:
                self._publisher = None
                self._is_stamped_publisher = None

        # Create new publisher with better error handling
        topic_name = self._settings.topic.strip()
        if not topic_name:
            self._node.get_logger().warn("Cannot create publisher: empty topic name")
            return

        try:
            msg_type = TwistStamped if self._settings.use_stamped else Twist
            qos_profile = self._qos()
            
            # Validate that we're not conflicting with existing publishers
            self._publisher = self._node.create_publisher(msg_type, topic_name, qos_profile)
            self._is_stamped_publisher = self._settings.use_stamped
            
            label = "TwistStamped" if self._is_stamped_publisher else "Twist"
            self._node.get_logger().info(f"Created {label} publisher on '{topic_name}'")
            
        except Exception as exc:
            self._publisher = None
            self._is_stamped_publisher = None
            error_msg = f"Failed to create Twist publisher for '{topic_name}': {exc}"
            self._node.get_logger().error(error_msg)
            
            # If this is a type conflict, suggest a solution
            if "incompatible type" in str(exc):
                self._node.get_logger().error(
                    f"Topic '{topic_name}' already exists with incompatible message type. "
                    "Try using a different topic name or restart the ROS node."
                )
            return

        # Resume timer if needed
        if was_active and self._settings.enabled and self._publisher:
            self._timer.start()

    def _make_twist(self) -> Twist:
        t = Twist()
        t.linear.x = float(self._twist.linear.x)
        t.linear.y = float(self._twist.linear.y)
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(self._twist.angular.z)
        return t

    def _make_twist_stamped(self) -> TwistStamped:
        s = TwistStamped()
        s.header.stamp = self._node.get_clock().now().to_msg()
        s.twist = self._make_twist()
        return s

    def _on_timeout(self) -> None:
        if not self._settings.enabled or not self._publisher:
            return
        try:
            if self._is_stamped_publisher:
                msg = self._make_twist_stamped()
            else:
                msg = self._make_twist()
            self._publisher.publish(msg)
        except Exception as exc:
            self._node.get_logger().error(f"Failed to publish Twist: {exc}")
