
from typing import List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from python_qt_binding.QtCore import QObject, QTimer, pyqtSignal

from sensor_msgs.msg import Joy

from .config_manager import ConfigurationManager


DEFAULT_BUTTON_COUNT = 12


class JoyPublisherService(QObject):
    """
    Service class for publishing Joy messages.
    It manages topic creation, message publishing, and QoS configuration.
    
    Responsibilities:
    - Create and manage ROS publisher
    - Build and publish Joy messages
    - Handle topic changes
    - Manage publishing timer
    
    Signals:
        publisher_error: Emitted when publisher creation fails
        message_published: Emitted when a message is successfully published
    """
    
    # Signals
    publisher_error = pyqtSignal(str)
    message_published = pyqtSignal()
    
    def __init__(self, ros_node: Node, config_manager: ConfigurationManager):
        """
        Initialize the Joy publisher service.
        
        Args:
            ros_node: ROS 2 node for creating publishers
            config_manager: Configuration manager for settings
        """
        super().__init__()
        
        self._node = ros_node
        self._config_manager = config_manager
        self._publisher: Optional[rclpy.publisher.Publisher] = None
        self._enabled = self._config_manager.is_publish_enabled()
        
        # Current joystick state
        self._current_axes = [0.0] * 6  # 6 standard axes
        self._current_buttons: List[int] = [0] * DEFAULT_BUTTON_COUNT
        
        # Publishing timer
        self._publish_timer = QTimer(self)
        self._publish_timer.timeout.connect(self._publish_joy_message)
        
        # Connect to configuration changes
        self._connect_config_signals()
        
        # Initialize publisher with current configuration
        self._create_publisher()
        self._update_timer_rate()
        
    def _connect_config_signals(self):
        """Connect to configuration manager signals."""
        self._config_manager.topic_changed.connect(self._on_topic_changed)
        self._config_manager.rate_changed.connect(self._on_rate_changed)
        self._config_manager.publish_enabled_changed.connect(self._on_enabled_changed)
        
    def _dispose_publisher(self, publisher: Optional[rclpy.publisher.Publisher]) -> None:
        """Destroy a ROS publisher while swallowing cleanup errors."""
        if publisher is None:
            return

        try:
            self._node.destroy_publisher(publisher)
        except Exception as exc:
            self._node.get_logger().warn(
                f"Failed to destroy Joy publisher instance: {exc}"
            )

    def _create_publisher(self) -> bool:
        """
        Create or recreate the ROS publisher.

        Returns:
            True if publisher was created successfully, False otherwise
        """
        # Get current topic name
        topic_name = self._config_manager.get_topic_name()

        # Create QoS profile for reliable delivery
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        try:
            new_publisher = self._node.create_publisher(
                Joy,
                topic_name,
                qos_profile
            )
        except Exception as exc:
            error_msg = f"Failed to create publisher for topic '{topic_name}': {exc}"
            self._node.get_logger().error(error_msg)
            self.publisher_error.emit(error_msg)
            return False

        old_publisher = self._publisher
        self._publisher = new_publisher
        self._node.get_logger().info(f"Created Joy publisher for topic: {topic_name}")

        if old_publisher is not None and old_publisher is not new_publisher:
            self._dispose_publisher(old_publisher)

        return True
            
    def _update_timer_rate(self):
        """Update the publishing timer rate based on configuration."""
        rate_hz = self._config_manager.get_publish_rate()
        interval_ms = int(1000.0 / rate_hz)

        if self._publish_timer.isActive():
            self._publish_timer.stop()

        self._publish_timer.setInterval(interval_ms)

        # Restart timer to ensure continuous publishing at the configured rate
        if self._publisher and self._enabled:
            self._publish_timer.start()
        
    def _publish_joy_message(self):
        """Publish the current joystick state as a Joy message."""
        if not self._publisher or not self._enabled:
            return
            
        try:
            # Create Joy message
            joy_msg = Joy()
            joy_msg.header.stamp = self._node.get_clock().now().to_msg()
            joy_msg.header.frame_id = ""  # No frame needed for virtual joystick
            
            # Set axes (copy to avoid reference issues)
            joy_msg.axes = list(self._current_axes)
            
            # Set buttons (copy to avoid reference issues)
            joy_msg.buttons = list(self._current_buttons)
            
            # Publish message
            self._publisher.publish(joy_msg)
            self.message_published.emit()
            
        except Exception as e:
            error_msg = f"Failed to publish Joy message: {str(e)}"
            self._node.get_logger().error(error_msg)
            self.publisher_error.emit(error_msg)
            
    def update_axes(self, x: float, y: float, additional_axes: Optional[List[float]] = None):
        """
        Update joystick axes values.
        
        Args:
            x: X-axis value (-1.0 to 1.0)
            y: Y-axis value (-1.0 to 1.0)  
            additional_axes: Additional axis values for complex joysticks
        """
        # Update primary axes
        self._current_axes[0] = max(-1.0, min(1.0, x))
        self._current_axes[1] = max(-1.0, min(1.0, y))
        
        # Update additional axes if provided
        if additional_axes:
            for i, value in enumerate(additional_axes):
                if i + 2 < len(self._current_axes):
                    self._current_axes[i + 2] = max(-1.0, min(1.0, value))
                    
        # Start publishing if not already active
        if self._enabled and not self._publish_timer.isActive() and self._publisher:
            self._publish_timer.start()
            
    def update_button(self, button_index: int, pressed: bool):
        """
        Update a specific button state.
        
        Args:
            button_index: Index of the button (0-based)
            pressed: True if button is pressed, False if released
        """
        if button_index < 0:
            return

        if button_index >= len(self._current_buttons):
            self._current_buttons.extend([0] * (button_index + 1 - len(self._current_buttons)))

        self._current_buttons[button_index] = 1 if pressed else 0

        if self._enabled and not self._publish_timer.isActive() and self._publisher:
            self._publish_timer.start()
                
    def start_publishing(self):
        """Start continuous publishing."""
        if self._enabled and self._publisher and not self._publish_timer.isActive():
            self._publish_timer.start()
            
    def stop_publishing(self):
        """Stop continuous publishing."""
        if self._publish_timer.isActive():
            self._publish_timer.stop()

    def is_publishing(self) -> bool:
        """Check if currently publishing."""
        return self._publish_timer.isActive()
        
    def _on_enabled_changed(self, enabled: bool):
        self._enabled = enabled
        if not enabled:
            if self._publish_timer.isActive():
                self._publish_timer.stop()
        else:
            # Publish current state immediately then resume timer
            self._publish_joy_message()
            if self._publisher and not self._publish_timer.isActive():
                self._publish_timer.start()

    def get_current_topic(self) -> str:
        """Get the current topic name."""
        return self._config_manager.get_topic_name()
        
    def get_publish_rate(self) -> float:
        """Get the current publish rate."""
        return self._config_manager.get_publish_rate()
        
    # Configuration change handlers
    def _on_topic_changed(self, new_topic: str):
        """Handle topic name changes."""
        was_publishing = self._publish_timer.isActive()
        
        if was_publishing:
            self._publish_timer.stop()
            
        # Recreate publisher with new topic
        if self._create_publisher() and was_publishing:
            self._publish_timer.start()
            
    def _on_rate_changed(self, new_rate: float):
        """Handle publish rate changes."""
        self._update_timer_rate()
        
    def shutdown(self):
        """Clean shutdown of the publisher service."""
        # Stop timer
        if self._publish_timer.isActive():
            self._publish_timer.stop()

        # Clean up publisher
        if self._publisher is not None:
            self._dispose_publisher(self._publisher)
            self._publisher = None

        self._node.get_logger().info("Joy publisher service shut down")
