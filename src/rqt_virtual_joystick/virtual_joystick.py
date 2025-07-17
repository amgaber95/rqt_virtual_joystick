
from qt_gui.plugin import Plugin
from .joystick_main_widget import JoystickMainWidget

class VirtualJoystick(Plugin):
    """
    RQT Plugin for Virtual Joystick.
    This class serves as the main entry point for the RQT plugin system.
    
    Responsibilities:
    - Plugin initialization and shutdown
    - Settings persistence (save/restore)
    - Integration with RQT framework
    """

    def __init__(self, context):
        """
        Initialize the Virtual Joystick plugin.
        
        Args:
            context: RQT plugin context containing node and UI management
        """
        super().__init__(context)
        self.setObjectName('VirtualJoystick')
        
        # Create the main widget - this encapsulates all plugin functionality
        self._widget = JoystickMainWidget(context.node)
        
        # Add widget to RQT interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        """
        Clean shutdown of the plugin.
        
        This method is called when the plugin is being closed.
        It ensures proper cleanup of resources.
        """
        if hasattr(self, '_widget'):
            self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        """
        Save plugin instance settings.
        
        This method is called by RQT to persist plugin configuration
        between sessions. It delegates to the main widget which knows
        about all configurable parameters.
        
        Args:
            plugin_settings: Global plugin settings (not used)
            instance_settings: Instance-specific settings to save to
        """
        if hasattr(self, '_widget'):
            self._widget.save_settings(instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        """
        Restore plugin instance settings.
        
        This method is called by RQT to restore plugin configuration
        from previous sessions. It delegates to the main widget which
        manages all configuration.
        
        Args:
            plugin_settings: Global plugin settings (not used)
            instance_settings: Instance-specific settings to restore from
        """
        if hasattr(self, '_widget'):
            self._widget.restore_settings(instance_settings)
