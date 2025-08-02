import math
from typing import Dict, Set
from python_qt_binding.QtCore import Qt, pyqtSignal, QSize, QPoint
from python_qt_binding.QtGui import QPainter, QPen, QBrush, QColor, QRadialGradient, QFont
from python_qt_binding.QtWidgets import QWidget, QGridLayout


class ControllerButton(QWidget):
    """A single controller button with 3D styling matching the joystick."""
    
    pressed = pyqtSignal(int, bool)  # Emits button index and state when pressed/released
    
    def __init__(self, name, button_index, color, parent=None):
        super().__init__(parent)
        self.name = name
        self.button_index = button_index
        self.base_color = color
        self.is_pressed = False
        self._sticky = False
        # Make buttons slightly flexible in size
        self.setMinimumSize(40, 40)
        self.setMaximumSize(60, 60)
        self.resize(60, 60)  # Default size
        self.setMouseTracking(True)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        center = QPoint(self.width() // 2, self.height() // 2)
        radius = min(self.width(), self.height()) // 2 - 2
        
        self._draw_button_shadow(painter, center, radius)
        self._draw_button_body(painter, center, radius)
        self._draw_button_highlight(painter, center, radius)
        self._draw_button_text(painter, center)
        
    def _draw_button_shadow(self, painter, center, radius):
        """Draw button shadow for 3D effect."""
        if not self.is_pressed:
            shadow_offset = 2
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 0, 0, 60)))
            shadow_center = QPoint(center.x() + shadow_offset, center.y() + shadow_offset)
            painter.drawEllipse(shadow_center, radius + 1, radius + 1)
    
    def _draw_button_body(self, painter, center, radius):
        """Draw the main button body with gradient."""
        # Create radial gradient for 3D effect
        if self.is_pressed:
            # Pressed state - darker and flatter
            gradient = QRadialGradient(center.x() + 2, center.y() + 2, radius)
            gradient.setColorAt(0.0, self.base_color.darker(130))
            gradient.setColorAt(0.7, self.base_color.darker(110))
            gradient.setColorAt(1.0, self.base_color.darker(150))
            border_color = self.base_color.darker(180)
        else:
            # Normal state - lighter with highlight
            gradient = QRadialGradient(center.x() - 3, center.y() - 3, radius)
            gradient.setColorAt(0.0, self.base_color.lighter(150))
            gradient.setColorAt(0.5, self.base_color)
            gradient.setColorAt(1.0, self.base_color.darker(120))
            border_color = self.base_color.darker(150)
        
        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(border_color, 2))
        
        # Adjust center slightly when pressed for pressed effect
        draw_center = center
        if self.is_pressed:
            draw_center = QPoint(center.x() + 1, center.y() + 1)
            
        painter.drawEllipse(draw_center, radius, radius)
    
    def _draw_button_highlight(self, painter, center, radius):
        """Draw button highlight for extra 3D effect."""
        if not self.is_pressed:
            highlight_radius = radius - 6
            if highlight_radius > 0:
                painter.setBrush(QBrush(QColor(255, 255, 255, 80)))
                painter.setPen(Qt.NoPen)
                highlight_center = QPoint(center.x() - 2, center.y() - 2)
                painter.drawEllipse(highlight_center, highlight_radius, highlight_radius)
    
    def _draw_button_text(self, painter, center):
        """Draw the button text/label."""
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setBrush(Qt.NoBrush)
        
        font = QFont()
        font.setPointSize(15)
        font.setBold(True)
        painter.setFont(font)
        
        # Adjust text position when pressed
        text_center = center
        if self.is_pressed:
            text_center = QPoint(center.x() + 1, center.y() + 1)
            
        # Center the text
        rect = painter.fontMetrics().boundingRect(self.name)
        text_pos = QPoint(text_center.x() - rect.width() // 2, 
                         text_center.y() + rect.height() // 2 - 2)
        painter.drawText(text_pos, self.name)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self._sticky:
                self.is_pressed = not self.is_pressed
                self.update()
                self.pressed.emit(self.button_index, self.is_pressed)
            else:
                self.is_pressed = True
                self.update()
                self.pressed.emit(self.button_index, True)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and not self._sticky and self.is_pressed:
            self.is_pressed = False
            self.update()
            self.pressed.emit(self.button_index, False)
    
    def reset_position(self):
        """Reset button to unpressed state."""
        if self.is_pressed:
            self.is_pressed = False
            self.update()

    def set_sticky(self, enabled: bool):
        self._sticky = enabled
        if not enabled and self.is_pressed:
            # When disabling sticky, ensure button returns to neutral.
            self.is_pressed = False
            self.update()


class ControllerButtonsWidget(QWidget):
    """Widget containing controller buttons (X, Y, A, B) arranged in standard layout with 3D styling."""
    
    button_pressed = pyqtSignal(int, bool)  # Emits button index and state
    
    # Standard controller button indices (Joy message mapping)
    BUTTON_A = 0
    BUTTON_B = 1 
    BUTTON_X = 2
    BUTTON_Y = 3
    
    def __init__(self, parent=None, sticky_buttons: bool = False):
        super().__init__(parent)
        # Set a reasonable size range
        self.setMinimumSize(200, 200)
        self._pressed_buttons: Set[int] = set()
        self._buttons: Dict[str, ControllerButton] = {}
        self._sticky_buttons = sticky_buttons
        self._init_ui()
        self.set_sticky_buttons(sticky_buttons)
        
    def _init_ui(self):
        """Initialize the UI with grid layout."""
        # Create grid layout for flexible button arrangement
        layout = QGridLayout(self)
        layout.setSpacing(2)  # Space between buttons
        layout.setContentsMargins(8, 8, 8, 8)  # Margins around the grid
        
        # Create buttons with their colors and indices
        self.y_button = ControllerButton("Y", self.BUTTON_Y, QColor(255, 220, 100), self)  # Yellow
        self.x_button = ControllerButton("X", self.BUTTON_X, QColor(100, 150, 255), self)  # Blue  
        self.b_button = ControllerButton("B", self.BUTTON_B, QColor(255, 100, 100), self)  # Red
        self.a_button = ControllerButton("A", self.BUTTON_A, QColor(120, 255, 120), self)  # Green
        
        # Store buttons in dictionary for easy access
        self._buttons = {
            'Y': self.y_button,
            'X': self.x_button, 
            'B': self.b_button,
            'A': self.a_button
        }
        
        # Arrange buttons in cross pattern using grid layout
        # Grid positions:    0   1   2
        #                0       Y      
        #                1   X       B  
        #                2       A      
        
        layout.addWidget(self.y_button, 0, 1, Qt.AlignCenter)  # Top center
        layout.addWidget(self.x_button, 1, 0, Qt.AlignCenter)  # Middle left
        layout.addWidget(self.b_button, 1, 2, Qt.AlignCenter)  # Middle right
        layout.addWidget(self.a_button, 2, 1, Qt.AlignCenter)  # Bottom center
        
        # Make all grid cells equal size and expand to fill space
        for i in range(3):
            layout.setRowStretch(i, 1)
            layout.setColumnStretch(i, 1)
        
        # Connect button signals
        for button in self._buttons.values():
            button.pressed.connect(self._on_button_state_changed)
    
    def _on_button_state_changed(self, button_index: int, pressed: bool):
        """Handle button state change."""
        if pressed:
            self._pressed_buttons.add(button_index)
        else:
            self._pressed_buttons.discard(button_index)
        
        self.button_pressed.emit(button_index, pressed)
    
    def paintEvent(self, event):
        """Paint a circular background similar to the joystick styling."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Calculate circular background similar to joystick
        width = self.width()
        height = self.height()
        size = min(width, height)
        center_x = width // 2
        center_y = height // 2
        radius = size // 2 - 10  # Leave some margin
        
        # Draw shadow first (similar to joystick shadow)
        shadow_offset = 3
        shadow_radius = radius + shadow_offset
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 80)))
        painter.drawEllipse(
            center_x - shadow_radius + shadow_offset,
            center_y - shadow_radius + shadow_offset,
            shadow_radius * 2,
            shadow_radius * 2
        )
        
        # Main background gradient (matching joystick outer circle)
        gradient = QRadialGradient(center_x, center_y, radius)
        gradient.setColorAt(0.0, QColor(60, 60, 60))
        gradient.setColorAt(0.7, QColor(45, 45, 45))
        gradient.setColorAt(1.0, QColor(30, 30, 30))
        
        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
        
        # Add highlight ring (similar to joystick highlight)
        highlight_radius = radius - 5
        painter.setPen(QPen(QColor(100, 100, 100, 100), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(
            center_x - highlight_radius,
            center_y - highlight_radius,
            highlight_radius * 2,
            highlight_radius * 2
        )
        
    def get_pressed_buttons(self) -> Set[int]:
        """Get the set of currently pressed buttons."""
        return self._pressed_buttons.copy()
        
    def reset_buttons(self):
        """Reset all buttons to their unpressed state."""
        for button in self._buttons.values():
            button.reset_position()
        self._pressed_buttons.clear()
    
    def set_button_layout(self, layout_type="cross"):
        """
        Set the button layout arrangement.
        
        Args:
            layout_type (str): Layout type - "cross" (default), "diamond", "square", etc.
                              Future enhancement for different controller layouts
        """
        # For now, only cross layout is implemented
        # This method is prepared for future layout variations
        if layout_type == "cross":
            # This is already implemented in _init_ui()
            pass
        # Future layouts can be added here:
        # elif layout_type == "diamond":
        #     # Arrange in diamond pattern
        # elif layout_type == "square": 
        #     # Arrange in 2x2 grid
        pass

    def set_sticky_buttons(self, enabled: bool) -> None:
        enabled = bool(enabled)
        previously_enabled = self._sticky_buttons
        self._sticky_buttons = enabled

        for button in self._buttons.values():
            button.set_sticky(enabled)

        if not enabled and previously_enabled and self._pressed_buttons:
            for button_index in list(self._pressed_buttons):
                self.button_pressed.emit(button_index, False)
            self._pressed_buttons.clear()
