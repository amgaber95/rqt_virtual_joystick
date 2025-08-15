from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QToolButton, QSizePolicy


class SegmentedToggle(QWidget):
    """Two-state segmented control emitting boolean toggled signal."""

    toggled = pyqtSignal(bool)

    def __init__(self, parent=None, false_label="Disabled", true_label="Enabled"):
        super().__init__(parent)
        self._checked = True
        self._buttons = {}

        self._true_button = self._create_button(true_label, position="left", target_state=True)
        self._false_button = self._create_button(false_label, position="right", target_state=False)
        self._buttons = {False: self._false_button, True: self._true_button}

        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._true_button)
        layout.addWidget(self._false_button)
        self.setLayout(layout)

        self.setFixedHeight(28)
        self._apply_stylesheet()
        self.setChecked(True)

    def isChecked(self) -> bool:
        return self._checked

    def setChecked(self, checked: bool) -> None:
        checked = bool(checked)
        if self._checked == checked:
            self._sync_buttons()
            return

        self._checked = checked
        self._sync_buttons()
        if not self.signalsBlocked():
            self.toggled.emit(self._checked)

    def setEnabled(self, enabled: bool) -> None:
        super().setEnabled(enabled)
        for button in self._buttons.values():
            button.setEnabled(enabled)

    def sizeHint(self):
        hint = super().sizeHint()
        if hint.isValid():
            return hint
        return self._true_button.sizeHint()

    def _on_button_clicked(self, target_state: bool) -> None:
        if self._checked != target_state:
            self.setChecked(target_state)

    def _sync_buttons(self) -> None:
        # Prevent recursive signal loops when syncing button state.
        self._false_button.blockSignals(True)
        self._true_button.blockSignals(True)
        self._false_button.setChecked(not self._checked)
        self._true_button.setChecked(self._checked)
        self._false_button.blockSignals(False)
        self._true_button.blockSignals(False)
        self._update_button_states()

    def _create_button(self, label: str, position: str, target_state: bool) -> QToolButton:
        button = QToolButton(self)
        button.setText(label)
        button.setCheckable(True)
        button.setAutoExclusive(True)
        button.setCursor(Qt.PointingHandCursor)
        button.setFocusPolicy(Qt.NoFocus)
        button.setObjectName(f"segment_{position}")
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        button.clicked.connect(lambda checked, value=target_state: self._on_button_clicked(value))
        return button

    def _update_button_states(self) -> None:
        self._true_button.setProperty("active", self._checked)
        self._false_button.setProperty("active", not self._checked)

        self.style().unpolish(self._true_button)
        self.style().polish(self._true_button)
        self.style().unpolish(self._false_button)
        self.style().polish(self._false_button)

    def _apply_stylesheet(self) -> None:
        self.setStyleSheet(
            """
            QToolButton#segment_left,
            QToolButton#segment_right {
                border: 1px solid #4c4c4c;
                padding: 4px 12px;
                background-color: #2d2d2d;
                color: #f0f0f0;
            }

            QToolButton#segment_left {
                border-top-left-radius: 4px;
                border-bottom-left-radius: 4px;
                border-right: none;
            }

            QToolButton#segment_right {
                border-top-right-radius: 4px;
                border-bottom-right-radius: 4px;
                border-left: none;
            }

            QToolButton#segment_left[active="true"] {
                background-color: #3a9d5a;
                color: white;
            }

            QToolButton#segment_right[active="true"] {
                background-color: #b44646;
                color: white;
            }

            QToolButton#segment_left:disabled,
            QToolButton#segment_right:disabled {
                background-color: #3a3a3a;
                color: #a0a0a0;
                border-color: #575757;
            }
            """
        )

        self._update_button_states()
