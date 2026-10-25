#!/usr/bin/env python3
"""Qt slider window publishing one tension per rod on /rod_tensions.

The rods come from the robot config (rods.name, grouped by rods.section), so
the window adapts to any number of sections. Styled like
joint_state_publisher_gui, as in lopsy.

Parameters
  rods.name         rod names (from the robot config; required)
  rods.section      section of each rod (from the robot config; optional, used for grouping)
  initial_tensions  one starting tension per rod [N] (optional)
  max_tension       slider range [N] (default 50.0)
  publish_rate      [Hz] (default 20.0)
"""

import signal
import sys

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rollchain_interfaces.msg import RodTensions

RESOLUTION = 0.1  # newtons per slider tick


def bold(label):
    font = label.font()
    font.setBold(True)
    label.setFont(font)
    return label


class TensionControl(QWidget):
    """One rod: bold name and numeric box on top, slider underneath."""

    def __init__(self, name, initial, max_value, parent=None):
        super().__init__(parent)

        label = bold(QLabel(name))

        self.spin = QDoubleSpinBox()
        self.spin.setRange(0.0, max_value)
        self.spin.setSingleStep(RESOLUTION)
        self.spin.setDecimals(1)
        self.spin.setSuffix(' N')
        self.spin.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.spin.setFixedWidth(100)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(round(max_value / RESOLUTION)))

        top = QHBoxLayout()
        top.addWidget(label)
        top.addStretch()
        top.addWidget(self.spin)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addLayout(top)
        layout.addWidget(self.slider)

        # Keep slider and box in sync without each one re-triggering the other.
        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)
        self.set_value(initial)

    def value(self):
        return self.spin.value()

    def set_value(self, newtons):
        self.spin.setValue(newtons)  # clamps to [0, max]
        self._on_spin(self.spin.value())

    def _on_slider(self, tick):
        self.spin.blockSignals(True)
        self.spin.setValue(tick * RESOLUTION)
        self.spin.blockSignals(False)

    def _on_spin(self, newtons):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(newtons / RESOLUTION)))
        self.slider.blockSignals(False)


class TensionSliderWindow(QWidget):
    """One TensionControl per rod, grouped by section, with a Zero-all button."""

    def __init__(self, rods, max_tension):
        super().__init__()
        self.setWindowTitle('Rod tensions')
        self.setMinimumWidth(420)
        self.controls = []

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setSpacing(14)
        current_section = object()
        for name, section, initial in rods:
            if section is not None and section != current_section:
                content_layout.addWidget(bold(QLabel(f'Section {section}')))
                current_section = section
            control = TensionControl(f'Rod {name}', initial, max_tension)
            content_layout.addWidget(control)
            self.controls.append((name, control))
        content_layout.addStretch()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(content)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)

        zero_button = QPushButton('Zero all')
        zero_button.clicked.connect(self.zero)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)
        layout.addWidget(scroll)
        layout.addWidget(separator)
        layout.addWidget(zero_button)
        self.resize(440, min(140 + 70 * len(self.controls), 800))

    def values(self):
        return [(name, control.value()) for name, control in self.controls]

    def zero(self):
        for _, control in self.controls:
            control.set_value(0.0)


class TensionPublisher(Node):

    def __init__(self):
        super().__init__('tension_slider_gui')
        names = self.declare_parameter('rods.name', Parameter.Type.STRING_ARRAY).value
        sections = self.declare_parameter('rods.section', Parameter.Type.INTEGER_ARRAY).value
        initial = self.declare_parameter('initial_tensions', Parameter.Type.DOUBLE_ARRAY).value
        self.max_tension = float(self.declare_parameter('max_tension', 50.0).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 20.0).value)

        names = list(names or [])
        if not names:
            raise RuntimeError('rods.name is empty: pass the robot config to this node')
        sections = list(sections) if sections else []
        if len(sections) != len(names):
            sections = [None] * len(names)
        initial = list(initial) if initial else []
        if len(initial) != len(names):
            if initial:
                self.get_logger().warning('initial_tensions must have one entry per rod; starting from 0')
            initial = [0.0] * len(names)
        self.rods = list(zip(names, sections, initial))
        self.publisher = self.create_publisher(RodTensions, 'rod_tensions', 10)

    def publish(self, values):
        msg = RodTensions()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [name for name, _ in values]
        msg.tension = [float(value) for _, value in values]
        self.publisher.publish(msg)


def main():
    # Handle signals ourselves so Ctrl-C / launch shutdown closes the window cleanly.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = TensionPublisher()
    except RuntimeError as error:
        print(f'tension_slider_gui: {error}', file=sys.stderr)
        rclpy.shutdown()
        return 1

    app = QApplication(sys.argv)
    window = TensionSliderWindow(node.rods, node.max_tension)

    # The publish timer also hands control back to Python regularly,
    # which is what lets the signal handlers below run while Qt is in its event loop.
    timer = QTimer()
    timer.timeout.connect(lambda: node.publish(window.values()))
    timer.start(max(1, int(1000.0 / node.publish_rate)))

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    window.show()
    run = getattr(app, 'exec', None) or app.exec_  # Qt5/Qt6 compatible
    try:
        run()
    finally:
        timer.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
