#!/usr/bin/env python3
"""Qt window to drive the rollchain model: one slider per rod, gravity and a tip payload.

Rod tensions are published on /rod_tensions. Gravity and the payload are
parameters of the state publisher (gravity.enabled, payload.mass); this window
sets them through its parameter service, so they stay in sync with
rqt_reconfigure and `ros2 param`. The status line shows the result of the last
solve from /rod_state. Styled like joint_state_publisher_gui, as in lopsy.

Parameters
  rods.name         rod names (from the robot config; required)
  rods.section      section of each rod (from the robot config; optional, used for grouping)
  initial_tensions  one starting tension per rod [N] (optional)
  max_tension       tension slider range [N] (default 60.0)
  max_payload       payload slider range [kg] (default 0.05)
  state_publisher   name of the state publisher node (default rollchain_state_publisher)
  publish_rate      [Hz] (default 20.0)
"""

import signal
import sys

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QCheckBox,
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
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from rollchain_interfaces.msg import RodState, RodTensions

GRAVITY = 'gravity.enabled'
PAYLOAD = 'payload.mass'

STYLE_OK = 'color: palette(text);'
STYLE_WARN = 'color: #b26a00; font-weight: bold;'
STYLE_ERROR = 'color: #c62828; font-weight: bold;'


def bold(label):
    font = label.font()
    font.setBold(True)
    label.setFont(font)
    return label


class SliderControl(QWidget):
    """A bold name and a numeric box on top, a slider underneath."""

    def __init__(self, name, initial, max_value, unit, resolution, parent=None):
        super().__init__(parent)
        self.resolution = resolution

        label = bold(QLabel(name))

        self.spin = QDoubleSpinBox()
        self.spin.setRange(0.0, max_value)
        self.spin.setSingleStep(resolution)
        self.spin.setDecimals(1)
        self.spin.setSuffix(f' {unit}')
        self.spin.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.spin.setFixedWidth(100)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(round(max_value / resolution)))

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

    def set_value(self, value):
        self.spin.setValue(value)  # clamps to the range
        self._on_spin(self.spin.value())

    def set_value_silently(self, value):
        """Set the value without emitting valueChanged (for values read back from the node)."""
        self.spin.blockSignals(True)
        self.set_value(value)
        self.spin.blockSignals(False)

    def _on_slider(self, tick):
        # Through the box, so that valueChanged reaches the box's listeners.
        self.spin.setValue(tick * self.resolution)

    def _on_spin(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(value / self.resolution)))
        self.slider.blockSignals(False)


class RollchainWindow(QWidget):
    """Loads on top, then one slider per rod grouped by section, a status line and Zero tensions."""

    def __init__(self, rods, max_tension, max_payload_g):
        super().__init__()
        self.setWindowTitle('Rollchain')
        self.setMinimumWidth(440)

        # ---- Loads -----------------------------------------------------------
        self.gravity = QCheckBox('Gravity in the bending plane')
        self.gravity.setToolTip('Weight of the links and of the payload, in the direction set by gravity.psi0 '
                                '(0 upright, pi hanging). Off: the robot lies on a table.')
        self.payload = SliderControl('Tip payload', 0.0, max_payload_g, 'g', 0.5)
        self.payload.setToolTip('Mass carried at the tip. Its weight acts only with gravity on.')

        # ---- Rod tensions ------------------------------------------------------
        self.controls = []
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setSpacing(14)
        current_section = object()
        for name, section, initial in rods:
            if section is not None and section != current_section:
                content_layout.addWidget(bold(QLabel(f'Section {section}')))
                current_section = section
            control = SliderControl(f'Rod {name}', initial, max_tension, 'N', 0.1)
            content_layout.addWidget(control)
            self.controls.append((name, control))
        content_layout.addStretch()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(content)

        # ---- Status and buttons --------------------------------------------------
        self.status = QLabel()
        self.status.setWordWrap(True)

        zero_button = QPushButton('Zero tensions')
        zero_button.clicked.connect(self.zero)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        layout.addWidget(bold(QLabel('Loads')))
        layout.addWidget(self.gravity)
        layout.addWidget(self.payload)
        layout.addWidget(separator())
        layout.addWidget(scroll, 1)
        layout.addWidget(separator())
        layout.addWidget(self.status)
        layout.addWidget(zero_button)
        self.resize(460, min(300 + 70 * len(self.controls), 860))

    def tensions(self):
        return [(name, control.value()) for name, control in self.controls]

    def zero(self):
        for _, control in self.controls:
            control.set_value(0.0)

    def set_status(self, text, style=STYLE_OK):
        self.status.setText(text)
        self.status.setStyleSheet(style)


def separator():
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setFrameShadow(QFrame.Shadow.Sunken)
    return line


class RollchainGui(Node):
    """Publishes the tensions, sets the load parameters and reads the solver status."""

    def __init__(self):
        super().__init__('tension_slider_gui')
        names = self.declare_parameter('rods.name', Parameter.Type.STRING_ARRAY).value
        sections = self.declare_parameter('rods.section', Parameter.Type.INTEGER_ARRAY).value
        initial = self.declare_parameter('initial_tensions', Parameter.Type.DOUBLE_ARRAY).value
        self.max_tension = float(self.declare_parameter('max_tension', 60.0).value)
        self.max_payload = float(self.declare_parameter('max_payload', 0.05).value)
        self.target = self.declare_parameter('state_publisher', 'rollchain_state_publisher').value
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
        self.set_client = self.create_client(SetParameters, f'{self.target}/set_parameters')
        self.get_client = self.create_client(GetParameters, f'{self.target}/get_parameters')
        self.rod_state = None
        self.create_subscription(RodState, 'rod_state', self._on_rod_state, 10)

        self.window = None
        self.synced = False
        self.sync_future = None
        self.set_futures = []      # (future, names)
        self.payload_dirty = False
        self.error = None          # last rejected parameter change

    # ---- Window wiring ------------------------------------------------------

    def attach(self, window):
        self.window = window
        window.gravity.toggled.connect(self._on_gravity)
        window.payload.spin.valueChanged.connect(self._on_payload)
        self._set_load_controls_enabled(False)

    def _set_load_controls_enabled(self, enabled):
        self.window.gravity.setEnabled(enabled)
        self.window.payload.setEnabled(enabled and self.window.gravity.isChecked())

    def _on_gravity(self, checked):
        self.window.payload.setEnabled(checked)
        self.error = None
        self._set_parameters([Parameter(GRAVITY, Parameter.Type.BOOL, bool(checked))])

    def _on_payload(self, _grams):
        self.payload_dirty = True  # sent from tick(), at most one request in flight

    # ---- Parameter service ---------------------------------------------------

    def _set_parameters(self, parameters):
        if not self.set_client.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = [p.to_parameter_msg() for p in parameters]
        self.set_futures.append((self.set_client.call_async(request), [p.name for p in parameters]))

    def _request_sync(self):
        if self.sync_future is None and self.get_client.service_is_ready():
            request = GetParameters.Request()
            request.names = [GRAVITY, PAYLOAD]
            self.sync_future = self.get_client.call_async(request)

    def _finish_sync(self):
        values = [parameter_value_to_python(v) for v in self.sync_future.result().values]
        self.sync_future = None
        if len(values) != 2:
            return
        gravity, payload = values
        self.window.gravity.blockSignals(True)
        self.window.gravity.setChecked(bool(gravity))
        self.window.gravity.blockSignals(False)
        self.window.payload.set_value_silently(1000.0 * float(payload or 0.0))
        self.synced = True
        self._set_load_controls_enabled(True)

    def _payload_in_flight(self):
        return any(PAYLOAD in names for _, names in self.set_futures)

    # ---- Periodic work (Qt timer) ------------------------------------------------

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.0)

        if not self.synced:
            self._request_sync()
        if self.sync_future is not None and self.sync_future.done():
            self._finish_sync()

        for future, names in list(self.set_futures):
            if not future.done():
                continue
            self.set_futures.remove((future, names))
            failed = [r.reason for r in future.result().results if not r.successful]
            if failed:
                self.error = f'{", ".join(names)} rejected: {failed[0]}'
                self.synced = False  # read the node's values back into the widgets
                self._set_load_controls_enabled(False)

        if self.synced and self.payload_dirty and not self._payload_in_flight():
            self.payload_dirty = False
            kg = self.window.payload.value() / 1000.0
            self._set_parameters([Parameter(PAYLOAD, Parameter.Type.DOUBLE, kg)])

        self._publish_tensions()
        self._update_status()

    def _publish_tensions(self):
        msg = RodTensions()
        msg.header.stamp = self.get_clock().now().to_msg()
        values = self.window.tensions()
        msg.name = [name for name, _ in values]
        msg.tension = [float(value) for _, value in values]
        self.publisher.publish(msg)

    def _on_rod_state(self, msg):
        self.rod_state = msg

    def _update_status(self):
        w = self.window
        if not self.set_client.service_is_ready():
            w.set_status(f'Waiting for {self.target}…', STYLE_WARN)
            return
        if self.error:
            w.set_status(self.error, STYLE_ERROR)
            return
        s = self.rod_state
        if s is None:
            w.set_status('Connected.')
        elif not s.converged:
            w.set_status('The solver did not converge: RViz shows the last converged shape.', STYLE_ERROR)
        elif not s.stable:
            w.set_status('Unstable equilibrium: the real robot would buckle under this load.', STYLE_WARN)
        elif s.joints_at_stop:
            joints = ', '.join(str(j) for j in s.joints_at_stop[:6])
            more = '…' if len(s.joints_at_stop) > 6 else ''
            w.set_status(f'Joints at their limit: {joints}{more}. The model is outside its range there.',
                         STYLE_WARN)
        else:
            w.set_status(f'OK. Peak rod strain {100.0 * s.max_strain:.2f} %.')


def main():
    # Handle signals ourselves so Ctrl-C / launch shutdown closes the window cleanly.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = RollchainGui()
    except RuntimeError as error:
        print(f'tension_slider_gui: {error}', file=sys.stderr)
        rclpy.shutdown()
        return 1

    app = QApplication(sys.argv)
    window = RollchainWindow(node.rods, node.max_tension, 1000.0 * node.max_payload)
    node.attach(window)

    # The timer also hands control back to Python regularly,
    # which is what lets the signal handlers below run while Qt is in its event loop.
    timer = QTimer()
    timer.timeout.connect(node.tick)
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
