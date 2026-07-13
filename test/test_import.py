"""
Import-level + pure-logic tests for jetank_ros_main.gripper_mimic_relay.

The relay module imports rclpy and the ROS message packages at module scope
(for the Node subclass + type hints), but none of those require a running ROS
context simply to be *imported*.  We stub rclpy / rclpy.node / rclpy.qos /
sensor_msgs / std_msgs only when the real packages are absent (bare pixi env),
mirroring jetank_web_control/test/test_labels.py.

We then exercise the pure mimic / joint-mapping logic of ``_js_callback``
without constructing a real ROS Node: an instance is built via
``object.__new__`` and the handful of attributes the callback touches are set
by hand, with a fake publisher that records what would be published.
"""

import importlib
import os
import sys
import types

import pytest


# ---------------------------------------------------------------------------
# Stub infrastructure (only used if the real packages are unavailable)
# ---------------------------------------------------------------------------

def _make_stub(name: str) -> types.ModuleType:
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


def _install_stubs():
    """Install minimal stubs for rclpy + message packages if absent."""
    if 'rclpy' not in sys.modules:
        try:
            import rclpy  # noqa: F401 — prefer the real package
        except ImportError:
            rclpy_stub = _make_stub('rclpy')
            rclpy_stub.init = lambda *a, **k: None
            rclpy_stub.spin = lambda *a, **k: None
            rclpy_stub.try_shutdown = lambda *a, **k: None

            node_stub = _make_stub('rclpy.node')
            node_stub.Node = object
            rclpy_stub.node = node_stub

            qos_stub = _make_stub('rclpy.qos')
            qos_stub.QoSProfile = type('QoSProfile', (), {})
            qos_stub.ReliabilityPolicy = type(
                'ReliabilityPolicy', (), {'RELIABLE': 1})
            qos_stub.DurabilityPolicy = type(
                'DurabilityPolicy', (), {'VOLATILE': 1})
            qos_stub.HistoryPolicy = type(
                'HistoryPolicy', (), {'KEEP_LAST': 1})
            rclpy_stub.qos = qos_stub

    for pkg in ['sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg']:
        if pkg not in sys.modules:
            try:
                importlib.import_module(pkg)
            except ImportError:
                _make_stub(pkg)

    sm = sys.modules.get('sensor_msgs.msg')
    if sm is not None and not hasattr(sm, 'JointState'):
        sm.JointState = type(
            'JointState', (), {'__init__': lambda self: None})

    std = sys.modules.get('std_msgs.msg')
    if std is not None and not hasattr(std, 'Float64MultiArray'):
        std.Float64MultiArray = type(
            'Float64MultiArray', (), {'__init__': lambda self: None})


# ---------------------------------------------------------------------------
# Import the module under test
# ---------------------------------------------------------------------------

_install_stubs()

pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if pkg_root not in sys.path:
    sys.path.insert(0, pkg_root)

try:
    relay = importlib.import_module('jetank_ros_main.gripper_mimic_relay')
except Exception as exc:  # pragma: no cover - diagnostic skip
    pytest.skip(
        f'Could not import gripper_mimic_relay: {exc}',
        allow_module_level=True,
    )


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

class _RecordingPub:
    """Stand-in publisher that records every published message."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _SilentLogger:
    def info(self, *a, **k):
        pass

    def debug(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class _FakeJointState:
    def __init__(self, name, position):
        self.name = name
        self.position = position


def _make_relay():
    """Build a GripperMimicRelay without invoking the ROS Node __init__."""
    node = relay.GripperMimicRelay.__new__(relay.GripperMimicRelay)
    node._left_joint = 'gripper_left_joint'
    node._left_idx = None
    node._last_position = -1.0
    node._cmd_pub = _RecordingPub()
    node._logger = _SilentLogger()
    node.get_logger = lambda: node._logger
    return node


# ---------------------------------------------------------------------------
# Import-level
# ---------------------------------------------------------------------------

def test_module_imports_and_exposes_symbols():
    assert relay is not None
    assert hasattr(relay, 'GripperMimicRelay')
    assert callable(relay.main)


def test_publish_topic_constant_unchanged():
    """The right-finger command topic must not drift (it is a contract)."""
    src = relay.__file__.replace('.pyc', '.py')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    assert '/gripper_right_mimic_controller/commands' in text


# ---------------------------------------------------------------------------
# Pure mimic / joint-mapping logic in _js_callback
# ---------------------------------------------------------------------------

def test_forwards_left_joint_position():
    node = _make_relay()
    msg = _FakeJointState(['other', 'gripper_left_joint'], [0.0, 0.025])
    node._js_callback(msg)

    assert len(node._cmd_pub.published) == 1
    data = node._cmd_pub.published[0].data
    assert len(data) == 1
    assert data[0] == pytest.approx(0.025)
    assert node._last_position == pytest.approx(0.025)


def test_missing_left_joint_does_not_publish():
    node = _make_relay()
    node._js_callback(_FakeJointState(['some_other_joint'], [0.1]))
    assert node._cmd_pub.published == []


def test_index_beyond_position_array_is_guarded():
    # gripper_left_joint is named but no matching position entry exists.
    node = _make_relay()
    node._js_callback(_FakeJointState(['a', 'gripper_left_joint'], [0.0]))
    assert node._cmd_pub.published == []


def test_empty_position_is_guarded():
    node = _make_relay()
    node._js_callback(_FakeJointState(['gripper_left_joint'], []))
    assert node._cmd_pub.published == []


def test_dedup_below_threshold_skips_publish():
    node = _make_relay()
    node._js_callback(_FakeJointState(['gripper_left_joint'], [0.030]))
    # A change smaller than the 1e-6 dedup threshold must be ignored.
    node._js_callback(_FakeJointState(['gripper_left_joint'], [0.0300005]))
    assert len(node._cmd_pub.published) == 1


def test_change_above_threshold_publishes_again():
    node = _make_relay()
    node._js_callback(_FakeJointState(['gripper_left_joint'], [0.030]))
    node._js_callback(_FakeJointState(['gripper_left_joint'], [0.040]))
    assert len(node._cmd_pub.published) == 2
    assert node._cmd_pub.published[-1].data[0] == pytest.approx(0.040)


def test_first_publish_fires_from_sentinel():
    # Sentinel _last_position is -1.0, so any plausible 0..0.04 value differs
    # by more than the threshold and must publish on the first message.
    node = _make_relay()
    node._js_callback(_FakeJointState(['gripper_left_joint'], [0.0]))
    assert len(node._cmd_pub.published) == 1
    assert node._cmd_pub.published[0].data[0] == pytest.approx(0.0)
