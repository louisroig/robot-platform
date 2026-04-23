"""Unit tests for motor_driver.

Covers:
  - TEST-HAL-001: skid-steer kinematics (SRS-HAL-001-F01)
  - TEST-HAL-011: NaN/Inf Twist rejection (SRS-HAL-001-F05)
  - TEST-HAL-012: velocity clipping to configured maxima (SRS-HAL-001-F02)
  - SR-008 / SRS-HAL-001-S02: hold zero until first valid command

TEST-HAL-009 (command-timeout safe-halt) is the launch_testing integration
test and lives in test_hal_009.py; this file only covers callback-level
behavior of the node.
"""

from __future__ import annotations

import math

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.parameter import Parameter

from platform_hal.motor_driver import MotorDriver, _skid_steer, _twist_is_finite


@pytest.fixture
def mock_driver():
    """A MotorDriver with gpio_backend=mock, in its own isolated rclpy Context.

    Per-test context isolation keeps parameter state, publishers, and timers
    from leaking between tests even though every test constructs a node with
    the same node name.
    """
    context = Context()
    rclpy.init(context=context)
    overrides = [Parameter('gpio_backend', Parameter.Type.STRING, 'mock')]
    node = MotorDriver(context=context, parameter_overrides=overrides)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


# ---------------------------------------------------------------------------
# Pure-function tests (no rclpy context needed for the logic itself, but the
# module-scoped autouse fixture above keeps rclpy available for the node tests
# in the same file).
# ---------------------------------------------------------------------------

class TestSkidSteerKinematics:
    """TEST-HAL-001 — SRS-HAL-001-F01."""

    def test_pure_forward(self):
        left, right = _skid_steer(v=0.5, w=0.0, track_width=0.28)
        assert left == pytest.approx(0.5)
        assert right == pytest.approx(0.5)

    def test_pure_rotation_in_place(self):
        # Positive ω = CCW (left track reverses, right track goes forward).
        left, right = _skid_steer(v=0.0, w=1.0, track_width=0.28)
        assert left == pytest.approx(-0.14)
        assert right == pytest.approx(0.14)

    def test_mixed_forward_and_turn(self):
        # v=0.5 m/s, ω=1.0 rad/s, track=0.28 m → half-track = 0.14 m.
        left, right = _skid_steer(v=0.5, w=1.0, track_width=0.28)
        assert left == pytest.approx(0.36)
        assert right == pytest.approx(0.64)

    def test_zero_twist(self):
        left, right = _skid_steer(v=0.0, w=0.0, track_width=0.28)
        assert left == 0.0
        assert right == 0.0

    def test_reverse_motion(self):
        left, right = _skid_steer(v=-0.3, w=0.0, track_width=0.28)
        assert left == pytest.approx(-0.3)
        assert right == pytest.approx(-0.3)


class TestTwistFiniteCheck:
    """TEST-HAL-011 helper — SRS-HAL-001-F05."""

    def _twist(self, **kwargs) -> Twist:
        t = Twist()
        for k, v in kwargs.items():
            component, axis = k.split('_')  # e.g. 'linear_x'
            setattr(getattr(t, component), axis, v)
        return t

    def test_all_zero_is_finite(self):
        assert _twist_is_finite(Twist())

    def test_nan_in_linear_x_rejected(self):
        assert not _twist_is_finite(self._twist(linear_x=math.nan))

    def test_nan_in_angular_z_rejected(self):
        assert not _twist_is_finite(self._twist(angular_z=math.nan))

    def test_positive_inf_rejected(self):
        assert not _twist_is_finite(self._twist(linear_x=math.inf))

    def test_negative_inf_rejected(self):
        assert not _twist_is_finite(self._twist(angular_z=-math.inf))


# ---------------------------------------------------------------------------
# Callback-level tests — full node construction with the mock backend.
# ---------------------------------------------------------------------------

class TestCmdVelCallback:
    """TEST-HAL-001/011/012 and SR-008 at the _on_cmd_vel callback boundary."""

    def test_startup_holds_zero_until_first_command(self, mock_driver):
        # SR-008: before any command arrives, _last_cmd_time is None and
        # targets are zero. The control loop (not exercised here) turns this
        # state into a zero write to the backend.
        assert mock_driver._last_cmd_time is None
        assert mock_driver._target_left == 0.0
        assert mock_driver._target_right == 0.0

    def test_valid_twist_updates_targets(self, mock_driver):
        # TEST-HAL-001: v=0.3 m/s, ω=0.5 rad/s, default track=0.28 m.
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.5
        mock_driver._on_cmd_vel(msg)
        assert mock_driver._target_left == pytest.approx(0.3 - 0.5 * 0.14)
        assert mock_driver._target_right == pytest.approx(0.3 + 0.5 * 0.14)
        assert mock_driver._last_cmd_time is not None
        assert mock_driver._n_cmds_received == 1

    def test_nan_twist_rejected_and_state_reset(self, mock_driver):
        # Prime with a valid command so _last_cmd_time is non-None.
        good = Twist(); good.linear.x = 0.2
        mock_driver._on_cmd_vel(good)
        assert mock_driver._last_cmd_time is not None

        # TEST-HAL-011: NaN Twist rejected → targets zeroed, last_cmd_time
        # reset to startup-equivalent (None), rejection counter incremented.
        bad = Twist(); bad.linear.x = math.nan
        mock_driver._on_cmd_vel(bad)
        assert mock_driver._target_left == 0.0
        assert mock_driver._target_right == 0.0
        assert mock_driver._last_cmd_time is None
        assert mock_driver._n_cmds_rejected_nonfinite == 1

    def test_inf_twist_rejected(self, mock_driver):
        bad = Twist(); bad.angular.z = math.inf
        mock_driver._on_cmd_vel(bad)
        assert mock_driver._n_cmds_rejected_nonfinite == 1
        assert mock_driver._target_left == 0.0
        assert mock_driver._target_right == 0.0

    def test_linear_velocity_clipped_to_max(self, mock_driver):
        # TEST-HAL-012: request v > max_linear_vel (default 0.7 m/s).
        msg = Twist(); msg.linear.x = 5.0
        mock_driver._on_cmd_vel(msg)
        # Targets are skid-steer of clipped (v=0.7, ω=0) → (0.7, 0.7).
        assert mock_driver._target_left == pytest.approx(0.7)
        assert mock_driver._target_right == pytest.approx(0.7)
        assert mock_driver._n_cmds_clipped == 1

    def test_negative_linear_velocity_clipped(self, mock_driver):
        msg = Twist(); msg.linear.x = -5.0
        mock_driver._on_cmd_vel(msg)
        assert mock_driver._target_left == pytest.approx(-0.7)
        assert mock_driver._target_right == pytest.approx(-0.7)
        assert mock_driver._n_cmds_clipped == 1

    def test_angular_velocity_clipped_to_max(self, mock_driver):
        # max_angular_vel default 1.5 rad/s.
        msg = Twist(); msg.angular.z = 10.0
        mock_driver._on_cmd_vel(msg)
        # Clipped ω=1.5, v=0 → (-0.21, 0.21).
        assert mock_driver._target_left == pytest.approx(-1.5 * 0.14)
        assert mock_driver._target_right == pytest.approx(1.5 * 0.14)
        assert mock_driver._n_cmds_clipped == 1

    def test_within_limits_does_not_increment_clip_counter(self, mock_driver):
        msg = Twist(); msg.linear.x = 0.5; msg.angular.z = 1.0
        mock_driver._on_cmd_vel(msg)
        assert mock_driver._n_cmds_clipped == 0
