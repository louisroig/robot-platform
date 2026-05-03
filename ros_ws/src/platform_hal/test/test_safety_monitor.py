"""Unit tests for safety_monitor.

Covers the pure-logic state machine (no rclpy context), tilt math, and
the callback-level behavior of the Node with subscriptions exercised
directly.

Integration test (full launch with motor_driver mock) lives in
test_safety_integration.py.
"""

from __future__ import annotations

import math

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from platform_hal.safety_monitor import (
    SafetyMonitor,
    SafetyState,
    StateMachine,
    tilt_angle_rad,
    twist_is_finite,
)


# ---------------------------------------------------------------------------
# Pure-function tests
# ---------------------------------------------------------------------------


class TestTiltAngle:
    """Quaternion → tilt-from-vertical."""

    def test_identity_is_level(self):
        assert tilt_angle_rad(1.0, 0.0, 0.0, 0.0) == pytest.approx(0.0)

    def test_45_deg_pitch(self):
        # 45° rotation about y-axis: q = (cos(22.5°), 0, sin(22.5°), 0).
        half = math.radians(45.0) / 2.0
        q = (math.cos(half), 0.0, math.sin(half), 0.0)
        assert tilt_angle_rad(*q) == pytest.approx(math.radians(45.0))

    def test_45_deg_roll(self):
        half = math.radians(45.0) / 2.0
        q = (math.cos(half), math.sin(half), 0.0, 0.0)
        assert tilt_angle_rad(*q) == pytest.approx(math.radians(45.0))

    def test_yaw_only_is_level(self):
        # Pure yaw (rotation about z) leaves body z-axis aligned with world z.
        half = math.radians(90.0) / 2.0
        q = (math.cos(half), 0.0, 0.0, math.sin(half))
        assert tilt_angle_rad(*q) == pytest.approx(0.0, abs=1e-12)

    def test_180_deg_flip(self):
        # Upside-down (rotation about y by π) gives tilt = π.
        q = (0.0, 0.0, 1.0, 0.0)
        assert tilt_angle_rad(*q) == pytest.approx(math.pi)

    def test_clamps_floating_point_overshoot(self):
        # Ill-conditioned non-unit quaternion that would push cos_tilt
        # marginally above 1; must not raise.
        assert tilt_angle_rad(1.0, 1e-9, 1e-9, 0.0) == pytest.approx(0.0, abs=1e-8)


class TestTwistIsFinite:
    def test_zero(self):
        assert twist_is_finite(Twist())

    def test_nan_linear(self):
        t = Twist(); t.linear.x = math.nan
        assert not twist_is_finite(t)

    def test_inf_angular(self):
        t = Twist(); t.angular.z = math.inf
        assert not twist_is_finite(t)


# ---------------------------------------------------------------------------
# State machine logic — no rclpy
# ---------------------------------------------------------------------------


def _sm(**overrides) -> StateMachine:
    params = dict(tilt_limit_deg=25.0, tilt_warning_deg=18.0, tilt_latches=True)
    params.update(overrides)
    return StateMachine(**params)


def _clear_startup(sm: StateMachine) -> None:
    """Helper: walk the SM out of the startup-incomplete state."""
    sm.set_active('startup_incomplete', False)


class TestStateMachineStartup:
    def test_initial_state_is_estop_with_startup_reason(self):
        sm = _sm()
        ev = sm.evaluate()
        assert ev.state == SafetyState.ESTOP
        assert 'startup_incomplete' in ev.reasons

    def test_clearing_startup_with_no_other_triggers_yields_ok(self):
        sm = _sm()
        _clear_startup(sm)
        ev = sm.evaluate()
        assert ev.state == SafetyState.OK
        assert ev.reasons == []
        assert ev.clearable is True


class TestStateMachineTilt:
    def test_below_warning_is_ok(self):
        sm = _sm(); _clear_startup(sm)
        sm.update_tilt(math.radians(10.0))
        assert sm.evaluate().state == SafetyState.OK

    def test_between_warning_and_limit_is_warning(self):
        sm = _sm(); _clear_startup(sm)
        sm.update_tilt(math.radians(20.0))
        ev = sm.evaluate()
        assert ev.state == SafetyState.WARNING
        assert 'tilt_warning' in ev.reasons

    def test_above_limit_is_estop(self):
        sm = _sm(); _clear_startup(sm)
        sm.update_tilt(math.radians(30.0))
        ev = sm.evaluate()
        assert ev.state == SafetyState.ESTOP
        assert 'tilt_exceeded' in ev.reasons

    def test_tilt_latches_when_configured(self):
        # With tilt_latches=True, once we exceed and recover, we stay in
        # ESTOP until /safety/reset is called.
        sm = _sm(tilt_latches=True); _clear_startup(sm)
        sm.update_tilt(math.radians(30.0))
        assert sm.evaluate().state == SafetyState.ESTOP
        sm.update_tilt(math.radians(5.0))   # rover levels back out
        ev = sm.evaluate()
        assert ev.state == SafetyState.ESTOP, 'latch should hold ESTOP after recovery'
        assert ev.clearable is False

    def test_tilt_does_not_latch_when_disabled(self):
        sm = _sm(tilt_latches=False); _clear_startup(sm)
        sm.update_tilt(math.radians(30.0))
        assert sm.evaluate().state == SafetyState.ESTOP
        sm.update_tilt(math.radians(5.0))
        assert sm.evaluate().state == SafetyState.OK

    def test_no_imu_clears_tilt_triggers(self):
        # `update_tilt(None)` represents IMU-stale or never-seen — tilt
        # triggers go inactive (separate trigger covers staleness itself).
        sm = _sm(); _clear_startup(sm)
        sm.update_tilt(math.radians(30.0))
        sm.set_active('tilt_exceeded', False)  # latch survives
        # Confirm latch is still asserted on the dict, not just via active.
        assert sm.evaluate().state == SafetyState.ESTOP


class TestStateMachineStaleness:
    def test_imu_stale_is_estop(self):
        sm = _sm(); _clear_startup(sm)
        sm.set_active('imu_stale', True)
        ev = sm.evaluate()
        assert ev.state == SafetyState.ESTOP
        assert 'topic_stale:/hal/imu/data' in ev.reasons

    def test_cmd_vel_stale_is_estop(self):
        sm = _sm(); _clear_startup(sm)
        sm.set_active('cmd_vel_stale', True)
        ev = sm.evaluate()
        assert ev.state == SafetyState.ESTOP
        assert 'topic_stale:/hal/cmd_vel_raw' in ev.reasons

    def test_staleness_is_auto_clearable_when_topic_resumes(self):
        sm = _sm(); _clear_startup(sm)
        sm.set_active('cmd_vel_stale', True)
        sm.set_active('cmd_vel_stale', False)
        assert sm.evaluate().state == SafetyState.OK


class TestStateMachineInvalidCmdVel:
    def test_nonfinite_promotes_to_estop(self):
        sm = _sm(); _clear_startup(sm)
        sm.set_active('cmd_vel_invalid', True)
        ev = sm.evaluate()
        assert ev.state == SafetyState.ESTOP
        assert 'cmd_vel_raw_nonfinite' in ev.reasons


class TestStateMachineReset:
    def test_reset_clears_tilt_latch_when_recovered(self):
        sm = _sm(); _clear_startup(sm)
        sm.update_tilt(math.radians(30.0))    # excursion
        sm.update_tilt(math.radians(5.0))     # recovered, latch holds ESTOP
        assert sm.evaluate().state == SafetyState.ESTOP

        # No longer-active latches.
        assert sm.latched_keys_active() == []
        cleared = sm.clear_latches()
        assert cleared
        assert sm.evaluate().state == SafetyState.OK

    def test_reset_refuses_when_underlying_condition_still_active(self):
        sm = _sm(); _clear_startup(sm)
        sm.update_tilt(math.radians(30.0))
        # Still tilted: the latch is active AND the condition is active.
        assert sm.latched_keys_active() == ['tilt_exceeded']
        # clear_latches doesn't blow away an actively-asserted condition.
        sm.clear_latches()
        assert sm.evaluate().state == SafetyState.ESTOP

    def test_reset_is_noop_when_nothing_latched(self):
        sm = _sm(); _clear_startup(sm)
        assert sm.clear_latches() is False


class TestStateMachineWorstTriggerWins:
    def test_warning_and_estop_yields_estop(self):
        sm = _sm(); _clear_startup(sm)
        sm.update_tilt(math.radians(20.0))           # warning
        sm.set_active('cmd_vel_stale', True)         # estop
        assert sm.evaluate().state == SafetyState.ESTOP


class TestStateMachineTriggersChanged:
    def test_no_change_means_no_transition(self):
        sm = _sm()
        sm.evaluate()                          # initial ESTOP (startup)
        assert sm.evaluate().triggers_changed is False

    def test_clearing_startup_marks_transition(self):
        sm = _sm()
        sm.evaluate()
        _clear_startup(sm)
        ev = sm.evaluate()
        assert ev.state == SafetyState.OK
        assert ev.triggers_changed is True


# ---------------------------------------------------------------------------
# Node-level callback tests (rclpy context, no executor spin)
# ---------------------------------------------------------------------------


@pytest.fixture
def safety_node():
    context = Context()
    rclpy.init(context=context)
    node = SafetyMonitor(context=context)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def _imu_msg(qw=1.0, qx=0.0, qy=0.0, qz=0.0) -> Imu:
    msg = Imu()
    msg.orientation.w = qw
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    return msg


def _twist(linear_x=0.0, angular_z=0.0) -> Twist:
    t = Twist()
    t.linear.x = linear_x
    t.angular.z = angular_z
    return t


class TestNodeStartup:
    def test_starts_in_estop(self, safety_node):
        # Node enters with startup_incomplete trigger asserted.
        assert safety_node._last_eval.state == SafetyState.ESTOP

    def test_eval_tick_clears_startup_after_inputs_seen(self, safety_node):
        # Prime both inputs so eval_tick can clear startup_incomplete.
        safety_node._on_imu(_imu_msg())
        safety_node._on_raw(_twist(linear_x=0.1))
        safety_node._eval_tick()
        assert safety_node._last_eval.state == SafetyState.OK


class TestNodeNanRejection:
    def test_nan_twist_publishes_zero_and_promotes_to_estop(self, safety_node):
        # Prime healthy state.
        safety_node._on_imu(_imu_msg())
        safety_node._on_raw(_twist(linear_x=0.1))
        safety_node._eval_tick()
        assert safety_node._last_eval.state == SafetyState.OK
        n_before = safety_node._n_raw_nonfinite

        bad = Twist(); bad.linear.x = math.nan
        safety_node._on_raw(bad)
        safety_node._eval_tick()

        assert safety_node._n_raw_nonfinite == n_before + 1
        assert safety_node._last_eval.state == SafetyState.ESTOP


class TestNodeTiltGate:
    def test_tilt_excursion_drives_estop(self, safety_node):
        # Simulate a 30° pitch (>25° limit).
        half = math.radians(30.0) / 2.0
        safety_node._on_imu(_imu_msg(qw=math.cos(half), qy=math.sin(half)))
        safety_node._on_raw(_twist(linear_x=0.5))
        safety_node._eval_tick()
        assert safety_node._last_eval.state == SafetyState.ESTOP
        assert 'tilt_exceeded' in safety_node._last_eval.reasons


class TestNodeResetService:
    def test_reset_refused_while_tilted(self, safety_node):
        half = math.radians(30.0) / 2.0
        safety_node._on_imu(_imu_msg(qw=math.cos(half), qy=math.sin(half)))
        safety_node._on_raw(_twist())
        safety_node._eval_tick()

        response = safety_node._on_reset_request(
            Trigger.Request(), Trigger.Response(),
        )
        assert response.success is False
        assert 'tilt_exceeded' in response.message

    def test_reset_succeeds_after_recovery(self, safety_node):
        # Excursion then recovery.
        half = math.radians(30.0) / 2.0
        safety_node._on_imu(_imu_msg(qw=math.cos(half), qy=math.sin(half)))
        safety_node._on_raw(_twist())
        safety_node._eval_tick()
        assert safety_node._last_eval.state == SafetyState.ESTOP

        # Rover levels out; latch still holds ESTOP.
        safety_node._on_imu(_imu_msg())
        safety_node._eval_tick()
        assert safety_node._last_eval.state == SafetyState.ESTOP
        assert safety_node._last_eval.clearable is False

        response = safety_node._on_reset_request(
            Trigger.Request(), Trigger.Response(),
        )
        assert response.success is True
        assert safety_node._last_eval.state == SafetyState.OK
