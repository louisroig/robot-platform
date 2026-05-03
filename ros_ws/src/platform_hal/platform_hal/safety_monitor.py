"""SRS-SAF-001 safety_monitor — M2 real gating.

The single software authority over whether the rover may move. Subscribes
the safety inputs that exist at M2, runs a state machine, and gates
/hal/cmd_vel_raw → /hal/cmd_vel_safe. Out-of-scope inputs from the full
spec (perception, geofence, battery) arrive in later milestones.

M2 triggers (subset of SRS-SAF-001 §4):
  - tilt > tilt_limit_deg ............. ESTOP, latched   (excursion)
  - tilt > tilt_warning_deg ........... WARNING, auto
  - /hal/imu/data stale ............... ESTOP, auto      (S01)
  - /hal/cmd_vel_raw stale ............ ESTOP, auto      (S01, heartbeat)
  - NaN/Inf on /hal/cmd_vel_raw ....... ESTOP, auto      (S03)
  - startup self-test incomplete ...... ESTOP, auto      (F05)

Tilt latching deviates from the strict spec (which classifies tilt as
auto-clearable). At M2 it is the only excursion-class trigger we have,
and the only trigger that exercises the /safety/reset codepath; treating
it as latched keeps the reset service meaningful and is the safer default
for a tip-over event. Configurable via the `tilt_latches` parameter.

Publishes /safety/state (platform_msgs/msg/SafetyState) per ICD-SAF-001:
RELIABLE, KEEP_LAST depth 3, TRANSIENT_LOCAL, 10 Hz baseline plus a
≤20 ms on-change update driven from the input callbacks.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from platform_msgs.msg import SafetyState as SafetyStateMsg
from platform_msgs.srv import ResetSafety
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu


class SafetyState(IntEnum):
    OK = 0
    WARNING = 1
    ESTOP = 2


def tilt_angle_rad(qw: float, qx: float, qy: float, qz: float) -> float:
    """Angle between the body z-axis and the world z-axis (REP-103).

    For a unit quaternion (w, x, y, z) describing body-from-world rotation,
    the body z-axis expressed in world frame has z-component
    R[2,2] = 1 - 2(x² + y²). The tilt is acos of that, clamped for numeric
    safety. A level rover (R[2,2]≈1) returns ≈0; tipped on its side
    (R[2,2]≈0) returns ≈π/2.
    """
    cos_tilt = 1.0 - 2.0 * (qx * qx + qy * qy)
    if cos_tilt > 1.0:
        cos_tilt = 1.0
    elif cos_tilt < -1.0:
        cos_tilt = -1.0
    return math.acos(cos_tilt)


def twist_is_finite(msg: Twist) -> bool:
    fields = (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z,
    )
    return all(math.isfinite(v) for v in fields)


@dataclass
class TriggerStatus:
    """One row of the trigger table inside the state machine.

    `active` reflects the current world ("is the condition true right now");
    `latched` is sticky and only cleared by /safety/reset. A trigger
    contributes to the state if active OR latched. `severity` chooses
    between WARNING and ESTOP; the worst trigger wins.
    """
    name: str
    severity: SafetyState
    active: bool = False
    latched: bool = False
    latches: bool = False  # if True, going active sets the latch bit


@dataclass
class StateEvaluation:
    state: SafetyState
    reasons: list[str]
    clearable: bool
    triggers_changed: bool = field(default=False)


class StateMachine:
    """Pure-logic state machine. No rclpy, no clocks; takes facts, returns state.

    Every input the node observes (last_imu_time, last_cmd_vel_time, last
    quaternion, etc.) is folded into trigger booleans before evaluation.
    Keeping the logic clock-free makes unit tests trivial: hand it
    timestamps and read out states.
    """

    def __init__(
        self,
        *,
        tilt_limit_deg: float,
        tilt_warning_deg: float,
        tilt_latches: bool,
    ) -> None:
        self._tilt_limit_rad = math.radians(tilt_limit_deg)
        self._tilt_warning_rad = math.radians(tilt_warning_deg)
        self._triggers: dict[str, TriggerStatus] = {
            'startup_incomplete': TriggerStatus(
                name='startup_incomplete',
                severity=SafetyState.ESTOP,
                active=True,
            ),
            'tilt_exceeded': TriggerStatus(
                name='tilt_exceeded',
                severity=SafetyState.ESTOP,
                latches=tilt_latches,
            ),
            'tilt_warning': TriggerStatus(
                name='tilt_warning',
                severity=SafetyState.WARNING,
            ),
            'imu_stale': TriggerStatus(
                name='topic_stale:/hal/imu/data',
                severity=SafetyState.ESTOP,
            ),
            'cmd_vel_stale': TriggerStatus(
                name='topic_stale:/hal/cmd_vel_raw',
                severity=SafetyState.ESTOP,
            ),
            'cmd_vel_invalid': TriggerStatus(
                name='cmd_vel_raw_nonfinite',
                severity=SafetyState.ESTOP,
            ),
        }
        self._last_state = SafetyState.ESTOP

    def set_active(self, key: str, active: bool) -> None:
        """Set or clear a trigger's `active` bit; latch on rising edge."""
        t = self._triggers[key]
        if active and not t.active and t.latches:
            t.latched = True
        t.active = active

    def latched_keys_active(self) -> list[str]:
        """Triggers whose latch bit is set AND whose underlying condition is still active."""
        return [k for k, t in self._triggers.items() if t.latched and t.active]

    def clear_latches(self) -> bool:
        """Clear every latch bit whose underlying condition is no longer active.

        Returns True if at least one latch was cleared. Latches whose
        underlying condition is still active are left in place — /safety/reset
        cannot defeat a live trigger (per ICD-SAF-002 §1).
        """
        cleared_any = False
        for t in self._triggers.values():
            if t.latched and not t.active:
                t.latched = False
                cleared_any = True
        return cleared_any

    def update_tilt(self, tilt_rad: Optional[float]) -> None:
        """Update tilt-derived triggers from the latest quaternion. None = no IMU yet."""
        if tilt_rad is None:
            self.set_active('tilt_exceeded', False)
            self.set_active('tilt_warning', False)
            return
        self.set_active('tilt_exceeded', tilt_rad > self._tilt_limit_rad)
        # Warning is only meaningful below the ESTOP threshold — when both
        # would fire, ESTOP wins anyway via the worst-trigger rule.
        self.set_active(
            'tilt_warning',
            self._tilt_warning_rad < tilt_rad <= self._tilt_limit_rad,
        )

    def evaluate(self) -> StateEvaluation:
        """Compute the current state from the trigger table."""
        active_or_latched = [
            t for t in self._triggers.values() if t.active or t.latched
        ]
        if not active_or_latched:
            state = SafetyState.OK
            reasons: list[str] = []
            clearable = True
        else:
            state = max(t.severity for t in active_or_latched)
            reasons = sorted(t.name for t in active_or_latched)
            # Clearable iff every contributing trigger is currently active
            # (i.e. condition itself, not a stale latch). Latched-and-cleared
            # triggers require /safety/reset.
            clearable = not any(t.latched and not t.active for t in active_or_latched)

        changed = state != self._last_state
        self._last_state = state
        return StateEvaluation(
            state=state, reasons=reasons,
            clearable=clearable, triggers_changed=changed,
        )


class SafetyMonitor(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('safety_monitor', **kwargs)

        # Thresholds and timing windows (defaults match SRS-SAF-001 §9 where
        # the spec specifies them; staleness windows are M2 picks consistent
        # with the per-topic publish rates).
        self.declare_parameter('tilt_limit_deg', 25.0)
        self.declare_parameter('tilt_warning_deg', 18.0)
        self.declare_parameter('tilt_latches', True)
        self.declare_parameter('imu_staleness_ms', 200)        # 100 Hz pub → 20 ms; 10× margin
        self.declare_parameter('cmd_vel_staleness_ms', 500)    # matches motor_driver timeout
        self.declare_parameter('eval_rate_hz', 10.0)           # SM-SAF-001 §6
        self.declare_parameter('safe_publish_rate_hz', 50.0)   # ICD §3.2 (cmd_vel_safe 50 Hz)

        self._tilt_limit_rad = math.radians(
            float(self.get_parameter('tilt_limit_deg').value)
        )
        self._imu_stale_ns = (
            int(self.get_parameter('imu_staleness_ms').value) * 1_000_000
        )
        self._cmd_vel_stale_ns = (
            int(self.get_parameter('cmd_vel_staleness_ms').value) * 1_000_000
        )

        self._sm = StateMachine(
            tilt_limit_deg=float(self.get_parameter('tilt_limit_deg').value),
            tilt_warning_deg=float(self.get_parameter('tilt_warning_deg').value),
            tilt_latches=bool(self.get_parameter('tilt_latches').value),
        )

        # Last-known facts about the world. None ⇒ never seen.
        self._last_imu_stamp_ns: Optional[int] = None
        self._last_tilt_rad: Optional[float] = None
        self._last_cmd_vel_stamp_ns: Optional[int] = None
        self._last_cmd_vel: Optional[Twist] = None
        self._last_eval = StateEvaluation(
            state=SafetyState.ESTOP,
            reasons=['startup_incomplete'],
            clearable=True,
        )

        # Diagnostic counters.
        self._n_raw_received = 0
        self._n_raw_passed = 0
        self._n_raw_blocked = 0
        self._n_raw_nonfinite = 0
        self._n_resets_attempted = 0
        self._n_resets_granted = 0
        self._n_state_transitions = 0

        # All state-machine-touching callbacks (IMU, cmd_vel_raw, reset,
        # eval, safe_publish) share one mutex-exclusive group: serialized
        # against each other to prevent races on the trigger table, but
        # non-blocking against non-safety work (diagnostics) which lives in
        # the node default group. The MultiThreadedExecutor lets the two
        # groups run on separate threads so a slow diag publish never holds
        # up a safety callback (S04).
        self._safety_cbgroup = MutuallyExclusiveCallbackGroup()

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ICD-SAF-001 §6 — RELIABLE, KEEP_LAST depth 3, TRANSIENT_LOCAL so a
        # late-joining telemetry subscriber immediately sees the current state.
        safety_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )

        self._safe_pub = self.create_publisher(
            Twist, '/hal/cmd_vel_safe', cmd_vel_qos,
        )
        self._state_pub = self.create_publisher(
            SafetyStateMsg, '/safety/state', safety_state_qos,
        )
        self.create_subscription(
            Twist, '/hal/cmd_vel_raw', self._on_raw,
            cmd_vel_qos, callback_group=self._safety_cbgroup,
        )
        self.create_subscription(
            Imu, '/hal/imu/data', self._on_imu,
            imu_qos, callback_group=self._safety_cbgroup,
        )
        self.create_service(
            ResetSafety, '/safety/reset', self._on_reset_request,
            callback_group=self._safety_cbgroup,
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10,
        )

        eval_period = 1.0 / float(self.get_parameter('eval_rate_hz').value)
        safe_period = 1.0 / float(self.get_parameter('safe_publish_rate_hz').value)
        self.create_timer(
            eval_period, self._eval_tick, callback_group=self._safety_cbgroup,
        )
        self.create_timer(
            safe_period, self._safe_publish_tick, callback_group=self._safety_cbgroup,
        )
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f'safety_monitor started (tilt_limit={math.degrees(self._tilt_limit_rad):.1f}°, '
            f'imu_stale={self._imu_stale_ns // 1_000_000}ms, '
            f'cmd_vel_stale={self._cmd_vel_stale_ns // 1_000_000}ms)'
        )

    # ----- Subscriber callbacks ------------------------------------------------

    def _on_imu(self, msg: Imu) -> None:
        self._last_imu_stamp_ns = self.get_clock().now().nanoseconds
        q = msg.orientation
        self._last_tilt_rad = tilt_angle_rad(q.w, q.x, q.y, q.z)
        # Drive on-change /safety/state from the input event itself so tilt
        # threshold crossings hit the ≤20 ms ICD-SAF-001 §5 latency bound
        # rather than waiting for the next 10 Hz eval tick (~100 ms).
        self._reevaluate(publish_only_on_change=True)

    def _on_raw(self, msg: Twist) -> None:
        self._n_raw_received += 1
        self._last_cmd_vel_stamp_ns = self.get_clock().now().nanoseconds

        if not twist_is_finite(msg):
            # SRS-SAF-001-S03: zero output and flag for the eval tick to
            # promote to ESTOP. Do not retain the bad value.
            self._n_raw_nonfinite += 1
            self._sm.set_active('cmd_vel_invalid', True)
            self._last_cmd_vel = None
            self._safe_pub.publish(Twist())
            # Same on-change low-latency pattern as tilt — NaN is also a
            # safety-event input, not a steady-state condition.
            self._reevaluate(publish_only_on_change=True)
            return

        # Each clean message clears the invalid latch-source. Stale check is
        # the eval tick's job (it has the timestamps).
        self._sm.set_active('cmd_vel_invalid', False)
        self._last_cmd_vel = msg

        # Low-latency pass-through: republish immediately when permitted.
        # The 50 Hz safe-publish timer is the fail-safe / steady-state path;
        # this callback path keeps the actual command latency near zero.
        if self._last_eval.state != SafetyState.ESTOP:
            self._safe_pub.publish(msg)
            self._n_raw_passed += 1
        else:
            self._n_raw_blocked += 1

    # ----- Service ------------------------------------------------------------

    def _on_reset_request(
        self, _request: ResetSafety.Request, response: ResetSafety.Response,
    ) -> ResetSafety.Response:
        self._n_resets_attempted += 1
        # Re-evaluate from the latest sensor facts BEFORE deciding whether
        # to honor the request. Without this, the decision can use trigger
        # state from the previous eval tick — which may have been computed
        # before the most recent /hal/imu/data message arrived, leaving an
        # excursion-class trigger marked inactive even though the rover is
        # still tilted. ICD-SAF-002 §1: never force-clear an active condition.
        self._reevaluate()
        still_active = self._sm.latched_keys_active()
        if still_active:
            response.success = False
            response.reason = (
                f'cannot reset — latched conditions still active: '
                f'{", ".join(still_active)}'
            )
            return response
        # Clear latches; remaining auto-clearable triggers resolve themselves.
        self._sm.clear_latches()
        # Re-evaluate so /safety/state and /diagnostics reflect the cleared
        # state without waiting for the next tick.
        self._reevaluate()
        self._n_resets_granted += 1
        response.success = True
        response.reason = ''
        return response

    # ----- Timed work ---------------------------------------------------------

    def _reevaluate(self, *, publish_only_on_change: bool = False) -> None:
        """Re-derive triggers from facts, evaluate the SM, publish state.

        publish_only_on_change=True is the path taken from input callbacks:
        the steady-state /safety/state cadence comes from the 10 Hz eval
        timer; input-driven calls only emit an additional message when the
        state actually changed (ICD-SAF-001 §5 on-change ≤ 20 ms).
        """
        now_ns = self.get_clock().now().nanoseconds

        # Staleness watchdogs. INV-7: absence-of-data is unsafe.
        imu_stale = (
            self._last_imu_stamp_ns is None
            or (now_ns - self._last_imu_stamp_ns) > self._imu_stale_ns
        )
        self._sm.set_active('imu_stale', imu_stale)

        cmd_vel_stale = (
            self._last_cmd_vel_stamp_ns is None
            or (now_ns - self._last_cmd_vel_stamp_ns) > self._cmd_vel_stale_ns
        )
        self._sm.set_active('cmd_vel_stale', cmd_vel_stale)

        # Startup self-test (F05): clear once both inputs have been seen
        # at least once and neither is currently stale.
        if (
            self._last_imu_stamp_ns is not None
            and self._last_cmd_vel_stamp_ns is not None
            and not imu_stale
            and not cmd_vel_stale
        ):
            self._sm.set_active('startup_incomplete', False)

        # Tilt is None until first IMU; once seen, latest quaternion governs.
        self._sm.update_tilt(self._last_tilt_rad if not imu_stale else None)

        evaluation = self._sm.evaluate()
        if evaluation.triggers_changed:
            self._n_state_transitions += 1
            self.get_logger().info(
                f'safety state → {evaluation.state.name} '
                f'(reasons={evaluation.reasons or ["—"]})'
            )
        self._last_eval = evaluation
        if not publish_only_on_change or evaluation.triggers_changed:
            self._publish_safety_state(evaluation)

    def _eval_tick(self) -> None:
        """10 Hz baseline. Always publishes /safety/state."""
        self._reevaluate()

    def _publish_safety_state(self, evaluation: 'StateEvaluation') -> None:
        """Build and publish a SafetyState message reflecting `evaluation`.

        Permission flags follow SM-SAF-001 §3: motion and tool both true in
        OK/WARNING, both false in ESTOP. CRITICAL is the only state where
        they diverge (motion permitted to reach safe stop, tool blocked) and
        does not exist at M2.
        """
        msg = SafetyStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = int(evaluation.state)
        msg.reasons = list(evaluation.reasons)
        permitted = evaluation.state != SafetyState.ESTOP
        msg.motion_permitted = permitted
        msg.tool_permitted = permitted
        msg.clearable = evaluation.clearable
        self._state_pub.publish(msg)

    def _safe_publish_tick(self) -> None:
        """Steady-state cmd_vel_safe pump.

        ESTOP: publish zero so the motor_driver sees a fresh, definitive
        zero rather than relying on its own staleness fallback. OK/WARNING:
        no-op (the pass-through in _on_raw does the work; republishing here
        would just duplicate every command at the timer cadence).
        """
        if self._last_eval.state == SafetyState.ESTOP:
            self._safe_pub.publish(Twist())

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = 'platform_hal: safety_monitor'
        status.hardware_id = 'm2_gate'
        if self._last_eval.state == SafetyState.OK:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        elif self._last_eval.state == SafetyState.WARNING:
            status.level = DiagnosticStatus.WARN
            status.message = 'WARNING: ' + ', '.join(self._last_eval.reasons)
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'ESTOP: ' + ', '.join(self._last_eval.reasons or ['—'])

        tilt_deg_str = (
            f'{math.degrees(self._last_tilt_rad):.2f}'
            if self._last_tilt_rad is not None else 'unknown'
        )
        status.values = [
            KeyValue(key='state', value=self._last_eval.state.name),
            KeyValue(key='reasons', value=','.join(self._last_eval.reasons)),
            KeyValue(key='clearable', value=str(self._last_eval.clearable).lower()),
            KeyValue(key='tilt_deg', value=tilt_deg_str),
            KeyValue(key='raw_received', value=str(self._n_raw_received)),
            KeyValue(key='raw_passed', value=str(self._n_raw_passed)),
            KeyValue(key='raw_blocked', value=str(self._n_raw_blocked)),
            KeyValue(key='raw_nonfinite', value=str(self._n_raw_nonfinite)),
            KeyValue(key='resets_attempted', value=str(self._n_resets_attempted)),
            KeyValue(key='resets_granted', value=str(self._n_resets_granted)),
            KeyValue(key='state_transitions', value=str(self._n_state_transitions)),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    sys.exit(0)
