"""SRS-MAP-001 drone_mission_coordinator — M3 scaffold.

Action server behind /mapping/run_mission (ICD-MAP-003). Owns the
mission state machine (SM-DRN-001) and orchestrates a pluggable MAVROS
backend, the image_retriever service (ICD-MAP-004), and the georeferencer
service (ICD-MAP-005) through one mission cycle.

M3 scaffold scope:
  - Action server end-to-end (accept → execute → result/feedback/cancel)
  - MockMavrosBackend that simulates a drone for tests + dev
  - Service-client calls to image_retriever and georeferencer (mockable
    via standard ROS 2 service clients pointing at test stubs)
  - Safety-state subscription that aborts the mission when ESTOP fires
  - Feedback at ≥ 2 Hz nominal driven by a separate timer

Out of M3-scaffold scope (deferred):
  - RealMavrosBackend implementation (no drone yet; class is scaffold-only)
  - Full TEST-MAP-020/021/022 field tests (these are field tests by design)
"""

from __future__ import annotations

import sys
import threading
import time
import uuid
from typing import Optional

import rclpy
from platform_msgs.action import RunMappingMission
from platform_msgs.msg import SafetyState as SafetyStateMsg
from platform_msgs.srv import Georeference, RetrieveLatestImage
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from drone_mission.mavros_backend import (
    MavrosBackend,
    make_mavros_backend,
)
from drone_mission.mission_state_machine import (
    AbortReason,
    MissionPhase,
    MissionStateMachine,
    phase_progress,
)


class DroneMissionCoordinator(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('drone_mission_coordinator', **kwargs)

        # ---- parameters ------------------------------------------------
        self.declare_parameter('mavros_backend', 'mock')
        self.declare_parameter('default_altitude_m', 30.0)
        self.declare_parameter('altitude_tolerance_m', 1.0)
        self.declare_parameter('climb_timeout_s', 45.0)
        self.declare_parameter('capture_hover_s', 3.0)
        self.declare_parameter('capture_timeout_s', 10.0)
        self.declare_parameter('descent_timeout_s', 60.0)
        self.declare_parameter('image_transfer_timeout_s', 30.0)
        self.declare_parameter('georef_timeout_s', 5.0)
        self.declare_parameter('feedback_rate_hz', 2.0)
        # Battery floor at which an in-flight mission aborts; matches
        # SM-DRN-001 §4 ("low battery (< 20%)") and must trip strictly
        # before the FC's BATT_LOW_VOLT failsafe (FW-ACP §3) per
        # SRS-MAP-001-S03.
        self.declare_parameter('abort_on_low_battery_pct', 20.0)
        # Pre-arm check tolerates a non-OK safety state during dev/bench.
        # In production this should be False so safety_monitor gates arming.
        self.declare_parameter('require_safety_ok', True)

        backend_name = str(self.get_parameter('mavros_backend').value)
        self._backend: MavrosBackend = make_mavros_backend(backend_name, self)
        self._altitude_tol = float(self.get_parameter('altitude_tolerance_m').value)
        self._climb_timeout = float(self.get_parameter('climb_timeout_s').value)
        self._capture_hover = float(self.get_parameter('capture_hover_s').value)
        self._capture_timeout = float(self.get_parameter('capture_timeout_s').value)
        self._descent_timeout = float(self.get_parameter('descent_timeout_s').value)
        self._image_transfer_timeout = float(
            self.get_parameter('image_transfer_timeout_s').value
        )
        self._georef_timeout = float(self.get_parameter('georef_timeout_s').value)
        self._battery_floor = float(self.get_parameter('abort_on_low_battery_pct').value)
        self._require_safety_ok = bool(self.get_parameter('require_safety_ok').value)

        # ---- state -----------------------------------------------------
        self._sm = MissionStateMachine()
        self._mission_id: str = ''
        self._goal_altitude_m: float = float(
            self.get_parameter('default_altitude_m').value
        )
        # Last-known safety state. None ⇒ never seen.
        self._safety_state: Optional[int] = None
        self._active_goal_handle: Optional[ServerGoalHandle] = None
        # Mission lock prevents two missions running at once even though
        # rclpy actions allow multiple goals queued — simpler than wiring
        # a goal-rejection policy here.
        self._mission_lock = threading.Lock()

        # ---- callback groups -------------------------------------------
        # The action execute callback runs on its own group so the mission
        # loop can sleep without blocking the safety subscription or the
        # feedback timer.
        self._action_cbgroup = ReentrantCallbackGroup()
        self._safety_cbgroup = MutuallyExclusiveCallbackGroup()
        self._feedback_cbgroup = MutuallyExclusiveCallbackGroup()

        # ---- service clients -------------------------------------------
        self._image_client = self.create_client(
            RetrieveLatestImage, '/mapping/retrieve_latest_image',
            callback_group=self._action_cbgroup,
        )
        self._georef_client = self.create_client(
            Georeference, '/mapping/georeference',
            callback_group=self._action_cbgroup,
        )

        # ---- safety subscription --------------------------------------
        # ICD-SAF-001 §6 publisher uses TRANSIENT_LOCAL; match for the
        # late-joining behavior on coordinator restart.
        safety_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )
        self.create_subscription(
            SafetyStateMsg, '/safety/state', self._on_safety_state,
            safety_qos, callback_group=self._safety_cbgroup,
        )

        # ---- action server --------------------------------------------
        self._action_server = ActionServer(
            self,
            RunMappingMission,
            '/mapping/run_mission',
            execute_callback=self._execute_mission,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._action_cbgroup,
        )

        # ---- feedback timer -------------------------------------------
        feedback_period = 1.0 / float(self.get_parameter('feedback_rate_hz').value)
        self.create_timer(
            feedback_period, self._publish_feedback,
            callback_group=self._feedback_cbgroup,
        )

        self.get_logger().info(
            f'drone_mission_coordinator started '
            f'(backend={backend_name}, altitude={self._goal_altitude_m:.1f}m, '
            f'climb_timeout={self._climb_timeout}s, '
            f'battery_floor={self._battery_floor}%, '
            f'safety_required={self._require_safety_ok})'
        )

    # ----- safety + action callbacks ------------------------------------

    def _on_safety_state(self, msg: SafetyStateMsg) -> None:
        prev = self._safety_state
        self._safety_state = msg.state
        # If we transitioned to ESTOP mid-mission, raise the abort flag
        # for the mission loop to observe on its next iteration.
        if (
            prev != SafetyStateMsg.STATE_ESTOP
            and msg.state == SafetyStateMsg.STATE_ESTOP
            and self._sm.is_in_flight
        ):
            self.get_logger().warning(
                f'safety state → ESTOP (reasons={list(msg.reasons)}); '
                f'aborting mission'
            )
            try:
                self._sm.abort(AbortReason.SAFETY_ESTOP)
            except Exception as exc:  # noqa: BLE001 — best-effort signal
                self.get_logger().warning(
                    f'abort signal raced with phase change: {exc}'
                )

    def _on_goal(self, goal_request) -> GoalResponse:
        """Decide whether to accept a new goal.

        REQ-ICD-MAP-003-01: accept within 100 ms when in valid pre-arm state.
        We enforce single-mission-at-a-time here rather than queueing.
        """
        if self._active_goal_handle is not None and self._active_goal_handle.is_active:
            self.get_logger().warning(
                'rejecting goal: another mission is already active'
            )
            return GoalResponse.REJECT
        if self._require_safety_ok and self._safety_state != SafetyStateMsg.STATE_OK:
            self.get_logger().warning(
                f'rejecting goal: safety state not OK '
                f'(current={self._safety_state}, require_safety_ok=True)'
            )
            return GoalResponse.REJECT
        if not self._backend.is_connected():
            self.get_logger().warning('rejecting goal: MAVROS not connected')
            return GoalResponse.REJECT
        if not self._backend.has_gps_fix():
            self.get_logger().warning('rejecting goal: no GPS fix')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        """Operator pressed cancel. Always accept; the mission loop sees
        is_cancel_requested next iteration and aborts."""
        self.get_logger().info('cancel request received')
        return CancelResponse.ACCEPT

    # ----- mission execution --------------------------------------------

    def _execute_mission(self, goal_handle: ServerGoalHandle):
        """Drive the state machine through one mission cycle.

        Runs on its own callback-group thread (action_cbgroup is
        ReentrantCallbackGroup) so blocking sleeps don't starve the
        safety subscription or the feedback timer.
        """
        with self._mission_lock:
            self._active_goal_handle = goal_handle
            self._mission_id = str(uuid.uuid4())
            goal = goal_handle.request
            self._goal_altitude_m = float(goal.altitude_m or self._goal_altitude_m)
            self.get_logger().info(
                f'mission {self._mission_id} accepted '
                f'(altitude={self._goal_altitude_m:.1f}m, '
                f'output_name={goal.output_name!r})'
            )

            try:
                self._run_mission(goal_handle)
            except Exception as exc:  # noqa: BLE001 — surface any bug as failure
                self.get_logger().error(f'mission loop crashed: {exc}')
                if self._sm.is_in_flight:
                    try:
                        self._sm.abort(AbortReason.MAVROS_HEARTBEAT_LOSS)
                    except Exception:
                        pass

            result = self._build_result(goal.output_name or '')
            if result.success:
                goal_handle.succeed()
            else:
                # Distinguish operator-cancel from other failures so the
                # action client can present them appropriately.
                if (
                    self._sm.abort_reason == AbortReason.OPERATOR_CANCEL
                    and goal_handle.is_cancel_requested
                ):
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
            self._active_goal_handle = None
            return result

    def _run_mission(self, goal_handle: ServerGoalHandle) -> None:
        """The actual driving loop. Called from inside _execute_mission."""
        sm = self._sm
        sm.accept_goal()  # DISARMED → ARMED

        # ---- Arm ------------------------------------------------------
        if not self._backend.arm():
            sm.arm_failed()
            return
        sm.arm_ack()  # ARMED → TAKEOFF

        # ---- Takeoff --------------------------------------------------
        if not self._backend.takeoff(self._goal_altitude_m):
            # Treat takeoff rejection as an abort; ArduCopter will hold on
            # the ground and the disarm in _finalize_landed() unwinds us.
            sm.abort(AbortReason.ARM_FAILED)
        else:
            sm.takeoff_ack()  # TAKEOFF → CLIMBING

            # ---- Climb to target altitude ----------------------------
            if not self._wait_for_altitude(goal_handle):
                pass  # _wait_for_altitude already aborted if needed
            else:
                sm.altitude_reached()  # CLIMBING → CAPTURING

                # ---- Capture ----------------------------------------
                if self._do_capture(goal_handle):
                    sm.image_confirmed()  # CAPTURING → DESCENDING

                    # ---- Descent + land --------------------------
                    self._wait_for_landing(goal_handle, climb_back=False)

        # ---- Land observed (success or abort path) ------------------
        # If we're already DISARMED (arm_failed), we're done.
        if sm.phase == MissionPhase.DISARMED:
            return

        # Otherwise drive through ABORTING-or-DESCENDING → LANDED → DISARMED.
        # If we're in an in-flight phase that hasn't issued land yet, do so now
        # (e.g. abort fired before any land command issued).
        if sm.is_in_flight or sm.phase == MissionPhase.ABORTING:
            self._wait_for_landing(goal_handle, climb_back=False)

        if sm.phase == MissionPhase.LANDED:
            self._backend.disarm()
            sm.disarm_ack()  # LANDED → IMAGE_TRANSFER (success) or DISARMED (abort)

        # ---- Image transfer + georef (success path only) ------------
        if sm.phase == MissionPhase.IMAGE_TRANSFER:
            if self._do_image_transfer():
                if self._do_georef():
                    sm.georef_complete()  # GEOREF → DISARMED (success)
                else:
                    sm.image_transfer_failed()
            else:
                sm.image_transfer_failed()

    # ----- mission-loop helpers -----------------------------------------

    def _wait_for_altitude(self, goal_handle: ServerGoalHandle) -> bool:
        """Spin until altitude is within tolerance of target or we abort.

        Returns True iff target altitude was reached. Side-effects: drives
        the state machine to ABORTING on timeout / cancel / safety event.
        """
        target = self._goal_altitude_m
        deadline = time.monotonic() + self._climb_timeout
        while time.monotonic() < deadline:
            if self._maybe_abort(goal_handle):
                return False
            if abs(self._backend.altitude_m() - target) <= self._altitude_tol:
                return True
            time.sleep(0.1)
        # Timeout
        try:
            self._sm.abort(AbortReason.CLIMB_TIMEOUT)
        except Exception:
            pass
        return False

    def _do_capture(self, goal_handle: ServerGoalHandle) -> bool:
        """Hover for capture_hover_s, then call XIAO /capture (via image_retriever's
        contract — but at this scaffold layer we don't actually trigger the
        XIAO; image_retriever does that on the post-land /retrieve call).

        Returns True iff capture phase completed without abort.
        """
        deadline = time.monotonic() + self._capture_hover
        while time.monotonic() < deadline:
            if self._maybe_abort(goal_handle):
                return False
            time.sleep(0.05)
        return True

    def _wait_for_landing(
        self, goal_handle: ServerGoalHandle, *, climb_back: bool,
    ) -> None:
        """Issue land command and wait for landed_state=ON_GROUND.

        Always called with the SM in DESCENDING or ABORTING. Drives the SM
        to LANDED on success.
        """
        # Issue land if not already commanded. Backends are idempotent.
        self._backend.land()
        deadline = time.monotonic() + self._descent_timeout
        while time.monotonic() < deadline:
            # During descent we still honor cancel/safety as further aborts,
            # but they're no-ops if we're already in ABORTING.
            self._maybe_abort(goal_handle, ignore_if_aborting=True)
            if self._backend.landed_state_on_ground():
                self._sm.on_ground()  # → LANDED
                return
            time.sleep(0.1)
        # Land timeout: there is nothing more we can do from software.
        self.get_logger().error(
            'descent timeout — drone did not reach ON_GROUND in time'
        )

    def _do_image_transfer(self) -> bool:
        """Call /mapping/retrieve_latest_image. Returns True iff success."""
        if not self._image_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                '/mapping/retrieve_latest_image not available'
            )
            return False
        req = RetrieveLatestImage.Request()
        req.mission_id = self._mission_id
        future = self._image_client.call_async(req)
        deadline = time.monotonic() + self._image_transfer_timeout
        while time.monotonic() < deadline:
            if future.done():
                resp = future.result()
                if resp is None or not resp.success:
                    self.get_logger().warning(
                        f'image retrieval failed: '
                        f'{getattr(resp, "message", "no response")}'
                    )
                    return False
                # Success — pass the path to the SM via image_retrieved.
                try:
                    self._sm.image_retrieved(resp.image_path)
                    return True
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(
                        f'image_retrieved rejected: {exc}'
                    )
                    return False
            time.sleep(0.1)
        self.get_logger().error('image retrieval timed out')
        return False

    def _do_georef(self) -> bool:
        """Call /mapping/georeference. Returns True iff success."""
        if not self._georef_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('/mapping/georeference not available')
            return False
        req = Georeference.Request()
        # The image_path was stashed in the SM; re-derive it from the
        # MissionResult-in-progress isn't available, so we hold it in the
        # SM and read it back on georef_complete via the result.
        req.image_path = self._sm._retrieved_image_path  # noqa: SLF001 (intentional)
        # Pose comes from the FC at shutter time. At M3 we don't have a
        # real shutter event, so stamp the current backend state. When
        # the real backend lands, capture this from /mavros/global_position
        # at the actual capture moment.
        req.capture_lat = 0.0
        req.capture_lon = 0.0
        req.capture_alt_m = float(self._backend.altitude_m())
        future = self._georef_client.call_async(req)
        deadline = time.monotonic() + self._georef_timeout
        while time.monotonic() < deadline:
            if future.done():
                resp = future.result()
                if resp is None or not resp.success:
                    self.get_logger().warning(
                        f'georeference failed: '
                        f'{getattr(resp, "message", "no response")}'
                    )
                    return False
                return True
            time.sleep(0.05)
        self.get_logger().error('georeference timed out')
        return False

    def _maybe_abort(
        self, goal_handle: ServerGoalHandle, *, ignore_if_aborting: bool = False,
    ) -> bool:
        """Check abort sources (cancel, safety state, battery, MAVROS link).

        Returns True iff an abort was triggered (or already in progress).
        ignore_if_aborting=True suppresses re-entry attempts during descent.
        """
        sm = self._sm
        if sm.phase == MissionPhase.ABORTING:
            return True

        reason: Optional[AbortReason] = None
        if goal_handle.is_cancel_requested:
            reason = AbortReason.OPERATOR_CANCEL
        elif self._safety_state == SafetyStateMsg.STATE_ESTOP:
            reason = AbortReason.SAFETY_ESTOP
        elif self._backend.battery_pct() >= 0 and self._backend.battery_pct() < self._battery_floor:
            reason = AbortReason.LOW_BATTERY
        elif not self._backend.is_connected():
            reason = AbortReason.MAVROS_HEARTBEAT_LOSS
        elif not self._backend.has_gps_fix():
            reason = AbortReason.GPS_LOSS

        if reason is None:
            return False

        if ignore_if_aborting and sm.phase == MissionPhase.ABORTING:
            return True
        try:
            sm.abort(reason)
        except Exception:
            return True   # already past the in-flight window; treat as aborted
        return True

    def _build_result(self, output_name: str) -> RunMappingMission.Result:
        result = RunMappingMission.Result()
        result.mission_id = self._mission_id
        if self._sm.result is None:
            # Mission machine never finalized — conservative failure.
            result.success = False
            result.failure_reason = 'mission_did_not_finalize'
            result.image_path = ''
        else:
            result.success = self._sm.result.success
            result.image_path = self._sm.result.image_path
            result.failure_reason = self._sm.result.failure_reason
        # output_name is recorded in the mission_id namespace by image_retriever;
        # we don't echo it back in the result, but log for traceability.
        if output_name:
            self.get_logger().info(
                f'mission {self._mission_id} done '
                f'(output_name={output_name!r}, success={result.success})'
            )
        return result

    # ----- feedback ----------------------------------------------------

    def _publish_feedback(self) -> None:
        """≥ 2 Hz feedback per ICD-MAP-003 §5. Active only when a goal is in flight."""
        gh = self._active_goal_handle
        if gh is None or not gh.is_active:
            return
        fb = RunMappingMission.Feedback()
        fb.phase = self._sm.phase.name
        fb.progress = phase_progress(self._sm.phase)
        fb.altitude_m = float(self._backend.altitude_m())
        fb.battery_pct = float(self._backend.battery_pct())
        gh.publish_feedback(fb)

    # ----- shutdown ----------------------------------------------------

    def destroy_node(self) -> bool:
        try:
            shutdown = getattr(self._backend, 'shutdown', None)
            if shutdown is not None:
                shutdown()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'backend shutdown failed: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneMissionCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
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
