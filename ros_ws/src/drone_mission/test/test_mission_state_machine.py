"""Unit tests for MissionStateMachine.

Pure-logic tests — no rclpy. Walks the state machine through every
documented transition in SM-DRN-001 §4 and verifies invariants.
"""

from __future__ import annotations

import pytest

from drone_mission.mission_state_machine import (
    AbortReason,
    IllegalTransition,
    MissionPhase,
    MissionStateMachine,
    phase_progress,
)


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


class TestHappyPath:
    def test_full_success_walks_disarmed_to_disarmed(self):
        sm = MissionStateMachine()
        assert sm.phase == MissionPhase.DISARMED
        assert sm.is_terminal is False  # nothing to terminate yet

        sm.accept_goal()
        assert sm.phase == MissionPhase.ARMED

        sm.arm_ack()
        assert sm.phase == MissionPhase.TAKEOFF

        sm.takeoff_ack()
        assert sm.phase == MissionPhase.CLIMBING
        assert sm.is_in_flight is True

        sm.altitude_reached()
        assert sm.phase == MissionPhase.CAPTURING

        sm.image_confirmed()
        assert sm.phase == MissionPhase.DESCENDING

        sm.on_ground()
        assert sm.phase == MissionPhase.LANDED
        assert sm.predecessor == MissionPhase.DESCENDING

        sm.disarm_ack()
        # success path: LANDED → IMAGE_TRANSFER (not DISARMED)
        assert sm.phase == MissionPhase.IMAGE_TRANSFER

        sm.image_retrieved('/var/lib/platform/images/m1/capture.jpg')
        assert sm.phase == MissionPhase.GEOREF

        sm.georef_complete()
        assert sm.phase == MissionPhase.DISARMED
        assert sm.is_terminal is True
        assert sm.result is not None
        assert sm.result.success is True
        assert sm.result.image_path == '/var/lib/platform/images/m1/capture.jpg'
        assert sm.result.failure_reason == ''


# ---------------------------------------------------------------------------
# Abort paths
# ---------------------------------------------------------------------------


class TestAborts:
    def test_climb_timeout_aborts_to_failure(self):
        sm = MissionStateMachine()
        sm.accept_goal(); sm.arm_ack(); sm.takeoff_ack()
        # CLIMBING; abort via climb timeout
        sm.abort(AbortReason.CLIMB_TIMEOUT)
        assert sm.phase == MissionPhase.ABORTING
        sm.on_ground()
        assert sm.phase == MissionPhase.LANDED
        assert sm.predecessor == MissionPhase.ABORTING
        sm.disarm_ack()
        # abort path: LANDED → DISARMED (not IMAGE_TRANSFER)
        assert sm.phase == MissionPhase.DISARMED
        assert sm.result is not None
        assert sm.result.success is False
        assert sm.result.failure_reason == 'climb_timeout'

    def test_safety_estop_aborts_from_capturing(self):
        sm = MissionStateMachine()
        sm.accept_goal(); sm.arm_ack(); sm.takeoff_ack()
        sm.altitude_reached()
        # CAPTURING; ESTOP fires
        assert sm.abort(AbortReason.SAFETY_ESTOP) is True
        assert sm.phase == MissionPhase.ABORTING
        sm.on_ground(); sm.disarm_ack()
        assert sm.result.failure_reason == 'safety_estop'

    def test_low_battery_during_descent_does_not_re_abort(self):
        sm = MissionStateMachine()
        sm.accept_goal(); sm.arm_ack(); sm.takeoff_ack()
        sm.altitude_reached(); sm.image_confirmed()
        # DESCENDING; abort fires
        sm.abort(AbortReason.LOW_BATTERY)
        # Subsequent abort during ABORTING is a no-op (per SM-DRN-001 §5).
        assert sm.abort(AbortReason.SAFETY_ESTOP) is False
        assert sm.abort_reason == AbortReason.LOW_BATTERY
        assert sm.phase == MissionPhase.ABORTING

    def test_arm_failed_short_circuits_to_disarmed(self):
        sm = MissionStateMachine()
        sm.accept_goal()
        # ARMED; arm rejected
        sm.arm_failed()
        assert sm.phase == MissionPhase.DISARMED
        assert sm.is_terminal is True
        assert sm.result.success is False
        assert sm.result.failure_reason == 'arm_failed'

    def test_image_transfer_failure_finalizes_after_landing(self):
        sm = MissionStateMachine()
        # walk to IMAGE_TRANSFER
        sm.accept_goal(); sm.arm_ack(); sm.takeoff_ack()
        sm.altitude_reached(); sm.image_confirmed()
        sm.on_ground(); sm.disarm_ack()
        assert sm.phase == MissionPhase.IMAGE_TRANSFER
        # Transfer fails (XIAO unreachable, retry exhausted)
        sm.image_transfer_failed()
        assert sm.phase == MissionPhase.DISARMED
        assert sm.result.success is False
        assert sm.result.failure_reason == 'image_transfer_failed'

    def test_abort_with_NONE_raises(self):
        sm = MissionStateMachine()
        sm.accept_goal()
        with pytest.raises(ValueError):
            sm.abort(AbortReason.NONE)

    def test_abort_from_disarmed_raises(self):
        sm = MissionStateMachine()
        with pytest.raises(IllegalTransition):
            sm.abort(AbortReason.OPERATOR_CANCEL)


# ---------------------------------------------------------------------------
# Illegal transitions surface as exceptions (calling-code bug catcher)
# ---------------------------------------------------------------------------


class TestIllegalTransitions:
    def test_takeoff_ack_before_arm_ack(self):
        sm = MissionStateMachine()
        sm.accept_goal()
        with pytest.raises(IllegalTransition):
            sm.takeoff_ack()  # in ARMED; expected TAKEOFF

    def test_disarm_ack_before_landed(self):
        sm = MissionStateMachine()
        sm.accept_goal(); sm.arm_ack()
        with pytest.raises(IllegalTransition):
            sm.disarm_ack()  # in TAKEOFF; expected LANDED

    def test_image_retrieved_with_empty_path(self):
        sm = MissionStateMachine()
        sm.accept_goal(); sm.arm_ack(); sm.takeoff_ack()
        sm.altitude_reached(); sm.image_confirmed()
        sm.on_ground(); sm.disarm_ack()  # IMAGE_TRANSFER
        with pytest.raises(ValueError):
            sm.image_retrieved('')

    def test_georef_complete_before_image_retrieved(self):
        sm = MissionStateMachine()
        sm.accept_goal(); sm.arm_ack(); sm.takeoff_ack()
        sm.altitude_reached(); sm.image_confirmed()
        sm.on_ground(); sm.disarm_ack()
        # In IMAGE_TRANSFER, not GEOREF
        with pytest.raises(IllegalTransition):
            sm.georef_complete()


# ---------------------------------------------------------------------------
# Mission re-use (DISARMED → ARMED again)
# ---------------------------------------------------------------------------


class TestMissionReuse:
    def test_terminal_disarmed_can_accept_new_goal(self):
        # Run one mission to terminal
        sm = MissionStateMachine()
        sm.accept_goal(); sm.arm_failed()
        assert sm.is_terminal is True

        # Accept a new goal — old result and abort_reason cleared.
        sm.accept_goal()
        assert sm.phase == MissionPhase.ARMED
        assert sm.result is None
        assert sm.abort_reason == AbortReason.NONE


# ---------------------------------------------------------------------------
# Progress mapping is monotonic on the success path
# ---------------------------------------------------------------------------


class TestProgressMapping:
    def test_progress_in_unit_interval(self):
        for phase in MissionPhase:
            p = phase_progress(phase)
            assert 0.0 <= p <= 1.0, f'{phase.name} → {p}'

    def test_success_path_is_monotonic(self):
        success_path = [
            MissionPhase.DISARMED, MissionPhase.ARMED, MissionPhase.TAKEOFF,
            MissionPhase.CLIMBING, MissionPhase.CAPTURING,
            MissionPhase.DESCENDING, MissionPhase.LANDED,
            MissionPhase.IMAGE_TRANSFER, MissionPhase.GEOREF,
        ]
        progresses = [phase_progress(p) for p in success_path]
        assert progresses == sorted(progresses), (
            f'progress not monotonic on success path: {progresses}'
        )
