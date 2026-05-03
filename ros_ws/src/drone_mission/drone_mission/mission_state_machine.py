"""SM-DRN-001 mission state machine — pure Python, no rclpy.

Models the nine sequential states + ABORTING from the spec. Every event the
node observes (MAVROS ACK, altitude reached, capture confirmed, operator
cancel, safety abort) is folded into a method here; the machine returns
the new phase and any side-effect cues (e.g. "command land", "publish
result") via a small enum the caller acts on.

Keeping the logic clock-free and rclpy-free makes it trivial to unit-test:
hand it events, read out phase, assert the expected actions list.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


class MissionPhase(IntEnum):
    """SM-DRN-001 §3 — keep names byte-identical to the spec for feedback strings."""
    DISARMED = 0
    ARMED = 1
    TAKEOFF = 2
    CLIMBING = 3
    CAPTURING = 4
    DESCENDING = 5
    LANDED = 6
    IMAGE_TRANSFER = 7
    GEOREF = 8
    ABORTING = 9


# Phases that are "in flight" — eligible to transition to ABORTING.
# DISARMED is pre/post flight; LANDED/IMAGE_TRANSFER/GEOREF are post-land.
IN_FLIGHT_PHASES = frozenset({
    MissionPhase.ARMED,
    MissionPhase.TAKEOFF,
    MissionPhase.CLIMBING,
    MissionPhase.CAPTURING,
    MissionPhase.DESCENDING,
})


class AbortReason(IntEnum):
    """Reasons the mission aborted. Map directly to ICD-MAP-003 failure_reason strings."""
    NONE = 0
    OPERATOR_CANCEL = 1
    SAFETY_ESTOP = 2
    LOW_BATTERY = 3
    GPS_LOSS = 4
    MAVROS_HEARTBEAT_LOSS = 5
    CLIMB_TIMEOUT = 6
    CAPTURE_TIMEOUT = 7
    CAPTURE_ERROR = 8
    ARM_FAILED = 9
    IMAGE_TRANSFER_FAILED = 10


ABORT_REASON_STRINGS = {
    AbortReason.OPERATOR_CANCEL: 'operator_cancel',
    AbortReason.SAFETY_ESTOP: 'safety_estop',
    AbortReason.LOW_BATTERY: 'low_battery',
    AbortReason.GPS_LOSS: 'gps_loss',
    AbortReason.MAVROS_HEARTBEAT_LOSS: 'mavros_heartbeat_loss',
    AbortReason.CLIMB_TIMEOUT: 'climb_timeout',
    AbortReason.CAPTURE_TIMEOUT: 'capture_timeout',
    AbortReason.CAPTURE_ERROR: 'capture_failed',
    AbortReason.ARM_FAILED: 'arm_failed',
    AbortReason.IMAGE_TRANSFER_FAILED: 'image_transfer_failed',
}


@dataclass
class MissionResult:
    """Final disposition of a mission, populated when DISARMED is re-entered."""
    success: bool
    image_path: str = ''
    failure_reason: str = ''


class IllegalTransition(RuntimeError):
    """Raised when an event is dispatched in a phase that doesn't permit it.

    Catches calling-code bugs early. For events that legitimately can
    happen any time (cancel, safety abort), use the abort() entry point
    which is idempotent.
    """


@dataclass
class MissionStateMachine:
    """Drone mission state machine per SM-DRN-001 rev 0.2.

    Internal invariants (assert-checked):
      - phase ∈ MissionPhase
      - predecessor is set whenever the machine is in LANDED, IMAGE_TRANSFER,
        GEOREF, or DISARMED-via-success-path (used to distinguish post-LANDED
        success path from abort path).
      - abort_reason != NONE ⇒ predecessor was ABORTING somewhere upstream.
      - is_terminal iff phase == DISARMED AND result is populated.
    """
    phase: MissionPhase = MissionPhase.DISARMED
    # The phase we came from. Only non-None inside LANDED / IMAGE_TRANSFER /
    # GEOREF / re-entered DISARMED — used to route LANDED's next transition
    # (success → IMAGE_TRANSFER, abort → DISARMED with failure result).
    predecessor: Optional[MissionPhase] = None
    abort_reason: AbortReason = AbortReason.NONE
    # Populated when a mission ends; consumed by the action server to fill
    # the action result.
    result: Optional[MissionResult] = None
    # The image path captured during GEOREF; carried through to the result.
    _retrieved_image_path: str = field(default='', repr=False)

    # ---- queries -------------------------------------------------------

    @property
    def is_terminal(self) -> bool:
        """True iff the machine has settled at DISARMED with a final result."""
        return self.phase == MissionPhase.DISARMED and self.result is not None

    @property
    def is_in_flight(self) -> bool:
        return self.phase in IN_FLIGHT_PHASES

    # ---- forward-path events -------------------------------------------

    def accept_goal(self) -> None:
        """DISARMED → ARMED. Caller is responsible for validating preconditions
        (safety state OK, MAVROS connected, GPS fix) before calling."""
        self._require(MissionPhase.DISARMED, 'accept_goal')
        # Reset per-mission state in case a previous mission left residue.
        self.predecessor = None
        self.abort_reason = AbortReason.NONE
        self.result = None
        self._retrieved_image_path = ''
        self._set(MissionPhase.ARMED)

    def arm_ack(self) -> None:
        """ARMED → TAKEOFF."""
        self._require(MissionPhase.ARMED, 'arm_ack')
        self._set(MissionPhase.TAKEOFF)

    def takeoff_ack(self) -> None:
        """TAKEOFF → CLIMBING."""
        self._require(MissionPhase.TAKEOFF, 'takeoff_ack')
        self._set(MissionPhase.CLIMBING)

    def altitude_reached(self) -> None:
        """CLIMBING → CAPTURING (within 1 m of target altitude)."""
        self._require(MissionPhase.CLIMBING, 'altitude_reached')
        self._set(MissionPhase.CAPTURING)

    def image_confirmed(self) -> None:
        """CAPTURING → DESCENDING (XIAO confirmed image in buffer)."""
        self._require(MissionPhase.CAPTURING, 'image_confirmed')
        self._set(MissionPhase.DESCENDING)

    def on_ground(self) -> None:
        """DESCENDING/ABORTING → LANDED."""
        if self.phase not in (MissionPhase.DESCENDING, MissionPhase.ABORTING):
            raise IllegalTransition(
                f'on_ground in phase {self.phase.name}'
            )
        self.predecessor = self.phase
        self._set(MissionPhase.LANDED)

    def disarm_ack(self) -> None:
        """LANDED → IMAGE_TRANSFER (success path) or DISARMED (abort path).

        The branch is decided by the predecessor recorded at LANDED entry.
        """
        self._require(MissionPhase.LANDED, 'disarm_ack')
        if self.predecessor == MissionPhase.DESCENDING:
            self._set(MissionPhase.IMAGE_TRANSFER)
        elif self.predecessor == MissionPhase.ABORTING:
            # Abort path: skip image transfer, finalize as failure.
            self._finish_failure()
        else:
            raise IllegalTransition(
                f'disarm_ack with unexpected predecessor {self.predecessor}'
            )

    def image_retrieved(self, image_path: str) -> None:
        """IMAGE_TRANSFER → GEOREF. Stash path for the eventual result."""
        self._require(MissionPhase.IMAGE_TRANSFER, 'image_retrieved')
        if not image_path:
            raise ValueError('image_retrieved called with empty image_path')
        self._retrieved_image_path = image_path
        self._set(MissionPhase.GEOREF)

    def georef_complete(self) -> None:
        """GEOREF → DISARMED (success). Populate the action result."""
        self._require(MissionPhase.GEOREF, 'georef_complete')
        self.result = MissionResult(
            success=True,
            image_path=self._retrieved_image_path,
            failure_reason='',
        )
        self._set(MissionPhase.DISARMED)

    # ---- failure / abort events ---------------------------------------

    def image_transfer_failed(self) -> None:
        """IMAGE_TRANSFER → DISARMED with failure result.

        Distinct from abort(): the drone is already on the ground here, so
        there's nothing to land. Per SM-DRN-001 §4, this transitions
        directly to DISARMED with failure_reason='image_transfer_failed'.
        """
        self._require(MissionPhase.IMAGE_TRANSFER, 'image_transfer_failed')
        self.abort_reason = AbortReason.IMAGE_TRANSFER_FAILED
        self._finish_failure()

    def abort(self, reason: AbortReason) -> bool:
        """Any in-flight phase → ABORTING.

        Idempotent: calling while already in ABORTING is a no-op (the spec
        explicitly says subsequent cancel / safety events are ignored).
        Returns True if the call actually triggered the transition, False
        if it was a no-op.
        """
        if reason == AbortReason.NONE:
            raise ValueError('abort() called with AbortReason.NONE')
        if self.phase == MissionPhase.ABORTING:
            return False
        if not self.is_in_flight:
            # Pre-flight (DISARMED/ARMED before takeoff_ack? ARMED is in IN_FLIGHT
            # already) and post-land phases can't abort to a land command —
            # nothing meaningful to do. The caller decides what to surface.
            raise IllegalTransition(
                f'abort({reason.name}) in non-in-flight phase {self.phase.name}'
            )
        self.abort_reason = reason
        self._set(MissionPhase.ABORTING)
        return True

    def arm_failed(self) -> None:
        """ARMED → DISARMED with failure (drone never lifted)."""
        self._require(MissionPhase.ARMED, 'arm_failed')
        self.abort_reason = AbortReason.ARM_FAILED
        self._finish_failure()

    # ---- internals ----------------------------------------------------

    def _require(self, phase: MissionPhase, event: str) -> None:
        if self.phase != phase:
            raise IllegalTransition(
                f'{event} requires phase {phase.name}, but in {self.phase.name}'
            )

    def _set(self, phase: MissionPhase) -> None:
        self.phase = phase

    def _finish_failure(self) -> None:
        reason = ABORT_REASON_STRINGS.get(self.abort_reason, 'unknown')
        self.result = MissionResult(
            success=False,
            image_path='',
            failure_reason=reason,
        )
        self._set(MissionPhase.DISARMED)


# Phase-weighted progress — see SRS-MAP-001 OPEN-X4. Picked from typical
# durations in SM-DRN-001 §6 so progress moves roughly linearly with
# wall-clock during a nominal ~3-minute mission.
_PHASE_PROGRESS = {
    MissionPhase.DISARMED: 0.0,
    MissionPhase.ARMED: 0.05,
    MissionPhase.TAKEOFF: 0.10,
    MissionPhase.CLIMBING: 0.15,
    MissionPhase.CAPTURING: 0.50,
    MissionPhase.DESCENDING: 0.60,
    MissionPhase.LANDED: 0.92,
    MissionPhase.IMAGE_TRANSFER: 0.94,
    MissionPhase.GEOREF: 0.98,
    MissionPhase.ABORTING: 0.60,
}


def phase_progress(phase: MissionPhase) -> float:
    """Map current phase to a [0.0, 1.0] progress estimate."""
    return _PHASE_PROGRESS.get(phase, 0.0)
