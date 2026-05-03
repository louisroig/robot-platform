"""MAVROS interaction abstraction for drone_mission_coordinator.

Two implementations:
  - MockMavrosBackend  — controllable from tests; simulates a drone
    that lifts, climbs, captures, and lands on a programmable timeline.
  - RealMavrosBackend  — talks to actual mavros services and topics.
    SCAFFOLD: not used in M3 development (no drone yet); raises a clear
    error if you forget to switch the backend before launching against
    the real FC.

The coordinator selects via the `mavros_backend` parameter.
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass


class MavrosBackend(ABC):
    """Interaction surface the mission state machine drives.

    Commands are synchronous: they return when the FC has ack'd or after
    a backend-internal timeout. Observations are polling-style: each call
    returns the most-recent value the backend has cached from MAVROS topics.
    """

    # ---- observations --------------------------------------------------

    @abstractmethod
    def is_connected(self) -> bool:
        """True iff MAVROS heartbeat is fresh per its own staleness window."""

    @abstractmethod
    def has_gps_fix(self) -> bool:
        """True iff GPS reports a 3D fix suitable for autonomous flight."""

    @abstractmethod
    def battery_pct(self) -> float:
        """Battery percentage in [0.0, 100.0]; -1.0 if unknown."""

    @abstractmethod
    def altitude_m(self) -> float:
        """Altitude above takeoff in meters; 0.0 on ground."""

    @abstractmethod
    def landed_state_on_ground(self) -> bool:
        """True iff mavros reports landed_state=ON_GROUND."""

    # ---- commands ------------------------------------------------------

    @abstractmethod
    def arm(self) -> bool:
        """Send arm command, wait for ack. Returns True on success."""

    @abstractmethod
    def takeoff(self, target_altitude_m: float) -> bool:
        """Send takeoff command to target altitude AGL, wait for ack."""

    @abstractmethod
    def land(self) -> bool:
        """Send land command, wait for ack. Drone descends at current lat/lon."""

    @abstractmethod
    def disarm(self) -> bool:
        """Send disarm command, wait for ack. Should only be called when
        landed_state_on_ground is True."""


# ---------------------------------------------------------------------------
# Mock backend
# ---------------------------------------------------------------------------


@dataclass
class MockBackendConfig:
    """Programmable behavior knobs for MockMavrosBackend.

    Defaults reflect a nominal mission: arm + takeoff succeed, climb and
    descend at 1.5 m/s, capture takes 1 s, battery starts at 95%.
    Tests override these for failure-path coverage.
    """
    connected: bool = True
    gps_fix: bool = True
    initial_battery_pct: float = 95.0
    climb_rate_mps: float = 1.5
    descent_rate_mps: float = 1.5
    arm_succeeds: bool = True
    takeoff_succeeds: bool = True
    arm_latency_s: float = 0.05
    takeoff_latency_s: float = 0.05
    land_latency_s: float = 0.05
    disarm_latency_s: float = 0.05
    # Battery drain per second of flight time.
    battery_drain_pct_per_s: float = 0.05


class MockMavrosBackend(MavrosBackend):
    """In-process drone simulator.

    Models altitude as a thread that moves toward a target at the
    configured rate. The mission coordinator polls altitude_m() and
    landed_state_on_ground() and reacts; the backend mirrors what a real
    FC would expose on /mavros/global_position/rel_alt and
    /mavros/extended_state.

    Thread-safe: the coordinator calls observations from one thread and
    commands from the same thread sequentially, but the altitude updater
    runs in the background.
    """

    def __init__(self, cfg: MockBackendConfig | None = None) -> None:
        self._cfg = cfg or MockBackendConfig()
        self._lock = threading.Lock()
        self._altitude_m = 0.0
        self._target_altitude_m = 0.0
        self._battery_pct = self._cfg.initial_battery_pct
        self._stop = threading.Event()
        self._is_flying = False
        self._update_thread = threading.Thread(
            target=self._update_loop, daemon=True,
        )
        self._update_thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        self._update_thread.join(timeout=1.0)

    # Test hooks (not part of MavrosBackend interface).

    def force_disconnect(self) -> None:
        with self._lock:
            self._cfg.connected = False

    def force_battery_pct(self, pct: float) -> None:
        with self._lock:
            self._battery_pct = pct

    def snapshot_altitude(self) -> float:
        with self._lock:
            return self._altitude_m

    # ---- observations -------------------------------------------------

    def is_connected(self) -> bool:
        with self._lock:
            return self._cfg.connected

    def has_gps_fix(self) -> bool:
        with self._lock:
            return self._cfg.gps_fix

    def battery_pct(self) -> float:
        with self._lock:
            return self._battery_pct

    def altitude_m(self) -> float:
        with self._lock:
            return self._altitude_m

    def landed_state_on_ground(self) -> bool:
        # mavros /extended_state.landed_state goes ON_GROUND based on
        # altitude and motion, NOT on arming state. Don't gate on
        # _is_flying — the coordinator polls this *before* disarming.
        with self._lock:
            return self._altitude_m < 0.2

    # ---- commands -----------------------------------------------------

    def arm(self) -> bool:
        time.sleep(self._cfg.arm_latency_s)
        with self._lock:
            if not self._cfg.arm_succeeds:
                return False
            self._is_flying = True   # motors idling (counts as flight for failsafe)
            return True

    def takeoff(self, target_altitude_m: float) -> bool:
        time.sleep(self._cfg.takeoff_latency_s)
        with self._lock:
            if not self._cfg.takeoff_succeeds:
                return False
            self._target_altitude_m = float(target_altitude_m)
            return True

    def land(self) -> bool:
        time.sleep(self._cfg.land_latency_s)
        with self._lock:
            self._target_altitude_m = 0.0
            return True

    def disarm(self) -> bool:
        time.sleep(self._cfg.disarm_latency_s)
        with self._lock:
            self._is_flying = False
            return True

    # ---- background update --------------------------------------------

    def _update_loop(self) -> None:
        last = time.monotonic()
        period = 0.05  # 20 Hz simulation tick
        while not self._stop.is_set():
            time.sleep(period)
            now = time.monotonic()
            dt = now - last
            last = now
            with self._lock:
                # Altitude tracks toward target at configured rate.
                if self._altitude_m < self._target_altitude_m:
                    delta = self._cfg.climb_rate_mps * dt
                    self._altitude_m = min(
                        self._target_altitude_m, self._altitude_m + delta,
                    )
                elif self._altitude_m > self._target_altitude_m:
                    delta = self._cfg.descent_rate_mps * dt
                    self._altitude_m = max(
                        self._target_altitude_m, self._altitude_m - delta,
                    )
                # Battery drains while motors are spinning.
                if self._is_flying:
                    self._battery_pct = max(
                        0.0,
                        self._battery_pct - self._cfg.battery_drain_pct_per_s * dt,
                    )


# ---------------------------------------------------------------------------
# Real backend (scaffold)
# ---------------------------------------------------------------------------


class RealMavrosBackend(MavrosBackend):
    """Talks to actual mavros. SCAFFOLD ONLY at M3 — no drone is wired up.

    Each method raises NotImplementedError with a pointer to where the
    real implementation will go. This exists so launching with the wrong
    backend by accident fails fast with a clear message instead of
    misbehaving silently.
    """

    def __init__(self, node) -> None:
        self._node = node
        node.get_logger().warning(
            'RealMavrosBackend constructed but iteration-1 methods are '
            'NotImplementedError. Use mavros_backend:=mock for bench work; '
            'switch to real once a drone is on the bench and the mavros '
            'service/topic plumbing is wired in.'
        )

    def is_connected(self) -> bool:
        raise NotImplementedError('subscribe /mavros/state.connected')

    def has_gps_fix(self) -> bool:
        raise NotImplementedError('subscribe /mavros/global_position/global; check status.status')

    def battery_pct(self) -> float:
        raise NotImplementedError('subscribe /mavros/battery; multiply percentage by 100')

    def altitude_m(self) -> float:
        raise NotImplementedError('subscribe /mavros/global_position/rel_alt')

    def landed_state_on_ground(self) -> bool:
        raise NotImplementedError('subscribe /mavros/extended_state.landed_state')

    def arm(self) -> bool:
        raise NotImplementedError('call /mavros/cmd/arming with value=True')

    def takeoff(self, target_altitude_m: float) -> bool:
        raise NotImplementedError('call /mavros/cmd/takeoff with altitude=target_altitude_m')

    def land(self) -> bool:
        raise NotImplementedError('call /mavros/cmd/land')

    def disarm(self) -> bool:
        raise NotImplementedError('call /mavros/cmd/arming with value=False')


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def make_mavros_backend(name: str, node) -> MavrosBackend:
    if name == 'mock':
        return MockMavrosBackend()
    if name == 'real':
        return RealMavrosBackend(node)
    raise ValueError(
        f"unknown mavros_backend: {name!r} (expected 'mock' or 'real')"
    )
