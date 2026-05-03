"""Sanity tests for MockMavrosBackend — the in-process drone simulator.

The simulator is itself a non-trivial piece of code (background thread,
target-altitude tracking, battery drain). These tests give it a small
self-check independent of the coordinator so coordinator-test failures
unambiguously implicate the coordinator, not the sim.
"""

from __future__ import annotations

import time

from drone_mission.mavros_backend import (
    MockBackendConfig,
    MockMavrosBackend,
)


def _wait_until(predicate, timeout_s: float = 5.0, period_s: float = 0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return False


class TestMockBackend:
    def test_starts_on_ground_with_full_battery(self):
        b = MockMavrosBackend()
        try:
            assert b.altitude_m() == 0.0
            assert b.landed_state_on_ground() is True
            assert b.battery_pct() == 95.0
            assert b.is_connected() is True
            assert b.has_gps_fix() is True
        finally:
            b.shutdown()

    def test_arm_takeoff_climbs_to_target(self):
        b = MockMavrosBackend(MockBackendConfig(climb_rate_mps=10.0))
        try:
            assert b.arm() is True
            assert b.takeoff(20.0) is True
            assert _wait_until(lambda: abs(b.altitude_m() - 20.0) < 0.5), (
                f'altitude reached only {b.altitude_m():.2f}m of 20m'
            )
            assert b.landed_state_on_ground() is False
        finally:
            b.shutdown()

    def test_land_descends_and_returns_on_ground(self):
        b = MockMavrosBackend(MockBackendConfig(
            climb_rate_mps=10.0, descent_rate_mps=10.0,
        ))
        try:
            b.arm(); b.takeoff(10.0)
            _wait_until(lambda: b.altitude_m() >= 9.5)
            b.land()
            assert _wait_until(lambda: b.altitude_m() < 0.2), (
                f'descent stopped at {b.altitude_m():.2f}m'
            )
            b.disarm()
            assert b.landed_state_on_ground() is True
        finally:
            b.shutdown()

    def test_arm_failure_returns_false(self):
        b = MockMavrosBackend(MockBackendConfig(arm_succeeds=False))
        try:
            assert b.arm() is False
        finally:
            b.shutdown()

    def test_force_disconnect(self):
        b = MockMavrosBackend()
        try:
            assert b.is_connected() is True
            b.force_disconnect()
            assert b.is_connected() is False
        finally:
            b.shutdown()

    def test_battery_drains_while_flying(self):
        b = MockMavrosBackend(MockBackendConfig(
            climb_rate_mps=20.0, battery_drain_pct_per_s=10.0,
        ))
        try:
            b.arm(); b.takeoff(5.0)
            _wait_until(lambda: b.altitude_m() >= 4.5)
            start_pct = b.battery_pct()
            time.sleep(0.5)
            end_pct = b.battery_pct()
            # Drained ~5% in 0.5s at 10%/s; allow generous tolerance for
            # background-thread scheduling jitter.
            assert end_pct < start_pct - 1.0, (
                f'expected battery to drain noticeably; '
                f'start={start_pct}, end={end_pct}'
            )
        finally:
            b.shutdown()
