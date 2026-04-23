"""Unit tests for imu_driver.

Covers:
  - Madgwick filter at-rest stability and tilt response (pure-function)
  - Calibration YAML loading + SRS-HAL-002-F02 missing-file refusal
  - ISM330DHCX backend raw→physical scaling
  - SRS-HAL-002-S01 I²C-fault suspends publication without emitting stale values
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from platform_hal.imu_backend import Ism330Backend, MockImuBackend
from platform_hal.imu_driver import (
    ImuCalibrationMissing,
    ImuDriver,
    MadgwickFilter,
    _load_calibration,
)


FIXTURES = Path(__file__).parent / 'fixtures'
STUB_CALIB = FIXTURES / 'imu_calibration_stub.yaml'


# ---------------------------------------------------------------------------
# Madgwick filter — pure-function tests (no rclpy needed).
# ---------------------------------------------------------------------------

class TestMadgwickFilter:

    def test_identity_is_fixed_point_at_level_rest(self):
        # At rest, level (+g on z in REP-103), zero gyro → identity quaternion
        # is a fixed point. After many steps the filter must stay at identity.
        f = MadgwickFilter(beta=0.1)
        for _ in range(500):
            f.update(gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 9.80665), dt=0.01)
        qw, qx, qy, qz = f.quaternion
        assert qw == pytest.approx(1.0, abs=1e-6)
        assert qx == pytest.approx(0.0, abs=1e-6)
        assert qy == pytest.approx(0.0, abs=1e-6)
        assert qz == pytest.approx(0.0, abs=1e-6)

    def test_quaternion_stays_normalized(self):
        f = MadgwickFilter(beta=0.1)
        # Drive with a non-trivial gyro and tilted accel to stress the integrator.
        for _ in range(200):
            f.update(gyro=(0.05, 0.02, 0.01), accel=(1.0, 0.5, 9.0), dt=0.01)
        q = f.quaternion
        norm = math.sqrt(sum(x * x for x in q))
        assert norm == pytest.approx(1.0, abs=1e-6)

    def test_reset_restores_identity(self):
        f = MadgwickFilter(beta=0.1)
        for _ in range(100):
            f.update(gyro=(0.1, 0.0, 0.0), accel=(2.0, 0.0, 9.0), dt=0.01)
        f.reset()
        assert f.quaternion == (1.0, 0.0, 0.0, 0.0)

    def test_beta_zero_is_pure_gyro_integration(self):
        # With beta=0 the filter integrates gyro only; accel is ignored.
        # Rotating about z at 1 rad/s for 1 s → ~0.5 rad half-angle, so
        # q = (cos 0.5, 0, 0, sin 0.5).
        f = MadgwickFilter(beta=0.0)
        dt = 0.001
        for _ in range(1000):
            f.update(gyro=(0.0, 0.0, 1.0), accel=(0.0, 0.0, 9.80665), dt=dt)
        qw, qx, qy, qz = f.quaternion
        assert qw == pytest.approx(math.cos(0.5), abs=1e-3)
        assert qz == pytest.approx(math.sin(0.5), abs=1e-3)
        assert qx == pytest.approx(0.0, abs=1e-4)
        assert qy == pytest.approx(0.0, abs=1e-4)


# ---------------------------------------------------------------------------
# Calibration loader — pure-function tests.
# ---------------------------------------------------------------------------

class TestCalibrationLoader:

    def test_loads_stub_fixture(self):
        calib = _load_calibration(STUB_CALIB)
        assert calib['gyro_bias'] == (0.0, 0.0, 0.0)
        assert calib['accel_bias'] == (0.0, 0.0, 0.0)
        assert calib['accel_scale'] == (1.0, 1.0, 1.0)

    def test_missing_file_raises_imu_calibration_missing(self, tmp_path):
        missing = tmp_path / 'nope.yaml'
        with pytest.raises(ImuCalibrationMissing):
            _load_calibration(missing)

    def test_partial_yaml_fills_in_neutral_defaults(self, tmp_path):
        partial = tmp_path / 'partial.yaml'
        partial.write_text('gyro_bias: [0.01, 0.02, 0.03]\n')
        calib = _load_calibration(partial)
        assert calib['gyro_bias'] == (0.01, 0.02, 0.03)
        assert calib['accel_bias'] == (0.0, 0.0, 0.0)
        assert calib['accel_scale'] == (1.0, 1.0, 1.0)


# ---------------------------------------------------------------------------
# ISM330DHCX backend — two's-complement helper is the only hardware-free path.
# ---------------------------------------------------------------------------

class TestIsm330S16:

    def test_positive(self):
        assert Ism330Backend._s16(0x64, 0x00) == 100

    def test_negative(self):
        assert Ism330Backend._s16(0x9C, 0xFF) == -100

    def test_zero(self):
        assert Ism330Backend._s16(0x00, 0x00) == 0

    def test_full_range(self):
        assert Ism330Backend._s16(0xFF, 0x7F) == 32767
        assert Ism330Backend._s16(0x00, 0x80) == -32768


# ---------------------------------------------------------------------------
# Node-level tests with mock backend.
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_driver():
    """An ImuDriver with imu_backend=mock and the stub calibration fixture."""
    context = Context()
    rclpy.init(context=context)
    overrides = [
        Parameter('imu_backend', Parameter.Type.STRING, 'mock'),
        Parameter('calibration_file', Parameter.Type.STRING, str(STUB_CALIB)),
    ]
    node = ImuDriver(context=context, parameter_overrides=overrides)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


class TestImuDriverConstruction:

    def test_missing_calibration_refuses_to_start(self, tmp_path):
        """SRS-HAL-002-F02: ImuDriver must raise ImuCalibrationMissing."""
        context = Context()
        rclpy.init(context=context)
        try:
            missing = tmp_path / 'absent.yaml'
            overrides = [
                Parameter('imu_backend', Parameter.Type.STRING, 'mock'),
                Parameter('calibration_file', Parameter.Type.STRING, str(missing)),
            ]
            with pytest.raises(ImuCalibrationMissing):
                ImuDriver(context=context, parameter_overrides=overrides)
        finally:
            rclpy.shutdown(context=context)

    def test_starts_with_mock_backend_and_stub_calibration(self, mock_driver):
        assert mock_driver._rate_hz == 100.0
        assert mock_driver._filter.quaternion == (1.0, 0.0, 0.0, 0.0)


def _capture_publishes(node) -> list[Imu]:
    """Replace node._pub.publish with a list-append sink.

    Bypasses DDS entirely so tests don't need an executor bound to the
    isolated rclpy Context used by the `mock_driver` fixture.
    """
    captured: list[Imu] = []
    node._pub.publish = captured.append  # type: ignore[method-assign]
    return captured


class TestImuDriverPublishPath:
    """Drive _sample_and_publish() directly and assert on published Imu msgs."""

    def test_single_sample_publishes_imu_msg(self, mock_driver):
        captured = _capture_publishes(mock_driver)
        mock_driver._sample_and_publish()
        assert len(captured) == 1
        msg = captured[0]
        assert msg.header.frame_id == 'imu_link'
        # Default mock: +g on z, zero gyro — level rest.
        assert msg.linear_acceleration.z == pytest.approx(9.80665, abs=1e-6)
        assert msg.angular_velocity.x == 0.0

    def test_i2c_fault_suspends_publication(self, mock_driver):
        # SRS-HAL-002-S01: on I²C fault, stop publishing (no stale values).
        captured = _capture_publishes(mock_driver)
        backend = mock_driver._backend
        assert isinstance(backend, MockImuBackend)

        backend.fail_next_read()
        mock_driver._sample_and_publish()
        assert captured == []
        assert mock_driver._consecutive_faults == 1

        # Recovery: next read succeeds → publication resumes and counter clears.
        mock_driver._sample_and_publish()
        assert len(captured) == 1
        assert mock_driver._consecutive_faults == 0

    def test_persistent_i2c_fault_resets_filter(self, mock_driver):
        # Drive filter to a non-identity state, then sustain faults long
        # enough that _sample_and_publish() resets Madgwick to identity.
        mock_driver._filter.update(
            gyro=(0.1, 0.0, 0.0), accel=(1.0, 0.0, 9.0), dt=0.01,
        )
        assert mock_driver._filter.quaternion != (1.0, 0.0, 0.0, 0.0)

        for _ in range(mock_driver._reset_filter_after_n_faults):
            mock_driver._backend.fail_next_read()
            mock_driver._sample_and_publish()
        assert mock_driver._filter.quaternion == (1.0, 0.0, 0.0, 0.0)

    def test_calibration_subtracts_gyro_bias(self, tmp_path):
        """Raw gyro=(1,2,3), gyro_bias=(0.1,0.2,0.3) → published gyro=(0.9,1.8,2.7)."""
        calib = tmp_path / 'biased.yaml'
        calib.write_text(
            'gyro_bias: [0.1, 0.2, 0.3]\n'
            'accel_bias: [0.0, 0.0, 0.0]\n'
            'accel_scale: [1.0, 1.0, 1.0]\n'
        )
        context = Context()
        rclpy.init(context=context)
        try:
            overrides = [
                Parameter('imu_backend', Parameter.Type.STRING, 'mock'),
                Parameter('calibration_file', Parameter.Type.STRING, str(calib)),
            ]
            node = ImuDriver(context=context, parameter_overrides=overrides)
            try:
                captured = _capture_publishes(node)
                node._backend.set_sample(  # type: ignore[attr-defined]
                    accel=(0.0, 0.0, 9.80665),
                    gyro=(1.0, 2.0, 3.0),
                )
                node._sample_and_publish()
                assert len(captured) == 1
                assert captured[0].angular_velocity.x == pytest.approx(0.9)
                assert captured[0].angular_velocity.y == pytest.approx(1.8)
                assert captured[0].angular_velocity.z == pytest.approx(2.7)
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown(context=context)
