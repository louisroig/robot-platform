"""Unit tests for the pure-math part of imu_calibrate.

Only `compute_accel_calibration` is tested here — the I/O loop and prompt
flow are bench-only and exercised by hand. The math is the part where
sign or scale errors would silently corrupt every IMU reading downstream.
"""
from __future__ import annotations

import math

import pytest

from platform_hal.imu_calibrate import (
    AxisMeans,
    G_MPS2,
    compute_accel_calibration,
)


def _ideal(g: float = G_MPS2) -> AxisMeans:
    """Six perfect readings: pure +/-g on the active axis, 0 elsewhere."""
    return AxisMeans(
        z_pos=(0.0, 0.0,  +g),
        z_neg=(0.0, 0.0,  -g),
        x_pos=( +g, 0.0, 0.0),
        x_neg=( -g, 0.0, 0.0),
        y_pos=(0.0,  +g, 0.0),
        y_neg=(0.0,  -g, 0.0),
    )


class TestComputeAccelCalibration:
    def test_ideal_readings_yield_zero_bias_unit_scale(self):
        bias, scale = compute_accel_calibration(_ideal())
        assert bias == pytest.approx((0.0, 0.0, 0.0))
        assert scale == pytest.approx((1.0, 1.0, 1.0))

    def test_symmetric_bias_recovered(self):
        # Inject a constant +0.5 m/s² bias on every axis-aligned reading.
        # corrected = (raw - bias) * scale, so raw = bias + truth/scale.
        # With scale=1 and bias=0.5: raw_pos = 0.5 + g, raw_neg = 0.5 - g.
        bx, by, bz = 0.5, -0.3, 0.1
        means = AxisMeans(
            z_pos=(0.0, 0.0, bz + G_MPS2),
            z_neg=(0.0, 0.0, bz - G_MPS2),
            x_pos=(bx + G_MPS2, 0.0, 0.0),
            x_neg=(bx - G_MPS2, 0.0, 0.0),
            y_pos=(0.0, by + G_MPS2, 0.0),
            y_neg=(0.0, by - G_MPS2, 0.0),
        )
        bias, scale = compute_accel_calibration(means)
        assert bias == pytest.approx((bx, by, bz))
        assert scale == pytest.approx((1.0, 1.0, 1.0))

    def test_scale_recovered_when_span_off(self):
        # Sensor reads 90% of true gravity ⇒ measured span is 1.8g, so
        # scale should come out to 1/0.9 ≈ 1.1111 to compensate.
        s_true = 0.9
        means = AxisMeans(
            z_pos=(0.0, 0.0, +G_MPS2 * s_true),
            z_neg=(0.0, 0.0, -G_MPS2 * s_true),
            x_pos=(+G_MPS2 * s_true, 0.0, 0.0),
            x_neg=(-G_MPS2 * s_true, 0.0, 0.0),
            y_pos=(0.0, +G_MPS2 * s_true, 0.0),
            y_neg=(0.0, -G_MPS2 * s_true, 0.0),
        )
        _, scale = compute_accel_calibration(means)
        assert scale == pytest.approx((1.0 / s_true, 1.0 / s_true, 1.0 / s_true))

    def test_calibration_inverse_recovers_truth(self):
        # End-to-end check: synthesize raws from a known (bias, scale),
        # solve for them, then apply the inverse and confirm we get +/-g.
        bias = (0.4, -0.2, 0.05)
        scale_inv = (0.95, 1.03, 0.98)  # raw = bias + truth * scale_inv
        means = AxisMeans(
            z_pos=(bias[0],          bias[1],          bias[2] + G_MPS2 * scale_inv[2]),
            z_neg=(bias[0],          bias[1],          bias[2] - G_MPS2 * scale_inv[2]),
            x_pos=(bias[0] + G_MPS2 * scale_inv[0], bias[1], bias[2]),
            x_neg=(bias[0] - G_MPS2 * scale_inv[0], bias[1], bias[2]),
            y_pos=(bias[0], bias[1] + G_MPS2 * scale_inv[1], bias[2]),
            y_neg=(bias[0], bias[1] - G_MPS2 * scale_inv[1], bias[2]),
        )
        recovered_bias, recovered_scale = compute_accel_calibration(means)
        assert recovered_bias == pytest.approx(bias)
        # imu_driver applies (raw - bias) * scale; for a +g reading on the
        # active axis the result should land on G_MPS2 to within rounding.
        for axis_idx, raw in enumerate([means.x_pos, means.y_pos, means.z_pos]):
            corrected = (raw[axis_idx] - recovered_bias[axis_idx]) * recovered_scale[axis_idx]
            assert corrected == pytest.approx(G_MPS2)

    def test_negative_span_rejected(self):
        # If the operator swaps a +/- pair the span goes negative — catch
        # it loudly, not by silently emitting a negative scale that would
        # mirror the axis at runtime.
        means = AxisMeans(
            z_pos=(0.0, 0.0, -G_MPS2),
            z_neg=(0.0, 0.0, +G_MPS2),
            x_pos=(+G_MPS2, 0.0, 0.0),
            x_neg=(-G_MPS2, 0.0, 0.0),
            y_pos=(0.0, +G_MPS2, 0.0),
            y_neg=(0.0, -G_MPS2, 0.0),
        )
        with pytest.raises(ValueError, match='span'):
            compute_accel_calibration(means)
