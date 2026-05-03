"""Interactive IMU calibration tool — produces imu_calibration.yaml.

Bench tool, not a ROS node. Talks to the ISM330DHCX directly over I²C
(reuses Ism330Backend from imu_backend) and walks the operator through:

  1. **Stationary capture** — keep the robot still for ~5 s; the mean gyro
     reading becomes `gyro_bias` (subtracted from every sample at runtime).
  2. **6-position accel walk** — orient each axis +/- toward the ground for
     ~3 s each; the six mean readings yield per-axis `accel_bias` and
     `accel_scale` so a level rest reads (0, 0, +g) after correction.

YAML schema matches what `imu_driver._load_calibration` consumes:

    gyro_bias:   [bx, by, bz]   # rad/s, subtracted from each gyro sample
    accel_bias:  [bx, by, bz]   # m/s², subtracted from each accel sample
    accel_scale: [sx, sy, sz]   # per-axis multiplicative correction

Default output path matches the imu_driver default
(`~/.config/platform/imu_calibration.yaml`). Writes atomically — the existing
file (if any) is preserved until the new one is fully written and renamed.

Run with::

    ros2 run platform_hal imu_calibrate
    ros2 run platform_hal imu_calibrate --gyro-seconds 10 --output ./imu_cal.yaml
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from platform_hal.imu_backend import Ism330Backend

G_MPS2 = Ism330Backend.G_MPS2
DEFAULT_OUTPUT = '~/.config/platform/imu_calibration.yaml'

# Six positions that put each body axis +/- toward the sky in turn.
# `axis` is the body-frame axis index (0=x, 1=y, 2=z); `sign` is the
# expected sign of the +g-aligned reading on that axis when the operator
# orients the robot correctly.
@dataclass(frozen=True)
class Position:
    label: str
    instruction: str
    axis: int
    sign: int  # +1 or -1


POSITIONS: tuple[Position, ...] = (
    Position('Z+', 'Place rover level on a flat surface (normal upright pose).',           2, +1),
    Position('Z-', 'Flip rover upside down (chassis bottom facing the sky).',              2, -1),
    Position('X+', 'Stand rover on its rear (front of chassis pointing up).',              0, +1),
    Position('X-', 'Stand rover on its nose (front of chassis pointing down).',            0, -1),
    Position('Y+', 'Lay rover on its right side (left side of chassis pointing up).',      1, +1),
    Position('Y-', 'Lay rover on its left side (right side of chassis pointing up).',      1, -1),
)


@dataclass(frozen=True)
class AxisMeans:
    z_pos: tuple[float, float, float]
    z_neg: tuple[float, float, float]
    x_pos: tuple[float, float, float]
    x_neg: tuple[float, float, float]
    y_pos: tuple[float, float, float]
    y_neg: tuple[float, float, float]


def compute_accel_calibration(
    means: AxisMeans, g: float = G_MPS2,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Six-position accel calibration.

    Per the model `corrected = (raw - bias) * scale`, when axis i is
    aligned with +gravity the corrected reading on axis i equals +g, and
    when aligned with -gravity it equals -g. Adding/subtracting the two
    raw readings gives bias = (raw_pos + raw_neg) / 2 and
    scale = 2g / (raw_pos - raw_neg). Off-axis readings during a single
    position are ignored (no cross-axis correction at this complexity tier).
    """
    bias_x = (means.x_pos[0] + means.x_neg[0]) / 2.0
    bias_y = (means.y_pos[1] + means.y_neg[1]) / 2.0
    bias_z = (means.z_pos[2] + means.z_neg[2]) / 2.0

    span_x = means.x_pos[0] - means.x_neg[0]
    span_y = means.y_pos[1] - means.y_neg[1]
    span_z = means.z_pos[2] - means.z_neg[2]

    if span_x <= 0 or span_y <= 0 or span_z <= 0:
        raise ValueError(
            f'non-positive accel span detected (x={span_x:.3f}, '
            f'y={span_y:.3f}, z={span_z:.3f}); positions likely swapped'
        )

    scale_x = (2.0 * g) / span_x
    scale_y = (2.0 * g) / span_y
    scale_z = (2.0 * g) / span_z

    return (bias_x, bias_y, bias_z), (scale_x, scale_y, scale_z)


def _capture_mean(backend, seconds: float, rate_hz: float, stream=sys.stdout
                 ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Sample for `seconds` at ~rate_hz, return (mean_accel, mean_gyro)."""
    n = max(1, int(round(seconds * rate_hz)))
    period = 1.0 / rate_hz
    sa = [0.0, 0.0, 0.0]
    sg = [0.0, 0.0, 0.0]
    next_tick = time.monotonic()
    for i in range(n):
        a, g = backend.read()
        sa[0] += a[0]; sa[1] += a[1]; sa[2] += a[2]
        sg[0] += g[0]; sg[1] += g[1]; sg[2] += g[2]
        if (i + 1) % max(1, n // 20) == 0:
            stream.write('.')
            stream.flush()
        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
    stream.write('\n')
    return (sa[0] / n, sa[1] / n, sa[2] / n), (sg[0] / n, sg[1] / n, sg[2] / n)


def _expand(path: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path)))


def _write_atomic(target: Path, content: str) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + '.tmp')
    tmp.write_text(content)
    os.replace(tmp, target)


def _confirm(prompt: str) -> bool:
    reply = input(f'{prompt} [y/N] ').strip().lower()
    return reply in ('y', 'yes')


def _wait_for_position(label: str, instruction: str) -> None:
    print(f'\n--- Position {label} ---')
    print(f'  {instruction}')
    input('  Press Enter when the rover is in position and stationary...')


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--i2c-bus', type=int, default=1)
    p.add_argument('--i2c-address', type=lambda s: int(s, 0), default=0x6B,
                   help='ISM330DHCX I²C address (0x6A or 0x6B; default 0x6B)')
    p.add_argument('--gyro-seconds', type=float, default=5.0,
                   help='Stationary capture duration for gyro bias (default 5)')
    p.add_argument('--accel-seconds', type=float, default=3.0,
                   help='Per-position capture duration for accel cal (default 3)')
    p.add_argument('--rate-hz', type=float, default=100.0,
                   help='Sample rate (default 100, matches imu_driver)')
    p.add_argument('--output', default=DEFAULT_OUTPUT,
                   help=f'Output YAML path (default {DEFAULT_OUTPUT})')
    p.add_argument('--force', action='store_true',
                   help='Overwrite output without confirmation')
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out_path = _expand(args.output)

    if out_path.exists() and not args.force:
        if not _confirm(f'{out_path} exists. Overwrite?'):
            print('aborted.')
            return 1

    print(f'Opening ISM330DHCX on i2c bus {args.i2c_bus} @ '
          f'0x{args.i2c_address:02x}...')
    backend = Ism330Backend()
    backend.setup(args.i2c_bus, args.i2c_address)

    try:
        # --- Step 1: gyro bias (stationary) ---
        print(f'\n=== Gyro bias ({args.gyro_seconds:.0f} s stationary) ===')
        print('Place rover on a stable, vibration-free surface and do not touch.')
        input('Press Enter when ready...')
        print('Capturing', end='')
        _, gyro_mean = _capture_mean(backend, args.gyro_seconds, args.rate_hz)
        print(f'  gyro_bias = ({gyro_mean[0]:+.5f}, {gyro_mean[1]:+.5f}, '
              f'{gyro_mean[2]:+.5f}) rad/s')

        # --- Step 2: 6-position accel cal ---
        print(f'\n=== Accel calibration (6 positions × {args.accel_seconds:.0f} s) ===')
        means: dict[str, tuple[float, float, float]] = {}
        for pos in POSITIONS:
            _wait_for_position(pos.label, pos.instruction)
            print('Capturing', end='')
            accel_mean, _ = _capture_mean(backend, args.accel_seconds, args.rate_hz)
            means[pos.label] = accel_mean
            expected_sign = '+' if pos.sign > 0 else '-'
            actual = accel_mean[pos.axis]
            sign_ok = (actual > 0) == (pos.sign > 0)
            warn = '' if sign_ok else '  ⚠ axis sign mismatch — orientation likely wrong'
            print(f'  axis-{["x","y","z"][pos.axis]} = {actual:+.3f} m/s² '
                  f'(expected {expected_sign}g ≈ {pos.sign * G_MPS2:+.3f}){warn}')

        axis_means = AxisMeans(
            z_pos=means['Z+'], z_neg=means['Z-'],
            x_pos=means['X+'], x_neg=means['X-'],
            y_pos=means['Y+'], y_neg=means['Y-'],
        )
        accel_bias, accel_scale = compute_accel_calibration(axis_means)
        print('\n  accel_bias  = '
              f'({accel_bias[0]:+.4f}, {accel_bias[1]:+.4f}, {accel_bias[2]:+.4f}) m/s²')
        print('  accel_scale = '
              f'({accel_scale[0]:.5f}, {accel_scale[1]:.5f}, {accel_scale[2]:.5f})')

        # --- Step 3: write YAML ---
        payload = {
            'gyro_bias':   [float(v) for v in gyro_mean],
            'accel_bias':  [float(v) for v in accel_bias],
            'accel_scale': [float(v) for v in accel_scale],
        }
        content = yaml.safe_dump(payload, default_flow_style=False, sort_keys=False)
        _write_atomic(out_path, content)
        print(f'\nWrote {out_path}')
        print('Restart imu_driver to pick up the new calibration.')
    finally:
        backend.cleanup()

    return 0


if __name__ == '__main__':
    sys.exit(main())
