#!/usr/bin/env python3
"""Hold one IBT-2 PWM channel high until Ctrl-C — sysfs hardware PWM.

For single-motor bench bringup. Drives the kernel hardware PWM exposed by
the `pwm-2chan` overlay on the Pi 5 RP1. lgpio software PWM is silently
broken on Pi 5, so we go through /sys/class/pwm instead.

Phase 1 wiring (left motor only):
    config.txt: dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
        pwmchip0/pwm0 -> GPIO 12 (L fwd / RPWM)
        pwmchip0/pwm1 -> GPIO 13 (L bwd / LPWM)

Right motor (GPIO 18/19) needs a different overlay choice — phase 2.

Usage:
    sudo scripts/motor_one.py --side L --dir fwd
    sudo scripts/motor_one.py --side L --dir bwd --duty 50

Sysfs writes typically need root; sudo is the simplest path until we add
a udev rule.
"""
from __future__ import annotations

import argparse
import errno
import signal
import sys
import time
from pathlib import Path

# (side, direction) -> (pwmchip index, channel index, GPIO for human reference)
CHANNELS = {
    ('L', 'fwd'): (0, 0, 12),
    ('L', 'bwd'): (0, 1, 13),
}


def chip_path(chip: int) -> Path:
    return Path(f'/sys/class/pwm/pwmchip{chip}')


def channel_path(chip: int, channel: int) -> Path:
    return chip_path(chip) / f'pwm{channel}'


def write(path: Path, value: str) -> None:
    path.write_text(value)


def export(chip: int, channel: int) -> None:
    if channel_path(chip, channel).exists():
        return
    try:
        write(chip_path(chip) / 'export', str(channel))
    except OSError as e:
        if e.errno != errno.EBUSY:
            raise
    # Wait briefly for udev / kernel to populate the channel directory.
    for _ in range(50):
        if channel_path(chip, channel).exists():
            return
        time.sleep(0.01)
    raise RuntimeError(f'pwm channel {channel} on chip {chip} did not appear after export')


def unexport(chip: int, channel: int) -> None:
    if not channel_path(chip, channel).exists():
        return
    try:
        write(chip_path(chip) / 'unexport', str(channel))
    except OSError:
        pass


def disable(chip: int, channel: int) -> None:
    cpath = channel_path(chip, channel)
    if not cpath.exists():
        return
    try:
        write(cpath / 'enable', '0')
        write(cpath / 'duty_cycle', '0')
    except OSError:
        pass


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--side', choices=['L', 'R'], required=True)
    p.add_argument('--dir', choices=['fwd', 'bwd'], required=True, dest='direction')
    p.add_argument('--duty', type=float, default=70.0, help='PWM duty %% (default 70)')
    p.add_argument('--frequency', type=int, default=2000, help='PWM Hz (default 2000)')
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if (args.side, args.direction) not in CHANNELS:
        print(f'ERROR: side={args.side} dir={args.direction} not yet wired to a PWM channel.\n'
              f'       Phase 1 supports L only. R needs a phase-2 overlay.', file=sys.stderr)
        return 2

    duty_pct = max(0.0, min(100.0, args.duty))
    chip, channel, gpio = CHANNELS[(args.side, args.direction)]
    period_ns = int(round(1_000_000_000 / args.frequency))
    duty_ns = int(round(period_ns * duty_pct / 100.0))

    if not chip_path(chip).exists():
        print(f'ERROR: {chip_path(chip)} not found. Did the pwm-2chan overlay load?\n'
              f'       Check /boot/firmware/config.txt and dmesg | grep -i pwm.', file=sys.stderr)
        return 3

    # Drive only the requested channel; force the opposite-direction channel
    # for the same side off — H-bridge short prohibited.
    other_dir = 'bwd' if args.direction == 'fwd' else 'fwd'
    other_key = (args.side, other_dir)

    def cleanup(*_: object) -> None:
        disable(chip, channel)
        if other_key in CHANNELS:
            o_chip, o_chan, _ = CHANNELS[other_key]
            disable(o_chip, o_chan)
        unexport(chip, channel)
        if other_key in CHANNELS:
            o_chip, o_chan, _ = CHANNELS[other_key]
            unexport(o_chip, o_chan)

    signal.signal(signal.SIGINT, lambda *_: (cleanup(), sys.exit(130)))
    signal.signal(signal.SIGTERM, lambda *_: (cleanup(), sys.exit(143)))

    if other_key in CHANNELS:
        o_chip, o_chan, _ = CHANNELS[other_key]
        export(o_chip, o_chan)
        disable(o_chip, o_chan)

    export(chip, channel)
    cpath = channel_path(chip, channel)
    # On a freshly-exported channel the RP1 driver returns EINVAL for any
    # write before `period` is set — including enable=0. Set period first.
    # Then duty (must be <= period), then enable.
    write(cpath / 'period', str(period_ns))
    write(cpath / 'duty_cycle', str(duty_ns))
    write(cpath / 'enable', '1')

    print(f'motor_one (sysfs): pwmchip{chip}/pwm{channel}  GPIO {gpio}  ({args.side} {args.direction})')
    print(f'  period={period_ns} ns ({args.frequency} Hz)  duty={duty_ns} ns ({duty_pct:.0f}%)')
    print('  Ctrl-C to stop.')

    try:
        while True:
            time.sleep(1.0)
    finally:
        cleanup()


if __name__ == '__main__':
    sys.exit(main())
