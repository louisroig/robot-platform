#!/usr/bin/env python3
"""Hardware probe: drive each track each direction in isolation.

Bypasses ROS / motor_driver and pokes lgpio directly. Sequence is
left-fwd, right-fwd, left-bwd, right-bwd, with a brief gap between
phases. Pin map and chip default match HW-PI5-001 / SRS-HAL-001 rev 0.4.

Usage:
    scripts/motor_check.py [--duty 70] [--dwell 1.0] [--gap 0.5]
                           [--gpio-chip 4] [--frequency 2000]

Stop with Ctrl-C; pins zero on any exit path.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

import lgpio

# HW-PI5-001 §3 pin freeze.
RPWM_LEFT = 12
LPWM_LEFT = 13
RPWM_RIGHT = 18
LPWM_RIGHT = 19


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duty', type=float, default=70.0,
                   help='PWM duty %% per active channel (0..100, default 70)')
    p.add_argument('--dwell', type=float, default=1.0,
                   help='seconds to drive each phase (default 1.0)')
    p.add_argument('--gap', type=float, default=0.5,
                   help='seconds at zero between phases (default 0.5)')
    p.add_argument('--gpio-chip', type=int, default=4,
                   help='gpiochip number (Pi 5 RP1=4, Pi 4=0; default 4)')
    p.add_argument('--frequency', type=int, default=2000,
                   help='PWM frequency Hz (default 2000)')
    return p.parse_args()


def main() -> int:
    args = parse_args()
    duty = max(0.0, min(100.0, args.duty))

    handle = lgpio.gpiochip_open(args.gpio_chip)
    pins = (RPWM_LEFT, LPWM_LEFT, RPWM_RIGHT, LPWM_RIGHT)

    def all_zero() -> None:
        for p in pins:
            lgpio.tx_pwm(handle, p, args.frequency, 0)

    def cleanup(*_: object) -> None:
        all_zero()
        lgpio.gpiochip_close(handle)

    signal.signal(signal.SIGINT, lambda *_: (cleanup(), sys.exit(130)))
    signal.signal(signal.SIGTERM, lambda *_: (cleanup(), sys.exit(143)))

    # Initialise every pin at 0% so the first tx_pwm doesn't glitch.
    all_zero()

    print(f"motor_check: chip={args.gpio_chip} freq={args.frequency}Hz "
          f"duty={duty:.0f}% dwell={args.dwell}s gap={args.gap}s")
    print(f"  pins: L_RPWM={RPWM_LEFT}  L_LPWM={LPWM_LEFT}  "
          f"R_RPWM={RPWM_RIGHT}  R_LPWM={LPWM_RIGHT}")

    phases = [
        ('LEFT  forward ', RPWM_LEFT),
        ('RIGHT forward ', RPWM_RIGHT),
        ('LEFT  backward', LPWM_LEFT),
        ('RIGHT backward', LPWM_RIGHT),
    ]

    try:
        for label, active_pin in phases:
            print(f"  → {label}  (GPIO {active_pin} @ {duty:.0f}%)")
            lgpio.tx_pwm(handle, active_pin, args.frequency, duty)
            time.sleep(args.dwell)
            lgpio.tx_pwm(handle, active_pin, args.frequency, 0)
            time.sleep(args.gap)
        print("motor_check: done")
        return 0
    finally:
        cleanup()


if __name__ == '__main__':
    sys.exit(main())
