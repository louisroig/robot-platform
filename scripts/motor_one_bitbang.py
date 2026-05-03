#!/usr/bin/env python3
"""Bit-bang PWM on one IBT-2 channel via lgpio.gpio_write — bench bringup only.

Why this exists: on this Pi 5 / Ubuntu 24.04 stack, lgpio.tx_pwm silently
no-ops, and sysfs hardware PWM on pwmchip0 (RP1) fails with EINVAL because
the rpi-pwm driver can't resolve its clock rate. Static gpio_write *does*
work, so we toggle in a Python loop. ~1 kHz, jittery but enough to spin
a motor on the bench. Not a production motor_driver pattern — production
will need PCA9685 over I²C (or equivalent) once the architecture is decided.

Usage:
    scripts/motor_one_bitbang.py --side L --dir fwd
    scripts/motor_one_bitbang.py --side L --dir bwd --duty 50
    scripts/motor_one_bitbang.py --side both --dir fwd     # drives L+R together
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import lgpio

PINS = {
    ('L', 'fwd'): 12,
    ('L', 'bwd'): 13,
    ('R', 'fwd'): 18,
    ('R', 'bwd'): 19,
}

GPIO_CHIP = 4
PERIOD_S = 0.001  # 1 kHz — H-bridge tolerates 0.5-25 kHz


def unexport_lingering_pwm() -> None:
    # Earlier sysfs PWM experiments may have left pwm0/pwm1 exported.
    # Release them so the GPIO lines aren't contended by anything else.
    for ch in (0, 1):
        if Path(f'/sys/class/pwm/pwmchip0/pwm{ch}').exists():
            try:
                Path('/sys/class/pwm/pwmchip0/unexport').write_text(str(ch))
            except OSError:
                pass


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--side', choices=['L', 'R', 'both'], required=True)
    p.add_argument('--dir', choices=['fwd', 'bwd'], required=True, dest='direction')
    p.add_argument('--duty', type=float, default=70.0, help='PWM duty %% (default 70)')
    return p.parse_args()


def main() -> int:
    args = parse_args()
    duty_frac = max(0.0, min(100.0, args.duty)) / 100.0
    sides = ['L', 'R'] if args.side == 'both' else [args.side]
    driven = [PINS[(s, args.direction)] for s in sides]
    high_s = PERIOD_S * duty_frac
    low_s = PERIOD_S - high_s

    unexport_lingering_pwm()
    handle = lgpio.gpiochip_open(GPIO_CHIP)
    for p in PINS.values():
        lgpio.gpio_claim_output(handle, p, 0)

    print(f'motor_one_bitbang: GPIO {driven} ({args.side} {args.direction})')
    print(f'  ~{int(1/PERIOD_S)} Hz, duty {duty_frac*100:.0f}% '
          f'({high_s*1e6:.0f} us high / {low_s*1e6:.0f} us low)')
    print(f'  other pins held LOW: {sorted(p for p in PINS.values() if p not in driven)}')
    print('  Ctrl-C to stop.')

    try:
        if duty_frac >= 1.0:
            for pin in driven:
                lgpio.gpio_write(handle, pin, 1)
            while True:
                time.sleep(1.0)
        elif duty_frac <= 0.0:
            while True:
                time.sleep(1.0)
        else:
            while True:
                for pin in driven:
                    lgpio.gpio_write(handle, pin, 1)
                time.sleep(high_s)
                for pin in driven:
                    lgpio.gpio_write(handle, pin, 0)
                time.sleep(low_s)
    except KeyboardInterrupt:
        pass
    finally:
        for p in PINS.values():
            try:
                lgpio.gpio_write(handle, p, 0)
            except Exception:
                pass
        lgpio.gpiochip_close(handle)
        print('\nstopped, all pins LOW')

    return 0


if __name__ == '__main__':
    sys.exit(main())
