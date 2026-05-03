"""GPIO backend abstraction for motor_driver.

Three implementations:
  - LgpioBackend  — Pi 5 hardware via `lgpio` (currently non-functional on
    Ubuntu 24.04 / RP1; tx_pwm silently no-ops). Kept for forward
    compatibility if the kernel/PWM situation is fixed.
  - Pca9685Backend — off-board PWM via PCA9685 16-channel I²C controller.
    Production path on this stack; sidesteps the broken RP1 PWM.
  - MockGpioBackend — publishes per-track signed PWM to /test/motor_pwm
    for integration tests with no hardware.

The motor_driver selects via the `gpio_backend` parameter.

Signalling convention is IBT-2 dual-PWM per SRS-HAL-001 rev 0.4 / HW-PI5-001:
each track has an RPWM (forward) and LPWM (reverse) channel, mutually
exclusive at every instant. The `write(left_signed, right_signed)`
interface takes duty in [-1.0, 1.0] — sign selects the channel, magnitude
sets the duty — and every backend enforces the mutual-exclusion invariant
by zeroing the opposite channel before activating the new one.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod


class GpioBackend(ABC):
    @abstractmethod
    def setup(self, rpwm_left_pin: int, lpwm_left_pin: int,
              rpwm_right_pin: int, lpwm_right_pin: int,
              pwm_frequency_hz: int, gpio_chip: int) -> None:
        ...

    @abstractmethod
    def write(self, left_signed: float, right_signed: float) -> None:
        """Drive both tracks. Inputs in [-1.0, 1.0]: sign = direction, magnitude = duty."""
        ...

    @abstractmethod
    def cleanup(self) -> None:
        ...


class LgpioBackend(GpioBackend):
    """Drives IBT-2 (BTS7960B) pairs via dual hardware PWM on Raspberry Pi 5."""

    def __init__(self) -> None:
        import lgpio  # lazy import — package must build without lgpio installed
        self._lgpio = lgpio
        self._handle: int | None = None
        self._rpwm_left: int | None = None
        self._lpwm_left: int | None = None
        self._rpwm_right: int | None = None
        self._lpwm_right: int | None = None
        self._frequency: int = 0

    def setup(self, rpwm_left_pin, lpwm_left_pin, rpwm_right_pin, lpwm_right_pin,
              pwm_frequency_hz, gpio_chip) -> None:
        self._handle = self._lgpio.gpiochip_open(gpio_chip)
        self._rpwm_left = rpwm_left_pin
        self._lpwm_left = lpwm_left_pin
        self._rpwm_right = rpwm_right_pin
        self._lpwm_right = lpwm_right_pin
        self._frequency = pwm_frequency_hz
        for pin in (rpwm_left_pin, lpwm_left_pin, rpwm_right_pin, lpwm_right_pin):
            self._lgpio.tx_pwm(self._handle, pin, pwm_frequency_hz, 0)

    def write(self, left_signed: float, right_signed: float) -> None:
        if self._handle is None:
            return
        self._write_track(self._rpwm_left, self._lpwm_left, left_signed)
        self._write_track(self._rpwm_right, self._lpwm_right, right_signed)

    def _write_track(self, rpwm_pin: int | None, lpwm_pin: int | None,
                     signed_duty: float) -> None:
        if rpwm_pin is None or lpwm_pin is None:
            return
        duty_pct = min(100.0, abs(signed_duty) * 100.0)
        # SRS-HAL-001-F03 mutual exclusion: zero the opposite channel BEFORE
        # activating the new one. The reverse order would briefly drive both
        # RPWM and LPWM non-zero, shorting the H-bridge.
        if signed_duty > 0:
            self._lgpio.tx_pwm(self._handle, lpwm_pin, self._frequency, 0)
            self._lgpio.tx_pwm(self._handle, rpwm_pin, self._frequency, duty_pct)
        elif signed_duty < 0:
            self._lgpio.tx_pwm(self._handle, rpwm_pin, self._frequency, 0)
            self._lgpio.tx_pwm(self._handle, lpwm_pin, self._frequency, duty_pct)
        else:
            self._lgpio.tx_pwm(self._handle, rpwm_pin, self._frequency, 0)
            self._lgpio.tx_pwm(self._handle, lpwm_pin, self._frequency, 0)

    def cleanup(self) -> None:
        if self._handle is None:
            return
        for pin in (self._rpwm_left, self._lpwm_left,
                    self._rpwm_right, self._lpwm_right):
            if pin is not None:
                self._lgpio.tx_pwm(self._handle, pin, self._frequency, 0)
        self._lgpio.gpiochip_close(self._handle)
        self._handle = None


class Pca9685Backend(GpioBackend):
    """Drives IBT-2 (BTS7960B) pairs via PCA9685 16-channel I²C PWM controller.

    For this backend the `*_pin` arguments to setup() are PCA9685 channel
    indices in [0, 15], not BCM GPIO numbers; `gpio_chip` is unused. The
    chip's PWM frequency is global across all 16 channels and is programmed
    once at setup. I²C bus and address come from node parameters
    (`i2c_bus`, `i2c_address`) so the chip can share a bus with the IMU.
    """

    REG_MODE1 = 0x00
    REG_PRESCALE = 0xFE
    REG_LED0_ON_L = 0x06
    BIT_SLEEP = 0x10
    BIT_AI = 0x20  # MODE1 auto-increment register address on read/write

    OSC_HZ = 25_000_000
    RESOLUTION = 4096       # 12-bit per channel
    FREQ_MIN_HZ = 24
    FREQ_MAX_HZ = 1526

    def __init__(self, node) -> None:
        import smbus2  # lazy — package must build without smbus2 installed
        self._smbus2 = smbus2
        self._node = node
        self._bus = None
        self._address = 0
        self._rpwm_left: int | None = None
        self._lpwm_left: int | None = None
        self._rpwm_right: int | None = None
        self._lpwm_right: int | None = None

    def setup(self, rpwm_left_pin, lpwm_left_pin, rpwm_right_pin, lpwm_right_pin,
              pwm_frequency_hz, gpio_chip) -> None:
        if not self.FREQ_MIN_HZ <= pwm_frequency_hz <= self.FREQ_MAX_HZ:
            raise ValueError(
                f'PCA9685 supports {self.FREQ_MIN_HZ}-{self.FREQ_MAX_HZ} Hz; '
                f'pwm_frequency_hz={pwm_frequency_hz} is out of range'
            )

        i2c_bus = int(self._node.get_parameter('i2c_bus').value)
        self._address = int(self._node.get_parameter('i2c_address').value)
        self._rpwm_left = rpwm_left_pin
        self._lpwm_left = lpwm_left_pin
        self._rpwm_right = rpwm_right_pin
        self._lpwm_right = lpwm_right_pin

        self._bus = self._smbus2.SMBus(i2c_bus)
        # Cheap presence probe — surfaces "PCA9685 not wired or wrong addr"
        # as a clear error during bringup, instead of an opaque failure
        # mid-setup.
        try:
            self._bus.read_byte_data(self._address, self.REG_MODE1)
        except OSError as exc:
            raise RuntimeError(
                f'no PCA9685 acking on i2c bus {i2c_bus} @ '
                f'0x{self._address:02x}: {exc}'
            ) from exc

        # Sleep → write prescale → wake → enable address auto-increment.
        # Per datasheet §7.3.1.1 the PRESCALE register is only writable
        # while SLEEP=1, and the oscillator needs ~500 µs to restart.
        prescale = round(self.OSC_HZ / (self.RESOLUTION * pwm_frequency_hz)) - 1
        prescale = max(3, min(255, prescale))
        self._bus.write_byte_data(self._address, self.REG_MODE1, self.BIT_SLEEP)
        self._bus.write_byte_data(self._address, self.REG_PRESCALE, prescale)
        self._bus.write_byte_data(self._address, self.REG_MODE1, 0x00)
        time.sleep(0.0005)
        self._bus.write_byte_data(self._address, self.REG_MODE1, self.BIT_AI)

        for ch in (rpwm_left_pin, lpwm_left_pin, rpwm_right_pin, lpwm_right_pin):
            self._set_channel_duty(ch, 0.0)

    def write(self, left_signed: float, right_signed: float) -> None:
        if self._bus is None:
            return
        self._write_track(self._rpwm_left, self._lpwm_left, left_signed)
        self._write_track(self._rpwm_right, self._lpwm_right, right_signed)

    def _write_track(self, rpwm_ch: int | None, lpwm_ch: int | None,
                     signed_duty: float) -> None:
        if rpwm_ch is None or lpwm_ch is None:
            return
        duty = min(1.0, abs(signed_duty))
        # SRS-HAL-001-F03 mutual exclusion: zero the opposite channel BEFORE
        # activating the new one.
        if signed_duty > 0:
            self._set_channel_duty(lpwm_ch, 0.0)
            self._set_channel_duty(rpwm_ch, duty)
        elif signed_duty < 0:
            self._set_channel_duty(rpwm_ch, 0.0)
            self._set_channel_duty(lpwm_ch, duty)
        else:
            self._set_channel_duty(rpwm_ch, 0.0)
            self._set_channel_duty(lpwm_ch, 0.0)

    def _set_channel_duty(self, channel: int, duty_frac: float) -> None:
        # Per-channel registers: ON_L, ON_H, OFF_L, OFF_H starting at
        # LED0_ON_L + 4*channel. ON=0 means rising edge at start of cycle;
        # OFF=count means falling edge at `count` of 4096. Full-off uses
        # OFF_H bit 4 (datasheet §7.3.3) — unambiguous "no pulse this cycle"
        # even across frequency changes.
        if duty_frac <= 0.0:
            block = [0x00, 0x00, 0x00, 0x10]
        else:
            off_count = int(round(duty_frac * (self.RESOLUTION - 1)))
            off_count = max(1, min(self.RESOLUTION - 1, off_count))
            block = [0x00, 0x00, off_count & 0xFF, (off_count >> 8) & 0x0F]
        base = self.REG_LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(self._address, base, block)

    def cleanup(self) -> None:
        if self._bus is None:
            return
        for ch in (self._rpwm_left, self._lpwm_left,
                   self._rpwm_right, self._lpwm_right):
            if ch is not None:
                try:
                    self._set_channel_duty(ch, 0.0)
                except OSError:
                    pass
        self._bus.close()
        self._bus = None


class MockGpioBackend(GpioBackend):
    """Captures every track command and publishes it on /test/motor_pwm."""

    TOPIC = '/test/motor_pwm'

    def __init__(self, node) -> None:
        from geometry_msgs.msg import Vector3Stamped
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        self._node = node
        self._Vector3Stamped = Vector3Stamped
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self._pub = node.create_publisher(Vector3Stamped, self.TOPIC, qos)

    def setup(self, *_args, **_kwargs) -> None:
        return

    def write(self, left_signed: float, right_signed: float) -> None:
        msg = self._Vector3Stamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.vector.x = float(left_signed)
        msg.vector.y = float(right_signed)
        msg.vector.z = 0.0
        self._pub.publish(msg)

    def cleanup(self) -> None:
        return


def make_backend(name: str, node) -> GpioBackend:
    if name == 'mock':
        return MockGpioBackend(node)
    if name == 'lgpio':
        return LgpioBackend()
    if name == 'pca9685':
        return Pca9685Backend(node)
    raise ValueError(
        f"unknown gpio_backend: {name!r} "
        "(expected 'lgpio', 'pca9685', or 'mock')"
    )
