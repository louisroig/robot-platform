"""GPIO backend abstraction for motor_driver.

Two implementations: LgpioBackend (Pi 5 hardware via the `lgpio` library)
and MockGpioBackend (publishes per-track signed PWM to /test/motor_pwm
for integration tests). The motor_driver selects via the `gpio_backend`
parameter so unit and launch tests can run without hardware.

Signalling convention is IBT-2 dual-PWM per SRS-HAL-001 rev 0.4 / HW-PI5-001:
each track has an RPWM (forward) and LPWM (reverse) channel, mutually
exclusive at every instant. The `write(left_signed, right_signed)`
interface takes duty in [-1.0, 1.0] — sign selects the channel, magnitude
sets the duty — and the LgpioBackend enforces the mutual-exclusion
invariant by zeroing the opposite channel before activating the new one.
"""

from __future__ import annotations

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
    raise ValueError(f"unknown gpio_backend: {name!r} (expected 'lgpio' or 'mock')")
