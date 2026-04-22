"""GPIO backend abstraction for motor_driver.

Two implementations: LgpioBackend (Pi 5 hardware via the `lgpio` library)
and MockGpioBackend (publishes per-track signed PWM to /test/motor_pwm
for integration tests). The motor_driver selects via the `gpio_backend`
parameter so unit and launch tests can run without hardware.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class GpioBackend(ABC):
    @abstractmethod
    def setup(self, pwm_left_pin: int, dir_left_pin: int,
              pwm_right_pin: int, dir_right_pin: int,
              pwm_frequency_hz: int) -> None:
        ...

    @abstractmethod
    def write(self, left_signed: float, right_signed: float) -> None:
        """Drive both tracks. Inputs in [-1.0, 1.0]: sign = direction, magnitude = duty."""
        ...

    @abstractmethod
    def cleanup(self) -> None:
        ...


class LgpioBackend(GpioBackend):
    """Drives BTS7960 via the `lgpio` library on Raspberry Pi 5."""

    def __init__(self) -> None:
        import lgpio  # lazy import — package must build without lgpio installed
        self._lgpio = lgpio
        self._handle: int | None = None
        self._pwm_left: int | None = None
        self._pwm_right: int | None = None
        self._dir_left: int | None = None
        self._dir_right: int | None = None
        self._frequency: int = 0

    def setup(self, pwm_left_pin, dir_left_pin, pwm_right_pin, dir_right_pin,
              pwm_frequency_hz) -> None:
        self._handle = self._lgpio.gpiochip_open(0)
        self._pwm_left = pwm_left_pin
        self._pwm_right = pwm_right_pin
        self._dir_left = dir_left_pin
        self._dir_right = dir_right_pin
        self._frequency = pwm_frequency_hz
        for pin in (dir_left_pin, dir_right_pin):
            self._lgpio.gpio_claim_output(self._handle, pin, 0)
        for pin in (pwm_left_pin, pwm_right_pin):
            self._lgpio.tx_pwm(self._handle, pin, pwm_frequency_hz, 0)

    def write(self, left_signed: float, right_signed: float) -> None:
        if self._handle is None:
            return
        self._lgpio.gpio_write(self._handle, self._dir_left, 1 if left_signed >= 0 else 0)
        self._lgpio.gpio_write(self._handle, self._dir_right, 1 if right_signed >= 0 else 0)
        self._lgpio.tx_pwm(self._handle, self._pwm_left, self._frequency,
                           min(100.0, abs(left_signed) * 100.0))
        self._lgpio.tx_pwm(self._handle, self._pwm_right, self._frequency,
                           min(100.0, abs(right_signed) * 100.0))

    def cleanup(self) -> None:
        if self._handle is None:
            return
        for pin in (self._pwm_left, self._pwm_right):
            if pin is not None:
                self._lgpio.tx_pwm(self._handle, pin, self._frequency, 0)
        for pin in (self._dir_left, self._dir_right):
            if pin is not None:
                self._lgpio.gpio_write(self._handle, pin, 0)
        self._lgpio.gpiochip_close(self._handle)
        self._handle = None


class MockGpioBackend(GpioBackend):
    """Captures every track command and publishes it on /test/motor_pwm."""

    TOPIC = '/test/motor_pwm'

    def __init__(self, node) -> None:
        from geometry_msgs.msg import Vector3Stamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
