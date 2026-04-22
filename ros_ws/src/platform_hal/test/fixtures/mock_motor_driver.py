"""Mock GPIO test fixture for motor_driver.

The actual GPIO mock implementation lives in `platform_hal.gpio_backend`
(MockGpioBackend) so the motor_driver subprocess can select it via the
`gpio_backend=mock` parameter. This module is the test-side counterpart:
a helper node that subscribes to /test/motor_pwm and records every track
command with its timestamp, used by TEST-HAL-009 and other integration
tests to verify motor behavior without hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from platform_hal.gpio_backend import MockGpioBackend


@dataclass(frozen=True)
class MotorSample:
    stamp_ns: int       # ROS time when motor_driver wrote the command
    left: float         # signed PWM in [-1.0, 1.0]
    right: float

    @property
    def is_zero(self) -> bool:
        return self.left == 0.0 and self.right == 0.0


class MotorCommandRecorder(Node):
    """Subscribes to /test/motor_pwm and stores every sample for assertion."""

    def __init__(self) -> None:
        super().__init__('motor_command_recorder')
        self._lock = Lock()
        self._samples: list[MotorSample] = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(
            Vector3Stamped, MockGpioBackend.TOPIC, self._on_msg, qos,
        )

    def _on_msg(self, msg: Vector3Stamped) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._lock:
            self._samples.append(MotorSample(stamp_ns, msg.vector.x, msg.vector.y))

    def snapshot(self) -> list[MotorSample]:
        with self._lock:
            return list(self._samples)

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()
