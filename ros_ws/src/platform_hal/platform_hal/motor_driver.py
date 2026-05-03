"""SRS-HAL-001 motor_driver (rev 0.4 — dual-PWM).

Subscribes /hal/cmd_vel_safe (Twist), converts to skid-steer per-track
velocities, and drives two IBT-2 (BTS7960B) H-bridges via a pluggable GPIO
backend. Signalling is dual-PWM per SRS-HAL-001 rev 0.4 / HW-PI5-001:
each track exposes an RPWM (forward) and LPWM (reverse) channel,
mutually exclusive at every instant. Implements the M1 safety rules:

  - SR-008 / SRS-HAL-001-S02: hold zero until first valid command.
  - SR-005 / SRS-HAL-001-F04: halt within cmd_vel_timeout_ms (default 500 ms)
    of last received command. Independent of the upstream safety_monitor.
  - REQ-ICD-002-04 / SRS-HAL-001-F05: reject NaN/Inf in any Twist field.
  - SRS-HAL-001-F02: clip linear and angular velocity to configured maxima.
  - SRS-HAL-001-F03: mutual-exclusion of RPWM/LPWM per track (backend).
"""

from __future__ import annotations

import math
import sys

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from platform_hal.gpio_backend import make_backend


def _twist_is_finite(msg: Twist) -> bool:
    fields = (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z,
    )
    return all(math.isfinite(v) for v in fields)


def _clip(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def _skid_steer(v: float, w: float, track_width: float) -> tuple[float, float]:
    """Twist (linear v, angular w) → (left, right) per-track velocity in m/s."""
    half_track = track_width / 2.0
    return v - w * half_track, v + w * half_track


class MotorDriver(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('motor_driver', **kwargs)

        # Pin/channel defaults match the PCA9685 backend (production path on
        # this stack; RP1 hardware PWM is broken). For gpio_backend='pca9685'
        # the four `*_pin` values are PCA9685 channel indices in [0, 15].
        # For gpio_backend='lgpio' override via YAML to the BCM GPIOs frozen
        # in HW-PI5-001 §3 (12/13/18/19) — but note lgpio.tx_pwm currently
        # no-ops on Ubuntu 24.04 / RP1.
        self.declare_parameter('rpwm_left_pin', 0)
        self.declare_parameter('lpwm_left_pin', 1)
        self.declare_parameter('rpwm_right_pin', 2)
        self.declare_parameter('lpwm_right_pin', 3)
        # PCA9685 PWM range is 24-1526 Hz; IBT-2 tolerates 0.5-25 kHz.
        self.declare_parameter('pwm_frequency_hz', 1000)
        self.declare_parameter('track_width_m', 0.28)
        self.declare_parameter('max_linear_vel', 0.7)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('cmd_vel_timeout_ms', 500)
        self.declare_parameter('control_loop_hz', 50.0)
        self.declare_parameter('gpio_backend', 'pca9685')
        # I²C bus + address for the pca9685 backend (shared with IMU on bus 1;
        # PCA9685 default address 0x40 doesn't collide with ISM330DHCX 0x6A/B).
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        # Pi 5: header GPIOs live on /dev/gpiochip4 (RP1). Pi 4: gpiochip0.
        # Used only by the lgpio backend.
        self.declare_parameter('gpio_chip', 4)

        self._track_width = float(self.get_parameter('track_width_m').value)
        self._max_lin = float(self.get_parameter('max_linear_vel').value)
        self._max_ang = float(self.get_parameter('max_angular_vel').value)
        self._timeout = Duration(
            nanoseconds=int(self.get_parameter('cmd_vel_timeout_ms').value) * 1_000_000
        )
        backend_name = str(self.get_parameter('gpio_backend').value)

        self._backend = make_backend(backend_name, self)
        self._backend.setup(
            rpwm_left_pin=int(self.get_parameter('rpwm_left_pin').value),
            lpwm_left_pin=int(self.get_parameter('lpwm_left_pin').value),
            rpwm_right_pin=int(self.get_parameter('rpwm_right_pin').value),
            lpwm_right_pin=int(self.get_parameter('lpwm_right_pin').value),
            pwm_frequency_hz=int(self.get_parameter('pwm_frequency_hz').value),
            gpio_chip=int(self.get_parameter('gpio_chip').value),
        )

        # Last-known-good target velocity (track-frame, in m/s).
        # None until first valid /hal/cmd_vel_safe arrives — satisfies SR-008.
        self._last_cmd_time = None
        self._target_left = 0.0
        self._target_right = 0.0

        # Diagnostic counters.
        self._n_cmds_received = 0
        self._n_cmds_rejected_nonfinite = 0
        self._n_cmds_clipped = 0
        self._n_safe_halts = 0

        # /hal/cmd_vel_safe QoS per ICD-HAL-002 §6.
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Twist, '/hal/cmd_vel_safe', self._on_cmd_vel, cmd_vel_qos,
        )

        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10,
        )

        control_period = 1.0 / float(self.get_parameter('control_loop_hz').value)
        self.create_timer(control_period, self._control_tick)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"motor_driver started "
            f"(backend={backend_name}, timeout={self._timeout.nanoseconds // 1_000_000}ms, "
            f"track_width={self._track_width}m)"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        if not _twist_is_finite(msg):
            self._n_cmds_rejected_nonfinite += 1
            self.get_logger().warning(
                'rejected Twist with non-finite field; holding zero'
            )
            self._target_left = 0.0
            self._target_right = 0.0
            self._last_cmd_time = None  # reset to startup-equivalent state
            return

        v = msg.linear.x
        w = msg.angular.z
        if abs(v) > self._max_lin or abs(w) > self._max_ang:
            self._n_cmds_clipped += 1
        v = _clip(v, self._max_lin)
        w = _clip(w, self._max_ang)

        self._target_left, self._target_right = _skid_steer(v, w, self._track_width)
        self._last_cmd_time = self.get_clock().now()
        self._n_cmds_received += 1

    def _control_tick(self) -> None:
        now = self.get_clock().now()
        if self._last_cmd_time is None or (now - self._last_cmd_time) > self._timeout:
            if self._last_cmd_time is not None:
                # Edge: was running, just timed out.
                self._last_cmd_time = None
                self._n_safe_halts += 1
                self.get_logger().warning(
                    'cmd_vel_safe stale beyond timeout; halting motors'
                )
            self._backend.write(0.0, 0.0)
            return

        # Map per-track velocity (m/s) to PWM duty in [-1, 1].
        # Calibration is empirical (SRS-HAL-001 OPEN-01); first-pass linear map.
        left_duty = _clip(self._target_left / self._max_lin, 1.0)
        right_duty = _clip(self._target_right / self._max_lin, 1.0)
        self._backend.write(left_duty, right_duty)

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = 'platform_hal: motor_driver'
        status.hardware_id = 'ibt2_x2'
        if self._last_cmd_time is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'no recent cmd_vel_safe (held at zero)'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'driving'
        status.values = [
            KeyValue(key='cmds_received', value=str(self._n_cmds_received)),
            KeyValue(key='cmds_rejected_nonfinite', value=str(self._n_cmds_rejected_nonfinite)),
            KeyValue(key='cmds_clipped', value=str(self._n_cmds_clipped)),
            KeyValue(key='safe_halts', value=str(self._n_safe_halts)),
            KeyValue(key='target_left_mps', value=f'{self._target_left:.3f}'),
            KeyValue(key='target_right_mps', value=f'{self._target_right:.3f}'),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._backend.write(0.0, 0.0)
            self._backend.cleanup()
        except Exception as exc:  # noqa: BLE001 — best-effort during shutdown
            self.get_logger().warning(f'backend cleanup failed: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    sys.exit(0)
