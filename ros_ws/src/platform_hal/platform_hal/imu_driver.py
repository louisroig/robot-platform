"""SRS-HAL-002 imu_driver — M1 (gyro + accel only).

Reads the ISM330DHCX 6-DoF IMU over I²C at 100 Hz, applies calibration
(gyro bias + accel bias/scale from imu_calibration.yaml), runs a 6-DoF
Madgwick filter over gyro + accel, and publishes sensor_msgs/Imu on
/hal/imu/data. Magnetometer is deferred to iteration 2 per HW-PI5-001 §6.

Implements:
  - SRS-HAL-002-F01: 100 Hz publish in REP-103 axis convention.
  - SRS-HAL-002-F02: calibration loaded at startup; refuse to start if missing.
  - SRS-HAL-002-F03: Madgwick filter populates orientation + covariance.
  - SRS-HAL-002-S01: stop publishing on I²C fault (no stale values).
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from platform_hal.imu_backend import (
    AccelTriple,
    GyroTriple,
    ImuBackend,
    make_imu_backend,
)


class ImuCalibrationMissing(RuntimeError):
    """Raised when the calibration file is not present at startup.

    Per SRS-HAL-002-F02 / §9 Failure Modes, the node refuses to start rather
    than emit raw uncalibrated samples.
    """


class MadgwickFilter:
    """6-DoF Madgwick filter (gyro + accel, no mag).

    State is the unit quaternion (w, x, y, z) describing the sensor
    orientation relative to Earth in REP-103 axis convention (z-up, so
    a level at-rest sensor measures +g on the z-axis).

    Reference: Madgwick (2010), "An efficient orientation filter for inertial
    and inertial/magnetic sensor arrays", §3.4 (IMU-only). The gradient of
    the gravity-alignment objective uses the standard IMU-only J^T f form:

        f  = [ 2(q1q3 - q0q2) - ax,
               2(q0q1 + q2q3) - ay,
               2(0.5 - q1² - q2²) - az ]
    """

    def __init__(self, beta: float = 0.1):
        self._beta = float(beta)
        self._q = [1.0, 0.0, 0.0, 0.0]  # identity

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        return (self._q[0], self._q[1], self._q[2], self._q[3])

    def reset(self) -> None:
        self._q = [1.0, 0.0, 0.0, 0.0]

    def update(self, gyro: GyroTriple, accel: AccelTriple, dt: float) -> None:
        gx, gy, gz = gyro
        ax, ay, az = accel
        q0, q1, q2, q3 = self._q

        # Quaternion derivative from angular rate: q̇ω = 0.5 · q ⊗ (0, gx, gy, gz).
        qdot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qdot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qdot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qdot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        anorm = math.sqrt(ax * ax + ay * ay + az * az)
        if anorm > 0.0:
            ax /= anorm
            ay /= anorm
            az /= anorm
            f1 = 2.0 * (q1 * q3 - q0 * q2) - ax
            f2 = 2.0 * (q0 * q1 + q2 * q3) - ay
            f3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az
            s0 = -2.0 * q2 * f1 + 2.0 * q1 * f2
            s1 = 2.0 * q3 * f1 + 2.0 * q0 * f2 - 4.0 * q1 * f3
            s2 = -2.0 * q0 * f1 + 2.0 * q3 * f2 - 4.0 * q2 * f3
            s3 = 2.0 * q1 * f1 + 2.0 * q2 * f2
            snorm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if snorm > 0.0:
                s0 /= snorm
                s1 /= snorm
                s2 /= snorm
                s3 /= snorm
            qdot0 -= self._beta * s0
            qdot1 -= self._beta * s1
            qdot2 -= self._beta * s2
            qdot3 -= self._beta * s3

        q0 += qdot0 * dt
        q1 += qdot1 * dt
        q2 += qdot2 * dt
        q3 += qdot3 * dt
        qnorm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if qnorm > 0.0:
            self._q = [q0 / qnorm, q1 / qnorm, q2 / qnorm, q3 / qnorm]


def _expand_calibration_path(raw: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(raw)))


def _load_calibration(path: Path) -> dict:
    """Load the calibration YAML. Expected schema:

        gyro_bias: [bx, by, bz]           # rad/s to subtract from each gyro sample
        accel_bias: [bx, by, bz]          # m/s² to subtract from each accel sample
        accel_scale: [sx, sy, sz]         # per-axis multiplicative correction

    Any missing key defaults to a neutral no-op value (bias 0, scale 1). The
    file must exist — SRS-HAL-002-F02 forbids publishing uncalibrated samples
    when it doesn't.
    """
    if not path.is_file():
        raise ImuCalibrationMissing(f'imu calibration file not found: {path}')
    data = yaml.safe_load(path.read_text()) or {}
    return {
        'gyro_bias': tuple(data.get('gyro_bias', (0.0, 0.0, 0.0))),
        'accel_bias': tuple(data.get('accel_bias', (0.0, 0.0, 0.0))),
        'accel_scale': tuple(data.get('accel_scale', (1.0, 1.0, 1.0))),
    }


class ImuDriver(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('imu_driver', **kwargs)

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x6B)
        self.declare_parameter('imu_rate_hz', 100.0)
        self.declare_parameter(
            'calibration_file',
            '$HOME/.config/platform/imu_calibration.yaml',
        )
        self.declare_parameter('madgwick_beta', 0.1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_backend', 'ism330')

        calib_path = _expand_calibration_path(
            str(self.get_parameter('calibration_file').value)
        )
        try:
            self._calib = _load_calibration(calib_path)
        except ImuCalibrationMissing as exc:
            self.get_logger().error(
                f'SRS-HAL-002-F02: refusing to start — {exc}. '
                f'Run the IMU calibration tool and write {calib_path}.'
            )
            raise

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._rate_hz = float(self.get_parameter('imu_rate_hz').value)
        self._period = 1.0 / self._rate_hz

        self._filter = MadgwickFilter(
            beta=float(self.get_parameter('madgwick_beta').value)
        )

        backend_name = str(self.get_parameter('imu_backend').value)
        self._backend: ImuBackend = make_imu_backend(backend_name)
        self._backend.setup(
            i2c_bus=int(self.get_parameter('i2c_bus').value),
            i2c_address=int(self.get_parameter('i2c_address').value),
        )

        # Once an I²C fault is sticky for more than this many consecutive
        # control ticks we also reset the Madgwick state, so that a recovered
        # bus doesn't splice orientation across a gap.
        self._consecutive_faults = 0
        self._reset_filter_after_n_faults = 5

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Imu, '/hal/imu/data', qos)

        self._last_tick: Optional[rclpy.clock.Time] = None
        self.create_timer(self._period, self._sample_and_publish)

        self.get_logger().info(
            f'imu_driver started (backend={backend_name}, '
            f'rate={self._rate_hz:.0f} Hz, calibration={calib_path})'
        )

    def _sample_and_publish(self) -> None:
        # SRS-HAL-002-S01: on I²C fault, suspend publication rather than emit
        # stale or uncalibrated values.
        try:
            raw_accel, raw_gyro = self._backend.read()
        except OSError as exc:
            self._consecutive_faults += 1
            if self._consecutive_faults == 1:
                self.get_logger().warning(f'I²C read fault: {exc} — suspending publication')
            if self._consecutive_faults == self._reset_filter_after_n_faults:
                self._filter.reset()
                self._last_tick = None
                self.get_logger().warning('I²C fault persistent — Madgwick state reset')
            return
        if self._consecutive_faults > 0:
            self.get_logger().info('I²C read recovered — resuming publication')
            self._consecutive_faults = 0

        accel = self._apply_accel_calibration(raw_accel)
        gyro = self._apply_gyro_calibration(raw_gyro)

        now = self.get_clock().now()
        if self._last_tick is None:
            dt = self._period
        else:
            dt_ns = (now - self._last_tick).nanoseconds
            dt = dt_ns * 1e-9
            if dt <= 0.0:
                dt = self._period
        self._last_tick = now

        self._filter.update(gyro, accel, dt)
        qw, qx, qy, qz = self._filter.quaternion

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        # Covariances are nominal placeholders until characterization is done
        # in iteration 2. Nonzero diagonals let consumers treat them as
        # "known-unknown" rather than "unknown-unknown" (-1 = not-provided).
        msg.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.02]
        msg.angular_velocity_covariance = [1e-4, 0.0, 0.0,
                                           0.0, 1e-4, 0.0,
                                           0.0, 0.0, 1e-4]
        msg.linear_acceleration_covariance = [1e-2, 0.0, 0.0,
                                              0.0, 1e-2, 0.0,
                                              0.0, 0.0, 1e-2]
        self._pub.publish(msg)

    def _apply_gyro_calibration(self, gyro: GyroTriple) -> GyroTriple:
        bx, by, bz = self._calib['gyro_bias']
        return (gyro[0] - bx, gyro[1] - by, gyro[2] - bz)

    def _apply_accel_calibration(self, accel: AccelTriple) -> AccelTriple:
        bx, by, bz = self._calib['accel_bias']
        sx, sy, sz = self._calib['accel_scale']
        return ((accel[0] - bx) * sx, (accel[1] - by) * sy, (accel[2] - bz) * sz)

    def destroy_node(self) -> bool:
        try:
            self._backend.cleanup()
        except Exception as exc:  # noqa: BLE001 — best-effort during shutdown
            self.get_logger().warning(f'imu backend cleanup failed: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ImuDriver()
    except ImuCalibrationMissing:
        # SRS-HAL-002-F02: error was already logged in __init__. Exit so
        # systemd can respawn the unit once the calibration file appears.
        rclpy.shutdown()
        sys.exit(1)
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
