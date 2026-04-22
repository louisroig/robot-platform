"""TEST-HAL-009 — motor_driver command timeout.

Verifies SRS-HAL-001-F04 / REQ-ICD-002-03 / SR-005:
the motor driver halts both tracks within 500 ms of the last
/hal/cmd_vel_safe message.

Method:
  - Launch motor_driver with gpio_backend=mock so every track command
    is mirrored on /test/motor_pwm.
  - For each of N_TRIALS trials, publish Twist(linear.x=0.3) at 50 Hz
    for a short window, stop, then assert from the captured samples
    that the first zero command lands within 600 ms of t_stop and no
    nonzero command appears beyond t_zero + 100 ms.
"""

from __future__ import annotations

import os
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# Make the test/ tree importable as a package so we can pull the recorder.
import sys
sys.path.insert(0, os.path.dirname(__file__))
from fixtures.mock_motor_driver import MotorCommandRecorder  # noqa: E402


N_TRIALS = 10
PUBLISH_HZ = 50.0
PUBLISH_DURATION_S = 1.0
POST_STOP_WAIT_S = 1.0
TIMEOUT_BUDGET_MS = 600   # spec: ≤500 ms + 100 ms margin
LATE_NONZERO_WINDOW_MS = 100


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    motor_driver = launch_ros.actions.Node(
        package='platform_hal',
        executable='motor_driver',
        name='motor_driver',
        parameters=[{
            'gpio_backend': 'mock',
            'cmd_vel_timeout_ms': 500,
            'control_loop_hz': 50.0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        motor_driver,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCmdVelTimeout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.recorder = MotorCommandRecorder()
        cls.executor.add_node(cls.recorder)
        cls.spin_thread = threading.Thread(
            target=cls.executor.spin, daemon=True,
        )
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _wait_for_motor_driver(self, timeout_s: float = 10.0) -> None:
        """Block until motor_driver has subscribed to /hal/cmd_vel_safe."""
        deadline = time.monotonic() + timeout_s
        node = self.recorder
        while time.monotonic() < deadline:
            count = node.count_subscribers('/hal/cmd_vel_safe')
            if count >= 1:
                return
            time.sleep(0.05)
        self.fail('motor_driver did not subscribe to /hal/cmd_vel_safe in time')

    def _wait_for_first_sample(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.recorder.snapshot():
                return
            time.sleep(0.02)
        self.fail('no /test/motor_pwm samples received')

    def test_cmd_vel_timeout(self):
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        publisher_node = rclpy.create_node('test_cmd_vel_publisher')
        try:
            self.executor.add_node(publisher_node)
            pub = publisher_node.create_publisher(
                Twist, '/hal/cmd_vel_safe', cmd_qos,
            )
            self._wait_for_motor_driver()
            self._wait_for_first_sample()

            latencies_ms: list[float] = []
            failures: list[str] = []

            for trial in range(N_TRIALS):
                self.recorder.clear()
                cmd = Twist()
                cmd.linear.x = 0.3
                period = 1.0 / PUBLISH_HZ
                pub_deadline = time.monotonic() + PUBLISH_DURATION_S
                while time.monotonic() < pub_deadline:
                    pub.publish(cmd)
                    time.sleep(period)
                # Final publish to mark the canonical t_stop in ROS time:
                pub.publish(cmd)
                t_stop_ns = publisher_node.get_clock().now().nanoseconds

                # Wait long enough for the timeout to fire and zeros to land.
                time.sleep(POST_STOP_WAIT_S)

                samples = self.recorder.snapshot()
                pre_stop_nonzero = [
                    s for s in samples
                    if s.stamp_ns < t_stop_ns and not s.is_zero
                ]
                post_stop = [s for s in samples if s.stamp_ns >= t_stop_ns]

                if not pre_stop_nonzero:
                    failures.append(
                        f'trial {trial}: no nonzero motor commands seen pre-stop '
                        f'({len(samples)} total samples)')
                    continue
                if not post_stop:
                    failures.append(
                        f'trial {trial}: no samples after t_stop')
                    continue

                first_zero = next(
                    (s for s in post_stop if s.is_zero), None,
                )
                if first_zero is None:
                    failures.append(
                        f'trial {trial}: motor never reached zero after t_stop')
                    continue

                latency_ms = (first_zero.stamp_ns - t_stop_ns) / 1_000_000.0
                latencies_ms.append(latency_ms)
                if latency_ms > TIMEOUT_BUDGET_MS:
                    failures.append(
                        f'trial {trial}: latency {latency_ms:.1f} ms > '
                        f'{TIMEOUT_BUDGET_MS} ms budget')

                # No nonzero command may appear after t_zero + LATE_NONZERO_WINDOW_MS.
                late_threshold_ns = first_zero.stamp_ns + LATE_NONZERO_WINDOW_MS * 1_000_000
                late_nonzero = [
                    s for s in post_stop
                    if s.stamp_ns > late_threshold_ns and not s.is_zero
                ]
                if late_nonzero:
                    failures.append(
                        f'trial {trial}: {len(late_nonzero)} late nonzero '
                        f'commands after t_zero + {LATE_NONZERO_WINDOW_MS} ms')

            if failures:
                joined = '\n  - '.join(failures)
                self.fail(
                    f'{len(failures)} of {N_TRIALS} trials failed:\n  - {joined}\n'
                    f'latencies (ms): {latencies_ms}'
                )

            print(
                f'TEST-HAL-009 PASS: {N_TRIALS} trials, '
                f'latencies (ms): min={min(latencies_ms):.1f} '
                f'max={max(latencies_ms):.1f} '
                f'mean={sum(latencies_ms) / len(latencies_ms):.1f}'
            )
        finally:
            self.executor.remove_node(publisher_node)
            publisher_node.destroy_node()
