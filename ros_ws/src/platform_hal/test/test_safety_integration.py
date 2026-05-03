"""End-to-end safety_monitor + motor_driver integration test.

Launches the full M2 safety chain — safety_monitor and motor_driver with
the mock GPIO backend — and proves the spec-level guarantee:

  publishing /hal/cmd_vel_raw at 50 Hz drives the motor to nonzero PWM,
  then injecting an IMU tilt excursion makes the motor go to zero on
  /test/motor_pwm even while raw publishing continues.

Also exercises:
  - level IMU + valid raw → motor sees nonzero (gate is open)
  - tilt > limit → motor sees zero within ~one safety eval period
  - /safety/reset refused while still tilted, granted after recovery
  - resumed level state restores motor commands

This is a launch_testing test (separate process per node) so it covers
QoS, timing, and the MultiThreadedExecutor path that unit tests bypass.
Publishers run on rclpy timers (not python sleep loops) so the cadence
is reliable enough that scheduler variance doesn't trip safety_monitor's
staleness watchdogs and hide the trigger we're actually testing.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
import unittest

# Isolate this test from any platform_hal stack already running on the host
# (e.g. the user's bench launch). Without isolation, our test's safety_monitor
# would receive IMU messages from BOTH our test publisher and the real
# imu_driver, oscillating between tilted and level — which masks the
# behavior under test. ROS_DOMAIN_ID must be set BEFORE rclpy initializes
# anywhere in the process, including the launch_ros subprocesses below.
os.environ['ROS_DOMAIN_ID'] = '47'

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
from sensor_msgs.msg import Imu
from platform_msgs.msg import SafetyState as SafetyStateMsg
from platform_msgs.srv import ResetSafety

sys.path.insert(0, os.path.dirname(__file__))
from fixtures.mock_motor_driver import MotorCommandRecorder  # noqa: E402


PUBLISH_HZ = 50.0
IMU_HZ = 100.0
SETTLE_S = 0.6
WARMUP_S = 1.0
# Generous staleness windows: this test is about tilt + reset behavior,
# not about staleness detection. Real-time scheduling jitter on a busy CI
# host would otherwise make timer cadence trip the staleness watchdogs and
# mask the trigger we're trying to verify. Staleness has its own coverage
# in the unit-test suite.
TEST_IMU_STALE_MS = 2000
TEST_CMD_VEL_STALE_MS = 2000


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    safety_monitor = launch_ros.actions.Node(
        package='platform_hal',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'tilt_limit_deg': 25.0,
            'tilt_warning_deg': 18.0,
            'tilt_latches': True,
            'imu_staleness_ms': TEST_IMU_STALE_MS,
            'cmd_vel_staleness_ms': TEST_CMD_VEL_STALE_MS,
            'eval_rate_hz': 20.0,
            'safe_publish_rate_hz': 50.0,
        }],
        output='screen',
    )
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
        safety_monitor,
        motor_driver,
        launch_testing.actions.ReadyToTest(),
    ])


def _imu_msg(tilt_deg: float = 0.0) -> Imu:
    """Build an IMU message representing a pure pitch by `tilt_deg`."""
    msg = Imu()
    half = math.radians(tilt_deg) / 2.0
    msg.orientation.w = math.cos(half)
    msg.orientation.x = 0.0
    msg.orientation.y = math.sin(half)
    msg.orientation.z = 0.0
    return msg


def _twist(linear_x: float = 0.0) -> Twist:
    t = Twist()
    t.linear.x = linear_x
    return t


class TestSafetyIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.recorder = MotorCommandRecorder()
        cls.executor.add_node(cls.recorder)

        cls.pub_node = rclpy.create_node('test_safety_publisher')
        cls.executor.add_node(cls.pub_node)

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cls.raw_pub = cls.pub_node.create_publisher(
            Twist, '/hal/cmd_vel_raw', cmd_vel_qos,
        )
        cls.imu_pub = cls.pub_node.create_publisher(
            Imu, '/hal/imu/data', imu_qos,
        )
        cls.reset_client = cls.pub_node.create_client(ResetSafety, '/safety/reset')

        # Subscribe to /safety/state so the test can also verify the new
        # topic publishes per ICD-SAF-001 (TRANSIENT_LOCAL means we get the
        # latest value immediately on subscription).
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )
        cls._latest_state_msg = None
        def _on_state(m):
            cls._latest_state_msg = m
        cls.pub_node.create_subscription(
            SafetyStateMsg, '/safety/state', _on_state, state_qos,
        )

        # Shared mutable state: a dict so test methods can mutate values via
        # `self._intent[...] = x` and the timer callbacks (closing over `cls`)
        # see the same object. Instance attribute assignment would shadow the
        # class-level state and timers would keep reading the original value.
        cls._intent_lock = threading.Lock()
        cls._intent = {
            'raw': _twist(linear_x=0.3),
            'tilt_deg': 0.0,
        }

        # rclpy timers are driven by the executor — far more reliable cadence
        # than a python thread + time.sleep, especially on shared CI hosts.
        def _publish_raw():
            with cls._intent_lock:
                cls.raw_pub.publish(cls._intent['raw'])

        def _publish_imu():
            with cls._intent_lock:
                cls.imu_pub.publish(_imu_msg(cls._intent['tilt_deg']))

        cls.pub_node.create_timer(1.0 / PUBLISH_HZ, _publish_raw)
        cls.pub_node.create_timer(1.0 / IMU_HZ, _publish_imu)

        cls.spin_thread = threading.Thread(
            target=cls.executor.spin, daemon=True,
        )
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.pub_node.destroy_node()
        cls.recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ---- helpers --------------------------------------------------------

    def _set_intent(self, *, raw_linear_x: float = None, tilt_deg: float = None):
        with self._intent_lock:
            if raw_linear_x is not None:
                self._intent['raw'] = _twist(linear_x=raw_linear_x)
            if tilt_deg is not None:
                self._intent['tilt_deg'] = tilt_deg

    def _wait_for_subscribers(self, timeout_s: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_s
        node = self.pub_node
        while time.monotonic() < deadline:
            ok = (
                node.count_subscribers('/hal/cmd_vel_raw') >= 1
                and node.count_subscribers('/hal/imu/data') >= 1
                and node.count_subscribers('/hal/cmd_vel_safe') >= 1
            )
            if ok and self.reset_client.service_is_ready():
                return
            time.sleep(0.05)
        self.fail('subscribers / reset service not ready in time')

    def _call_reset(self, timeout_s: float = 2.0) -> ResetSafety.Response:
        future = self.reset_client.call_async(ResetSafety.Request())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        self.fail('/safety/reset call did not return in time')

    def _last_motor_sample_after(self, t_start_ns: int):
        samples = self.recorder.snapshot()
        relevant = [s for s in samples if s.stamp_ns >= t_start_ns]
        return relevant[-1] if relevant else None

    def _assert_motor_zero_within(self, t_trigger_ns: int, budget_s: float):
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            sample = self._last_motor_sample_after(t_trigger_ns)
            if sample is not None and sample.is_zero:
                return sample
            time.sleep(0.02)
        last = self._last_motor_sample_after(t_trigger_ns)
        self.fail(
            f'motor did not reach zero within {budget_s:.2f}s of trigger; '
            f'last sample: {last}'
        )

    def _assert_motor_nonzero_within(self, t_trigger_ns: int, budget_s: float):
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            sample = self._last_motor_sample_after(t_trigger_ns)
            if sample is not None and not sample.is_zero:
                return sample
            time.sleep(0.02)
        last = self._last_motor_sample_after(t_trigger_ns)
        self.fail(
            f'motor did not reach nonzero within {budget_s:.2f}s; '
            f'last sample: {last}'
        )

    # ---- the test -------------------------------------------------------

    def test_full_safety_chain(self):
        self._wait_for_subscribers()

        # Warm up: level IMU + nonzero raw → motor should be driving.
        self._set_intent(raw_linear_x=0.3, tilt_deg=0.0)
        time.sleep(WARMUP_S)
        t0 = self.pub_node.get_clock().now().nanoseconds
        sample = self._assert_motor_nonzero_within(t0, budget_s=1.0)
        self.assertGreater(
            abs(sample.left), 0.0,
            'expected nonzero PWM with safe state and nonzero raw command',
        )

        # ---- 1. tilt excursion → motor zero -------------------------------
        self._set_intent(tilt_deg=30.0)
        t_tilt = self.pub_node.get_clock().now().nanoseconds
        zero_sample = self._assert_motor_zero_within(t_tilt, budget_s=SETTLE_S)
        self.assertGreaterEqual(zero_sample.stamp_ns, t_tilt)

        # ---- 2. /safety/reset refused while still tilted ------------------
        response = self._call_reset()
        self.assertFalse(
            response.success,
            f'/safety/reset should refuse while still tilted; got {response.reason!r}',
        )
        self.assertIn('tilt_exceeded', response.reason)

        # ---- 3. recover tilt; latch keeps motor at zero --------------------
        self._set_intent(tilt_deg=0.0)
        time.sleep(SETTLE_S)
        t_post_recover = self.pub_node.get_clock().now().nanoseconds
        time.sleep(0.2)
        post_sample = self._last_motor_sample_after(t_post_recover)
        self.assertIsNotNone(post_sample, 'no motor samples after recovery')
        self.assertTrue(
            post_sample.is_zero,
            f'tilt latch should hold motor at zero post-recovery; got {post_sample}',
        )

        # ---- 4. /safety/reset granted; motor resumes ----------------------
        response = self._call_reset()
        self.assertTrue(
            response.success,
            f'/safety/reset should succeed once recovered; got {response.reason!r}',
        )

        t_after_reset = self.pub_node.get_clock().now().nanoseconds
        nonzero_sample = self._assert_motor_nonzero_within(
            t_after_reset, budget_s=SETTLE_S,
        )
        self.assertGreater(abs(nonzero_sample.left), 0.0)

        # ---- 5. /safety/state was published and reflects current state ----
        # By this point the post-reset OK state must have been broadcast on
        # /safety/state. ICD-SAF-001 §6 says TRANSIENT_LOCAL so the latest
        # value sticks; any subscriber that joined late still sees it.
        self.assertIsNotNone(
            self._latest_state_msg,
            '/safety/state never published any message',
        )
        self.assertEqual(self._latest_state_msg.state, SafetyStateMsg.STATE_OK)
        self.assertTrue(self._latest_state_msg.motion_permitted)
        self.assertTrue(self._latest_state_msg.tool_permitted)

        print(
            'TEST-SAF-INT PASS: '
            'gate-open → tilt-excursion → motor-zero → reset-refused → '
            'recover → latched-zero → reset-granted → motor-resumed → '
            'safety_state=OK'
        )
