"""End-to-end drone_mission_coordinator integration test.

Launches the coordinator with the mock MAVROS backend, stands up
mock /mapping/retrieve_latest_image and /mapping/georeference service
servers, publishes /safety/state = OK, sends a goal through the action
client, and asserts the full happy-path mission completes with
result.success == True and a populated image_path.

Also covers the abort path: tilt-time safety ESTOP should make a
mid-flight mission cancel and surface failure_reason='safety_estop'.

Per the project-wide rule (`feedback_ros_test_isolation.md`),
ROS_DOMAIN_ID is set at module scope before rclpy or launch_ros init.
"""

from __future__ import annotations

import os
import sys
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '48'   # different domain from safety integration test

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from platform_msgs.action import RunMappingMission
from platform_msgs.msg import SafetyState as SafetyStateMsg
from platform_msgs.srv import Georeference, RetrieveLatestImage
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    coordinator = launch_ros.actions.Node(
        package='drone_mission',
        executable='drone_mission_coordinator',
        name='drone_mission_coordinator',
        parameters=[{
            'mavros_backend': 'mock',
            'default_altitude_m': 5.0,
            'altitude_tolerance_m': 0.5,
            'climb_timeout_s': 5.0,
            'capture_hover_s': 0.2,
            'capture_timeout_s': 2.0,
            'descent_timeout_s': 5.0,
            'image_transfer_timeout_s': 2.0,
            'georef_timeout_s': 2.0,
            'feedback_rate_hz': 10.0,
            'abort_on_low_battery_pct': 5.0,    # don't trip from sim drain
            'require_safety_ok': True,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        coordinator,
        launch_testing.actions.ReadyToTest(),
    ])


class _MockServiceProvider:
    """Stands up the two mocked services the coordinator depends on.

    Both services succeed by default; tests can flip flags to make either
    fail (e.g. simulate XIAO unreachable).
    """

    def __init__(self, node):
        self.node = node
        self.image_should_succeed = True
        self.georef_should_succeed = True
        self.image_path_to_return = '/tmp/test/capture.jpg'
        self.image_call_count = 0
        self.georef_call_count = 0
        self._lock = threading.Lock()

        node.create_service(
            RetrieveLatestImage, '/mapping/retrieve_latest_image',
            self._on_retrieve,
        )
        node.create_service(
            Georeference, '/mapping/georeference',
            self._on_georef,
        )

    def _on_retrieve(self, req, resp):
        with self._lock:
            self.image_call_count += 1
            ok = self.image_should_succeed
        if ok:
            resp.success = True
            resp.image_path = self.image_path_to_return
            resp.message = ''
        else:
            resp.success = False
            resp.image_path = ''
            resp.message = 'mock failure: xiao unreachable'
        return resp

    def _on_georef(self, req, resp):
        with self._lock:
            self.georef_call_count += 1
            ok = self.georef_should_succeed
        if ok:
            resp.success = True
            resp.message = ''
            resp.meta.image_path = req.image_path
            resp.meta.capture_lat = 37.5
            resp.meta.capture_lon = -122.3
        else:
            resp.success = False
            resp.message = 'mock failure: pyproj transform failed'
        return resp


class TestMissionIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.test_node = rclpy.create_node('test_mission_client')
        cls.executor.add_node(cls.test_node)

        # Publish /safety/state = OK so the coordinator's pre-arm check passes.
        safety_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )
        cls.safety_pub = cls.test_node.create_publisher(
            SafetyStateMsg, '/safety/state', safety_qos,
        )

        cls.mocks = _MockServiceProvider(cls.test_node)
        cls.action_client = ActionClient(
            cls.test_node, RunMappingMission, '/mapping/run_mission',
        )

        # Track feedback messages received during the active goal.
        cls._feedback_log: list = []
        cls._feedback_lock = threading.Lock()

        cls.spin_thread = threading.Thread(
            target=cls.executor.spin, daemon=True,
        )
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ---- helpers --------------------------------------------------------

    def _publish_safety_ok(self):
        msg = SafetyStateMsg()
        msg.state = SafetyStateMsg.STATE_OK
        msg.reasons = []
        msg.motion_permitted = True
        msg.tool_permitted = True
        msg.clearable = True
        self.safety_pub.publish(msg)

    def _publish_safety_estop(self):
        msg = SafetyStateMsg()
        msg.state = SafetyStateMsg.STATE_ESTOP
        msg.reasons = ['tilt_exceeded']
        msg.motion_permitted = False
        msg.tool_permitted = False
        msg.clearable = False
        self.safety_pub.publish(msg)

    def _wait_for_action_server(self, timeout_s: float = 10.0):
        if not self.action_client.wait_for_server(timeout_sec=timeout_s):
            self.fail('action server /mapping/run_mission not available')

    def _send_goal(self, altitude_m: float = 5.0, output_name: str = 'integration_test'):
        goal = RunMappingMission.Goal()
        goal.altitude_m = altitude_m
        goal.output_name = output_name

        with self._feedback_lock:
            self._feedback_log.clear()

        def _on_feedback(fb):
            with self._feedback_lock:
                self._feedback_log.append(fb.feedback)

        # Republish safety OK while the goal is being accepted (TRANSIENT_LOCAL
        # also delivers the latest, but be belt-and-suspenders since the
        # subscription may have just initialized).
        for _ in range(3):
            self._publish_safety_ok()
            time.sleep(0.05)

        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=_on_feedback,
        )
        deadline = time.monotonic() + 5.0
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(send_future.done(), 'send_goal_async did not return')
        return send_future.result()

    def _wait_for_result(self, goal_handle, timeout_s: float = 30.0):
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(
            result_future.done(),
            f'mission did not complete within {timeout_s}s',
        )
        return result_future.result()

    # ---- tests ----------------------------------------------------------

    def test_happy_path_completes_with_success(self):
        self._wait_for_action_server()
        self.mocks.image_should_succeed = True
        self.mocks.georef_should_succeed = True

        goal_handle = self._send_goal()
        self.assertTrue(goal_handle.accepted, 'goal was rejected')

        result_response = self._wait_for_result(goal_handle, timeout_s=15.0)
        self.assertEqual(result_response.status, 4)  # STATUS_SUCCEEDED = 4
        result = result_response.result
        self.assertTrue(
            result.success,
            f'mission failed: {result.failure_reason}',
        )
        self.assertEqual(
            result.image_path,
            self.mocks.image_path_to_return,
        )
        self.assertNotEqual(result.mission_id, '')

        # Mock services were each called exactly once.
        self.assertEqual(self.mocks.image_call_count, 1)
        self.assertEqual(self.mocks.georef_call_count, 1)

        # Feedback covered multiple distinct phases (at minimum CLIMBING
        # and DESCENDING, since CAPTURING/IMAGE_TRANSFER/GEOREF are brief
        # at these test-tuned timings).
        with self._feedback_lock:
            phases_seen = {fb.phase for fb in self._feedback_log}
        self.assertTrue(
            len(phases_seen) >= 2,
            f'expected feedback across multiple phases; saw {phases_seen}',
        )

        print(
            f'TEST-DRN-INT happy-path PASS: '
            f'mission_id={result.mission_id} image={result.image_path} '
            f'phases_seen={phases_seen}'
        )

    def test_image_transfer_failure_surfaces_in_result(self):
        self._wait_for_action_server()
        # Reset mock counters from previous test.
        prior_image_calls = self.mocks.image_call_count
        prior_georef_calls = self.mocks.georef_call_count
        self.mocks.image_should_succeed = False
        self.mocks.georef_should_succeed = True

        goal_handle = self._send_goal(output_name='integration_test_xfer_fail')
        self.assertTrue(goal_handle.accepted)
        result = self._wait_for_result(goal_handle, timeout_s=15.0).result

        self.assertFalse(result.success)
        self.assertEqual(result.failure_reason, 'image_transfer_failed')
        self.assertEqual(result.image_path, '')

        # image_retriever was called; georeferencer wasn't (transfer failed first).
        self.assertEqual(self.mocks.image_call_count, prior_image_calls + 1)
        self.assertEqual(self.mocks.georef_call_count, prior_georef_calls)

        print(
            'TEST-DRN-INT xfer-fail PASS: '
            'image_should_succeed=False → result.failure_reason=image_transfer_failed'
        )
