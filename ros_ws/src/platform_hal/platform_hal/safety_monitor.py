"""SRS-HAL-003 safety_monitor — M1 STUB.

Per CLAUDE.md, M1 behavior is unconditional pass-through:
/hal/cmd_vel_raw → /hal/cmd_vel_safe with no gating. The full
state-machine implementation (e-stop, tilt, person-detect, heartbeat,
decel ramp, latching, /safety/reset service) lands in M2.

This stub exists so the M1 wiring is end-to-end correct: teleop publishes
on /hal/cmd_vel_raw, motor_driver subscribes to /hal/cmd_vel_safe, and
the topic that connects them is owned by safety_monitor — even if it is
currently a no-op.
"""

from __future__ import annotations

import sys

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class SafetyMonitorStub(Node):
    def __init__(self) -> None:
        super().__init__('safety_monitor')

        # ICD-HAL-002 §6 — same QoS on both sides (publisher & subscriber).
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._safe_pub = self.create_publisher(
            Twist, '/hal/cmd_vel_safe', cmd_vel_qos,
        )
        self.create_subscription(
            Twist, '/hal/cmd_vel_raw', self._on_raw, cmd_vel_qos,
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10,
        )

        self._n_forwarded = 0
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            'safety_monitor started (M1 STUB: unconditional pass-through)'
        )

    def _on_raw(self, msg: Twist) -> None:
        self._safe_pub.publish(msg)
        self._n_forwarded += 1

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = 'platform_hal: safety_monitor'
        status.hardware_id = 'm1_stub'
        status.level = DiagnosticStatus.WARN  # always WARN — this is a stub
        status.message = 'M1 stub: pass-through, no safety gating'
        status.values = [
            KeyValue(key='msgs_forwarded', value=str(self._n_forwarded)),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitorStub()
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
