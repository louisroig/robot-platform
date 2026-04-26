#!/usr/bin/env bash
# Publish a Twist on /hal/cmd_vel_raw at 50 Hz.
# Ctrl-C stops publishing; motor_driver safe-halts within cmd_vel_timeout_ms (500 ms).
#
# Usage: scripts/pub-vel.sh [linear_x] [angular_z]
#   linear_x   m/s   (default: 0.1)
#   angular_z  rad/s (default: 0.0)

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIN="${1:-0.1}"
ANG="${2:-0.0}"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/ros_ws/install/setup.bash"

echo "Publishing /hal/cmd_vel_raw @ 50 Hz: linear.x=$LIN angular.z=$ANG"
echo "Ctrl-C to stop (watchdog halts motors within 500 ms)."
exec ros2 topic pub -r 50 /hal/cmd_vel_raw geometry_msgs/Twist \
    "{linear: {x: $LIN}, angular: {z: $ANG}}"
