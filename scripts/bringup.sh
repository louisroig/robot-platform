#!/usr/bin/env bash
# Bring up the M1 HAL stack: safety_monitor + motor_driver.
# IMU is off by default until imu_calibration.yaml exists.
#
# Usage: scripts/bringup.sh [enable_imu] [gpio_backend]
#   enable_imu     true|false   (default: false)
#   gpio_backend   lgpio|mock   (default: lgpio)

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENABLE_IMU="${1:-false}"
GPIO_BACKEND="${2:-lgpio}"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/ros_ws/install/setup.bash"

exec ros2 launch platform_hal platform_hal.launch.py \
    enable_imu:="$ENABLE_IMU" \
    gpio_backend:="$GPIO_BACKEND"
