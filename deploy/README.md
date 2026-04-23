# Deploy

Artifacts for the Pi 5 rover.

## systemd

`rover-platform-hal.service` starts the HAL stack on boot.

Install:
```bash
sudo cp deploy/systemd/rover-platform-hal.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rover-platform-hal
journalctl -u rover-platform-hal -f
```

## udev

Currently a placeholder. Populate with real rules once hardware is finalized.

Install (once populated):
```bash
sudo cp deploy/udev/99-robot-platform.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Teleop

The `teleop_web` package provides a browser joystick that publishes
`/hal/cmd_vel_raw` via rosbridge. One-time install of the rosbridge apt
packages on the Pi:

```bash
sudo apt install ros-jazzy-rosbridge-server
```

Run (on the Pi):
```bash
source /opt/ros/jazzy/setup.bash
source ~/robot-platform/ros_ws/install/setup.bash
ros2 launch teleop_web teleop_web.launch.py
```

This starts a rosbridge WebSocket on port 9090 and a static HTTP server on
port 8000 serving `web/index.html`. On the operator phone (same Wi-Fi as
the rover), open `http://<pi-ip>:8000/` — e.g. `http://192.168.1.128:8000/`.
The page connects automatically and publishes Twist at 50 Hz while the
joystick is held. Release the knob or hit the red EMERGENCY STOP button to
zero velocity; the motor_driver's 500 ms command-timeout halts the rover
if the page disconnects.

## IMU calibration bootstrap

Per SRS-HAL-002 §9 the `imu_driver` refuses to start if
`~/.config/platform/imu_calibration.yaml` is missing. Until the real calibration
tool is written (deferred OPEN-issue), seed it with an identity file so the HAL
stack comes up green on cold boot:

```bash
mkdir -p ~/.config/platform
cp ros_ws/src/platform_hal/test/fixtures/imu_calibration_stub.yaml \
   ~/.config/platform/imu_calibration.yaml
```

Replace with real per-axis biases (gyro especially) before relying on
orientation for anything consequential.
