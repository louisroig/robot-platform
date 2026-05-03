# Deploy — Pi 5 rover

Artifacts and runbook for bringing a fresh Pi 5 up to the M1 exit criterion:
*rover drives under teleop, and a cold power cycle brings everything back
green without manual intervention.*

## Contents

- `systemd/rover-platform-hal.service` — HAL stack (motor_driver + imu_driver + safety_monitor)
- `systemd/rover-teleop-web.service`   — rosbridge WebSocket + joystick HTTP server
- `config/platform_hal.yaml`           — **production** params (PCA9685 motor backend)
- `config/platform_hal-bench.yaml`     — **bench** params (mock motor backend, real IMU)
- `scripts/switch-platform-config.sh`  — swap which YAML the systemd unit uses
- `udev/99-robot-platform.rules`       — placeholder (finalize after HW complete)

## One-time install (first deployment)

Assumes the repo is cloned at `/home/rover/robot-platform` and the workspace
has been built at least once (`colcon build --symlink-install` inside `ros_ws/`).

### 1. Install runtime dependencies

```bash
sudo apt update
sudo apt install \
    ros-jazzy-rosbridge-server \
    python3-smbus2 \
    python3-yaml
```

`smbus2` is needed by `imu_driver` for I²C; `rosbridge_server` is needed by
`teleop_web` for the browser WebSocket.

### 2. Bootstrap the IMU calibration file

Per SRS-HAL-002 §9 the `imu_driver` refuses to start without
`~/.config/platform/imu_calibration.yaml`. Seed it with the identity stub so
first boot comes up green; replace with real per-axis biases once the
calibration tool exists.

```bash
mkdir -p ~/.config/platform
cp ~/robot-platform/ros_ws/src/platform_hal/test/fixtures/imu_calibration_stub.yaml \
   ~/.config/platform/imu_calibration.yaml
```

### 3. Install and enable the systemd units

```bash
sudo cp ~/robot-platform/deploy/systemd/rover-platform-hal.service /etc/systemd/system/
sudo cp ~/robot-platform/deploy/systemd/rover-teleop-web.service   /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rover-platform-hal.service
sudo systemctl enable --now rover-teleop-web.service
```

Both units run as `User=rover` with `WorkingDirectory=/home/rover/robot-platform/ros_ws`
and `Restart=on-failure`. The teleop unit declares `After=rover-platform-hal.service`
so the HAL stack reaches `ros2 launch` first (DDS discovery is order-independent,
but startup logs read more sensibly this way).

## Validation (post-install, before and after cold boot)

### Before power-cycle — verify both units are active

```bash
systemctl status rover-platform-hal rover-teleop-web --no-pager
```

Expected: both `active (running)`, no recent `Failed` entries.

### Verify HAL topics are live

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot-platform/ros_ws/install/setup.bash
ros2 node list
# expected: /motor_driver /imu_driver /safety_monitor /rosbridge_websocket
ros2 topic hz /hal/imu/data
# expected: ~100 Hz, tight jitter
```

### Cold-boot test (M1 exit criterion)

1. `sudo poweroff` the Pi.
2. Wait for green LED to go dark.
3. Cut power (unplug / flip main switch).
4. Restore power.
5. Wait ~60 s for boot + ROS 2 setup.sourcing + DDS discovery.
6. Repeat the "Before power-cycle" and "Verify HAL topics" checks above
   from a second machine on the same network (no manual intervention on
   the Pi itself).
7. On the operator phone, open `http://<pi-ip>:8000/` — joystick page loads,
   status pill reads `connected`. Dragging the knob publishes Twist.

If all three pass, M1 bring-up is green.

### Cold-boot test without the PCA9685 (bench validation)

When the PCA9685 isn't wired up, the production YAML's `gpio_backend: pca9685`
will make `motor_driver` exit with `no PCA9685 acking on i2c bus 1 @ 0x40`,
and per `on_exit=Shutdown` that takes the whole launch down. Use the
`switch-platform-config.sh` helper to point the systemd unit at the bench
YAML (mock motor backend, real IMU) for cold-boot validation of the
non-motor stack:

```bash
deploy/scripts/switch-platform-config.sh status        # what's wired now
deploy/scripts/switch-platform-config.sh bench         # mock motor backend
# … power-cycle, validate as above …
deploy/scripts/switch-platform-config.sh prod          # back to PCA9685
```

In bench mode `/test/motor_pwm` carries the per-track PWM commands
motor_driver would have written, useful for sanity-checking the gating
chain (teleop → safety → motor) without hardware in the loop.

## Operator flow (phone teleop)

1. Phone on same Wi-Fi as the rover.
2. Browse to `http://<pi-ip>:8000/` — e.g. `http://192.168.1.128:8000/`.
3. Page auto-connects to `ws://<pi-ip>:9090` (rosbridge).
4. Drag joystick → Twist at 50 Hz to `/hal/cmd_vel_raw`.
5. Release → zero; `motor_driver`'s 500 ms timeout halts the rover if the
   page disconnects.
6. Red `EMERGENCY STOP` button latches publish-zero; tap again to resume.

## Diagnostics / common failures

| Symptom | Check | Likely cause |
|---|---|---|
| `rover-platform-hal` in failed state, `Main process exited, code=exited, status=1` | `journalctl -u rover-platform-hal -n 50` | IMU calibration file missing — re-run step 2 |
| `ros2 topic hz /hal/imu/data` hangs | `journalctl -u rover-platform-hal \| grep -i i2c` | I²C bus/device error — check IMU wiring |
| `rover-teleop-web` fails with "Executable 'rosbridge_websocket' not found" | `apt list --installed \| grep rosbridge` | rosbridge not installed — re-run step 1 |
| Phone gets `connection refused` on port 9090 | `sudo ss -tlnp \| grep 9090` | teleop unit not running, or firewall — check `systemctl status` |
| motor_driver runs but no motion | `ros2 topic echo /hal/cmd_vel_safe` while operator drives | cmd path OK? Check `/hal/cmd_vel_raw → /hal/cmd_vel_safe` pass-through |

## udev (deferred)

The `udev/99-robot-platform.rules` file is a placeholder for USB device
symlinks once the hardware inventory is finalized. Install when populated:

```bash
sudo cp ~/robot-platform/deploy/udev/99-robot-platform.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Log tailing

```bash
journalctl -u rover-platform-hal -u rover-teleop-web -f
```
