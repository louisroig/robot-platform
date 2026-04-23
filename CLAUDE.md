# CLAUDE.md — robot-platform

## What this project is

Autonomous lawn mower robot platform. Two-vehicle system: a tracked rover (teleoperated in
iteration 1) and a drone that maps the yard. Built on ROS 2 Jazzy, Python/rclpy, running on
a Raspberry Pi 5.

The full specification lives at: https://louisroig.github.io/mower-spec/
A local copy is at `spec-site/` in this repo. When in doubt about a requirement or interface
contract, read the spec.

---

## Repo structure

```
robot-platform/
├── CLAUDE.md                  ← you are here
├── README.md
├── ros_ws/                    ← ROS 2 colcon workspace
│   └── src/                   ← ROS 2 packages go here
│       └── platform_hal/      ← HAL nodes (motor_driver, imu_driver, safety_monitor) — TO BUILD
├── firmware/
│   └── xiao-bridge/           ← XIAO MAVLink bridge firmware (M3+)
├── deploy/
│   ├── systemd/               ← systemd service units
│   └── udev/                  ← udev rules for USB/GPIO devices
└── spec-site/                 ← local copy of the HTML spec corpus
    ├── nodes/                 ← SRS documents per node
    ├── interfaces/            ← ICD documents
    ├── verification/          ← test protocols
    └── ...
```

---

## Current milestone: M1 — Rover drives under teleop

**Target:** May 2026
**Exit criterion:** Rover drives a figure-8 from the phone. A cold power cycle brings
everything back green without manual intervention.

### What must be built for M1

| Item | Notes |
|---|---|
| `ros_ws/src/platform_hal/` package | Create — see structure below |
| `motor_driver` node | Skid-steer dual-PWM via 2× IBT-2 (BTS7960B) |
| `imu_driver` node | ISM330DHCX + Madgwick filter |
| `safety_monitor` node (stub) | Pass-through only at M1 |
| `deploy/udev/` rules | USB/GPIO device permissions |
| `deploy/systemd/` units | Auto-start on boot |
| TEST-HAL-009 + unit tests | Must pass before M1 closes |

### Explicitly OUT of M1 scope

- Nav2 / autonomous navigation (iteration 2)
- Real safety gating in safety_monitor (M2)
- Encoder odometry (M5)
- Battery telemetry
- Drone / XIAO bridge code (M3)
- Geofence enforcement (M5)

---

## Hardware

| Component | Details |
|---|---|
| Compute | Raspberry Pi 5 |
| Motor controller | 2× IBT-2 43 A H-bridge modules (BTS7960B internal) |
| Drive | Aluminum tracks, skid-steer kinematics |
| IMU | ISM330DHCX (6-DoF) — I²C bus 1 |
| GPIO library | `lgpio` or `gpiod` (Pi 5 compatible) |
| Vision (M2+) | OAK-D on NPU |
| Drone (M3+) | ArduCopter + XIAO bridge (see `firmware/xiao-bridge/`) |

**GPIO BCM pin assignments frozen in `spec-site/hardware/hw-pi5-001-rover-wiring.html` §3:
left track RPWM/LPWM on GPIO 12/13, right track on GPIO 18/19 (all hardware-PWM channels).**

---

## Software stack

- ROS 2 Jazzy on Ubuntu 24.04
- Python 3 / rclpy
- `sensor_msgs`, `geometry_msgs`, `diagnostic_msgs`
- `smbus2` for I²C (IMU)
- `lgpio` or `gpiod` for GPIO/PWM

---

## platform_hal package (to create at ros_ws/src/platform_hal/)

```
ros_ws/src/platform_hal/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── platform_hal
├── platform_hal/
│   ├── __init__.py
│   ├── gpio_backend.py     ← GPIO abstraction (LgpioBackend + MockGpioBackend)
│   ├── motor_driver.py     ← SRS-HAL-001
│   ├── imu_driver.py       ← SRS-HAL-002
│   └── safety_monitor.py   ← SRS-SAF-001 (stub at M1)
└── test/
    ├── fixtures/
    │   └── mock_motor_driver.py   ← mock GPIO for CI
    ├── test_motor_driver.py       ← kinematics + timeout unit tests
    └── test_hal_009.py            ← launch_testing integration test
```

---

## Node specs (M1)

### motor_driver (SRS-HAL-001)
- **Sub:** `/hal/cmd_vel_safe` (Twist, 50 Hz)
- **Pub:** `/diagnostics` (DiagnosticArray, 1 Hz)
- **Does:** Twist → skid-steer kinematics → dual-PWM (RPWM + LPWM per track) on
  4 GPIO pins via 2× IBT-2 (BTS7960B). RPWM = forward duty, LPWM = reverse duty,
  mutually exclusive (H-bridge short prohibited).
- **Key params:** `track_width_m` 0.28 m · `max_linear_vel` 0.7 m/s · `max_angular_vel`
  1.5 rad/s · `pwm_frequency_hz` 2000 · `cmd_vel_timeout_ms` 500
- GPIO backend pluggable via `gpio_backend` param (`lgpio` for hardware, `mock` for tests).
- **Safety:** Hold zero on startup until first valid message. Safe-halt if no message
  in 500 ms. (satisfies SR-005, SR-008)
- **Spec:** `spec-site/nodes/srs-motor-driver.html`

### imu_driver (SRS-HAL-002)
- **Pub:** `/hal/imu/data` (Imu, 100 Hz) · `/diagnostics`
- **Does:** Reads ISM330DHCX (6-DoF) over I²C, Madgwick filter (gyro+accel),
  publishes orientation in REP-103 convention
- **Key params:** `i2c_bus` 1 · `imu_rate_hz` 100 ·
  `calibration_file` `~/.config/platform/imu_calibration.yaml` · `madgwick_beta` 0.1
- **Safety:** Refuses to start without calibration file. On I²C fault, stops publishing
  (never emits stale values — triggers safety_monitor staleness at M2).
- **First step:** Run IMU calibration tool and commit `imu_calibration.yaml` before
  coding the node.
- **Spec:** `spec-site/nodes/srs-imu-driver.html`

### safety_monitor (SRS-SAF-001) — STUB AT M1
- **Sub:** `/hal/cmd_vel_raw` (Twist)
- **Pub:** `/hal/cmd_vel_safe` (Twist)
- **M1 behavior:** Unconditional pass-through. No gating.
- **M2 behavior:** Real gate — e-stop, tilt, person-detect, heartbeat.
- **Spec:** `spec-site/nodes/srs-safety-monitor.html`

---

## Key ICDs (M1)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/hal/cmd_vel_raw` | geometry_msgs/Twist | 50 Hz | From teleop |
| `/hal/cmd_vel_safe` | geometry_msgs/Twist | 50 Hz | Pass-through at M1 |
| `/hal/imu/data` | sensor_msgs/Imu | 100 Hz | Orientation + accel + gyro |

Full ICDs: `spec-site/interfaces/` or https://louisroig.github.io/mower-spec/interfaces/

---

## M1 gating test: TEST-HAL-009

Motor driver command timeout — must pass before M1 closes.

- Publishes Twist at 50 Hz, stops, verifies both tracks zero within 500 ms
- Runs 10 trials; all must be ≤ 600 ms; no late non-zero commands
- Automated via `launch_testing` — runs on every PR
- Uses `test/fixtures/mock_motor_driver.py` (no hardware needed for CI)
- **Spec:** `spec-site/verification/test-hal-009.html`

---

## Build and run

```bash
cd ~/projects/robot-platform/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Individual nodes
ros2 run platform_hal motor_driver --ros-args --params-file ../config/motor_driver.yaml
ros2 run platform_hal imu_driver
ros2 run platform_hal safety_monitor

# Tests (no hardware needed)
colcon test --packages-select platform_hal
colcon test-result --verbose
```

---

## Coding conventions

- Python 3, rclpy, standard ROS 2 Jazzy patterns
- Parameters via `declare_parameter` / `get_parameter`, loaded from YAML
- Diagnostics at 1 Hz via `diagnostic_msgs/DiagnosticArray`
- No bare `except:` — catch specific exceptions and log with the node logger
- Safety-critical paths: comment citing the SR-XXX requirement being satisfied
- Tests: pytest for unit, `launch_testing` for integration
