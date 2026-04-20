# robot-platform

Ground-air autonomous platform — iteration 1. Solo build, targeting an M6 field test demo (Oct 2026).

## Layout

- `spec-site/` — HTML specification corpus. Open `spec-site/index.html` in any browser. See `spec-site/SPEC-README.md` for details on the corpus.
- `ros_ws/` — ROS 2 Jazzy workspace. Source `/opt/ros/jazzy/setup.bash`, then `colcon build` from inside this directory.
- `firmware/` — Custom firmware. `xiao-bridge/` hosts the XIAO ESP32-S3 MAVLink + camera bridge.
- `deploy/` — udev rules and systemd units for the Pi 5 rover.

## Target hardware

- **Rover:** Raspberry Pi 5 (8 GB), Ubuntu Server 24.04 LTS, ROS 2 Jazzy.
- **Drone:** iFlight Blitz Whoop F7 AIO (ArduCopter 4.5) + XIAO ESP32-S3 Sense bridge.
- **Operator:** Mobile app over Wi-Fi (REST + WebSocket).

See [`spec-site/architecture/deployment.html`](spec-site/architecture/deployment.html) for hardware allocation and [`spec-site/architecture/iteration-roadmap.html`](spec-site/architecture/iteration-roadmap.html) for the iteration plan.
