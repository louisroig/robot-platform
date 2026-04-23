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
