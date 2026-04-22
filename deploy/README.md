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
