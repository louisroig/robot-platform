# xiao_bridge — XIAO ESP32-S3 Sense firmware

PlatformIO + Arduino-ESP32 firmware for the drone-side XIAO ESP32-S3 Sense.
Two roles per [FW-XIAO](../../docs/nodes/firmware-xiao-bridge.html):

1. **Wi-Fi MAVLink bridge** — bidirectional TCP↔UART forwarder. mavros on
   the Pi connects to `xiao-bridge.local:5760` and the firmware shuttles
   bytes to/from the FC's TELEM2 UART.
2. **HTTP camera server** — `GET /capture` triggers an OV2640 JPEG capture,
   `GET /latest.jpg` returns the most-recent frame. Wire contract is
   normative in [ICD-XIAO-001](../../docs/interfaces/icd-xiao-http-api.html).

## Source layout

```
firmware/xiao-bridge/
├── platformio.ini          PlatformIO project — board, framework, build flags
├── include/
│   ├── config.h.example    Template for Wi-Fi creds + ports + camera config
│   └── config.h            (gitignored) Local copy with real credentials
├── src/
│   ├── main.cpp            setup() + loop()
│   ├── wifi_setup.{h,cpp}  Wi-Fi station + mDNS
│   ├── mavlink_bridge.{h,cpp}  TCP server + UART forwarding
│   ├── camera_http.{h,cpp} HTTP server + OV2640 init + capture cache
│   └── camera_pins.h       Hardwired OV2640 pin defs for XIAO Sense
└── lib/                    Empty; arduino-esp32 ships everything we need
```

## First-time bring-up

1. Install PlatformIO Core (one-time, host-side):

   ```
   pipx install platformio
   ```

   The first `pio run` pulls the ESP32-S3 platform packages (~500 MB);
   subsequent builds are fast.

2. Set up local credentials:

   ```
   cd firmware/xiao-bridge
   cp include/config.h.example include/config.h
   # edit include/config.h with your Wi-Fi SSID and PSK
   ```

   `config.h` is gitignored — do not commit it.

3. Build:

   ```
   pio run
   ```

4. Flash (XIAO connected over USB-C):

   ```
   pio run -t upload
   ```

5. Watch the serial monitor:

   ```
   pio device monitor
   ```

   On boot you should see `Wi-Fi: connected, IP=...`,
   `mDNS: xiao-bridge.local advertised`, then the HTTP and TCP banner lines.

## Bench tests

Verifies [ICD-XIAO-001 §9](../../docs/interfaces/icd-xiao-http-api.html#s9)
without a drone.

| Test | Command |
|---|---|
| TEST-XIAO-001 GET /capture works | `curl -v http://xiao-bridge.local/capture > /tmp/capture.jpg && file /tmp/capture.jpg` |
| TEST-XIAO-001 GET /latest.jpg works | `curl -v http://xiao-bridge.local/latest.jpg > /tmp/latest.jpg && file /tmp/latest.jpg` |
| TEST-XIAO-002 second /capture returns 429 | run two `curl` against `/capture` in parallel |
| TEST-XIAO-003 404 before first capture | reset XIAO (button or replug), then `curl -i http://xiao-bridge.local/latest.jpg` |
| TEST-XIAO-004 mDNS advertise within 5 s | `avahi-browse -r _http._tcp` from a host on the same LAN |

## MAVLink bridge

Once the FC's TELEM2 UART is wired (TX→GPIO 44 / RX→GPIO 43 by default —
override in `config.h`), connect mavros:

```
ros2 launch mavros apm.launch fcu_url:=tcp://xiao-bridge.local:5760
```

Single-client policy: a second connection while a client is attached gets
RST immediately. The existing client is not displaced (FW-XIAO §3).

## Known issues

- Real MAVROS-side coordinator (`drone_mission`'s `RealMavrosBackend`)
  isn't implemented yet — the `MockMavrosBackend` is the only thing
  exercised today. End-to-end "rover commands drone via XIAO via mavros"
  needs both this firmware AND the Real backend.
- Wi-Fi 2.4 GHz only; XIAO ESP32-S3 doesn't have a 5 GHz radio
  (FW-XIAO OPEN-02). Image-transfer throughput at yard-scale range is
  uncharacterized.
