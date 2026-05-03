// xiao_bridge — Wi-Fi MAVLink bridge + OV2640 image-capture HTTP server.
// Specs: FW-XIAO (firmware) + ICD-XIAO-001 (consumer-side wire contract).

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "camera_http.h"
#include "config.h"
#include "mavlink_bridge.h"
#include "wifi_setup.h"

namespace {
// Watchdog: bench-tested loop iteration is sub-millisecond. 5 s gives
// generous headroom for a misbehaving HTTP request handler while still
// catching a hard hang quickly. FW-XIAO §6 calls for ESP-IDF watchdog
// reset on main-loop hang.
constexpr uint32_t kTaskWdtTimeoutSeconds = 5;
}  // namespace

void setup() {
    Serial.begin(115200);
    // Give the USB CDC enumeration a moment so early log lines aren't lost.
    delay(100);
    log_i("xiao_bridge booting (build " __DATE__ " " __TIME__ ")");

    // Bring up Wi-Fi + mDNS first so a failure here surfaces fast and
    // doesn't waste seconds on camera init we can't use anyway.
    if (!xiao_bridge::wifi_and_mdns_init()) {
        log_e("setup: Wi-Fi/mDNS init failed — restarting in 5s");
        delay(5000);
        ESP.restart();
    }

    if (!xiao_bridge::camera_http_init()) {
        log_e("setup: camera/HTTP init failed — restarting in 5s");
        delay(5000);
        ESP.restart();
    }

    xiao_bridge::mavlink_bridge_init();

    // Subscribe THIS task to the task watchdog. The Arduino-ESP32 IDF
    // initializes the WDT for the IDLE tasks of both cores at 5 s by
    // default; we add the loop task here so a hung loop also resets us.
    // (Older esp-idf API in this arduino-esp32 release; the v5+ struct
    // form arrives whenever PlatformIO ships arduino-esp32 ≥ 3.0.)
    esp_task_wdt_init(kTaskWdtTimeoutSeconds, true);
    esp_task_wdt_add(nullptr);

    log_i("setup: complete");
}

void loop() {
    xiao_bridge::mavlink_bridge_tick();
    xiao_bridge::camera_http_tick();
    esp_task_wdt_reset();
    // No delay — both tick functions are cheap when idle, and we want
    // sub-millisecond latency on the MAVLink path. The IDLE task gets
    // CPU when we're not doing anything.
}
