#include "wifi_setup.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include "config.h"

namespace xiao_bridge {

namespace {
constexpr uint32_t kConnectTimeoutMs = 30000;
constexpr uint32_t kRetryDelayMs = 250;
}  // namespace

bool wifi_and_mdns_init() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   // disable Wi-Fi modem sleep — we're running on
                            // a drone with constant power, not a coin cell,
                            // and the latency hit on TCP/HTTP isn't worth it.
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    log_i("Wi-Fi: connecting to %s ...", WIFI_SSID);
    const uint32_t deadline = millis() + kConnectTimeoutMs;
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() > deadline) {
            log_e("Wi-Fi: connect timeout after %u ms", kConnectTimeoutMs);
            return false;
        }
        delay(kRetryDelayMs);
    }
    log_i("Wi-Fi: connected, IP=%s, RSSI=%d dBm",
          WiFi.localIP().toString().c_str(), WiFi.RSSI());

    if (!MDNS.begin(MDNS_HOSTNAME)) {
        log_e("mDNS: begin(%s) failed", MDNS_HOSTNAME);
        return false;
    }
    // Advertise the two services consumers care about (per ICD-XIAO-001).
    // mavros uses a raw TCP discovery, but registering the service lets
    // bench tools (e.g. avahi-browse) find the bridge during debugging.
    MDNS.addService("mavlink-tcp", "tcp", MAVLINK_TCP_PORT);
    MDNS.addService("http", "tcp", HTTP_PORT);
    log_i("mDNS: %s.local advertised", MDNS_HOSTNAME);
    return true;
}

}  // namespace xiao_bridge
