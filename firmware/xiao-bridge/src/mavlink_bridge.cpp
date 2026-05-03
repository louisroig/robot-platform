#include "mavlink_bridge.h"

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"

namespace xiao_bridge {

namespace {

WiFiServer g_server(MAVLINK_TCP_PORT);
WiFiClient g_client;
bool g_initialized = false;

// 512 B chosen to fit one or two MAVLink v2 packets per pump call;
// matches the XIAO's UART HW FIFO of 256 B with one packet of headroom.
constexpr size_t kPumpBufferBytes = 512;

void set_no_delay_if_open(WiFiClient& c) {
    if (c && c.connected()) {
        // Disable Nagle — every byte mavros writes is part of a tiny
        // MAVLink heartbeat or command; latency matters more than
        // throughput on this link.
        c.setNoDelay(true);
    }
}

}  // namespace

void mavlink_bridge_init() {
    if (g_initialized) return;
    Serial1.begin(
        MAVLINK_UART_BAUD,
        SERIAL_8N1,
        MAVLINK_UART_RX_PIN,
        MAVLINK_UART_TX_PIN
    );
    g_server.begin();
    g_server.setNoDelay(true);
    g_initialized = true;
    log_i("mavlink_bridge: TCP listening on :%u, UART1 %d8N1 RX=%d TX=%d",
          MAVLINK_TCP_PORT, MAVLINK_UART_BAUD,
          MAVLINK_UART_RX_PIN, MAVLINK_UART_TX_PIN);
}

void mavlink_bridge_tick() {
    if (!g_initialized) return;

    // Accept-or-reject incoming connections.
    if (g_server.hasClient()) {
        WiFiClient incoming = g_server.accept();
        if (!g_client || !g_client.connected()) {
            g_client = incoming;
            set_no_delay_if_open(g_client);
            log_i("mavlink_bridge: client connected from %s",
                  g_client.remoteIP().toString().c_str());
        } else {
            // Single-client policy (FW-XIAO §3, REQ-ICD-XIAO-001-03 cousin).
            // Refuse without disturbing the active client.
            log_w("mavlink_bridge: refusing second client from %s "
                  "(already serving %s)",
                  incoming.remoteIP().toString().c_str(),
                  g_client.remoteIP().toString().c_str());
            incoming.stop();
        }
    }

    // If our client dropped, drain the slot so a new one can take it.
    if (g_client && !g_client.connected()) {
        log_i("mavlink_bridge: client disconnected");
        g_client.stop();
    }

    // TCP → UART.
    if (g_client && g_client.connected() && g_client.available()) {
        uint8_t buf[kPumpBufferBytes];
        const int avail = g_client.available();
        const int to_read = avail < (int)sizeof(buf) ? avail : (int)sizeof(buf);
        const int n = g_client.read(buf, to_read);
        if (n > 0) {
            Serial1.write(buf, n);
        }
    }

    // UART → TCP.
    if (Serial1.available()) {
        uint8_t buf[kPumpBufferBytes];
        const int avail = Serial1.available();
        const int to_read = avail < (int)sizeof(buf) ? avail : (int)sizeof(buf);
        const int n = Serial1.read(buf, to_read);
        if (n > 0 && g_client && g_client.connected()) {
            g_client.write(buf, n);
        }
        // If no client, the bytes are silently dropped — there's no point
        // buffering FC telemetry the operator isn't listening to.
    }
}

}  // namespace xiao_bridge
