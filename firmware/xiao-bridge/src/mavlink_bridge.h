// MAVLink TCP↔UART bridge.
//
// Owns Serial1 (FC TELEM2 UART) and a TCP listener on MAVLINK_TCP_PORT.
// Single-client policy per FW-XIAO §3 / ICD-XIAO-001 §4: a second
// connection while a client is already attached is rejected immediately
// (RST), the existing client is not displaced.

#pragma once

namespace xiao_bridge {

// Initialize Serial1 at the configured baud rate and start the TCP server.
// Idempotent.
void mavlink_bridge_init();

// Pump bytes between UART and the connected TCP client. Call from loop().
// Designed to be cheap when no client is connected.
void mavlink_bridge_tick();

}  // namespace xiao_bridge
