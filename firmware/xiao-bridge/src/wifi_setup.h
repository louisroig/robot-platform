// Wi-Fi station-mode bring-up + mDNS responder for xiao_bridge.
// FW-XIAO §3 / ICD-XIAO-001 §4.

#pragma once

namespace xiao_bridge {

// Connect to the configured Wi-Fi network (blocking, with retry) and
// register the mDNS hostname. Returns true on success.
//
// On failure, the caller should reboot via the watchdog rather than
// soldier on without a network — every other module on the firmware
// depends on having a working IP stack.
bool wifi_and_mdns_init();

}  // namespace xiao_bridge
