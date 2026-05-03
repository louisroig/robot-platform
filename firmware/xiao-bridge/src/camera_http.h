// HTTP camera-capture API.
//
// FW-XIAO §4 / ICD-XIAO-001 §3.
//
//   GET /capture     trigger a fresh capture, return JPEG bytes.
//                    Returns 503 on camera fault, 429 if another
//                    /capture is already in flight.
//
//   GET /latest.jpg  return the most-recent capture without re-triggering.
//                    Returns 404 if no capture has happened since boot.
//
// Capture is 1600×1200 (UXGA), JPEG quality CAMERA_JPEG_QUALITY (lower
// = better; ESP32-camera convention 0..63).

#pragma once

namespace xiao_bridge {

// Initialize the OV2640 camera and start the HTTP server. Returns true on
// success; on failure the caller should reboot.
bool camera_http_init();

// Pump the HTTP server. Call from loop().
void camera_http_tick();

}  // namespace xiao_bridge
