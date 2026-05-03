#include "camera_http.h"

#include <Arduino.h>
#include <WebServer.h>
#include <esp_camera.h>

#include "camera_pins.h"
#include "config.h"

namespace xiao_bridge {

namespace {

WebServer g_http(HTTP_PORT);

// Cached "most recent" frame. esp_camera owns the underlying buffer
// (PSRAM-backed), so we copy on capture into our own owned buffer to
// keep the FB free for the next capture.
struct CachedFrame {
    uint8_t* data = nullptr;
    size_t   len  = 0;
    bool     valid = false;
};
CachedFrame g_latest;

// Single-flight lock for /capture. Per ICD-XIAO-001 §6, overlapping
// /capture requests get 429; the camera library is single-threaded and
// concurrent triggers would corrupt each other's frame buffers anyway.
volatile bool g_capture_in_flight = false;

void free_latest() {
    if (g_latest.data != nullptr) {
        free(g_latest.data);
        g_latest.data = nullptr;
        g_latest.len = 0;
        g_latest.valid = false;
    }
}

// Replace the cached frame with a fresh copy of `fb`. Returns false on
// allocation failure (out of PSRAM).
bool cache_frame(const camera_fb_t* fb) {
    free_latest();
    g_latest.data = static_cast<uint8_t*>(ps_malloc(fb->len));
    if (g_latest.data == nullptr) {
        log_e("camera_http: ps_malloc(%u) failed — out of PSRAM", (unsigned)fb->len);
        return false;
    }
    memcpy(g_latest.data, fb->buf, fb->len);
    g_latest.len = fb->len;
    g_latest.valid = true;
    return true;
}

void send_jpeg(const uint8_t* data, size_t len) {
    g_http.sendHeader("Content-Type", "image/jpeg");
    g_http.sendHeader("Content-Length", String((unsigned)len));
    g_http.sendHeader("Cache-Control", "no-store");
    g_http.send_P(200, "image/jpeg",
                  reinterpret_cast<const char*>(data), len);
}

void send_plain(int code, const char* body) {
    g_http.send(code, "text/plain", body);
}

void handle_capture() {
    if (g_capture_in_flight) {
        send_plain(429, "capture_in_progress");
        return;
    }
    g_capture_in_flight = true;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == nullptr) {
        g_capture_in_flight = false;
        log_w("camera_http: esp_camera_fb_get returned null");
        send_plain(503, "camera_timeout");
        return;
    }
    const bool cached = cache_frame(fb);
    // Send the freshly-captured bytes inline (REQ-ICD-XIAO-001-02:
    // Content-Length matches Content-Length matches the JPEG byte count).
    if (cached) {
        send_jpeg(g_latest.data, g_latest.len);
    } else {
        // Fall back to sending fb->buf directly even though we couldn't
        // cache. The next /latest.jpg will 404 since g_latest.valid is false.
        send_jpeg(fb->buf, fb->len);
    }
    esp_camera_fb_return(fb);
    g_capture_in_flight = false;
}

void handle_latest() {
    if (!g_latest.valid) {
        send_plain(404, "no_capture_since_boot");
        return;
    }
    send_jpeg(g_latest.data, g_latest.len);
}

void handle_not_found() {
    send_plain(404, "not_found");
}

}  // namespace

bool camera_http_init() {
    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer = LEDC_TIMER_0;
    cfg.pin_d0 = Y2_GPIO_NUM;
    cfg.pin_d1 = Y3_GPIO_NUM;
    cfg.pin_d2 = Y4_GPIO_NUM;
    cfg.pin_d3 = Y5_GPIO_NUM;
    cfg.pin_d4 = Y6_GPIO_NUM;
    cfg.pin_d5 = Y7_GPIO_NUM;
    cfg.pin_d6 = Y8_GPIO_NUM;
    cfg.pin_d7 = Y9_GPIO_NUM;
    cfg.pin_xclk = XCLK_GPIO_NUM;
    cfg.pin_pclk = PCLK_GPIO_NUM;
    cfg.pin_vsync = VSYNC_GPIO_NUM;
    cfg.pin_href = HREF_GPIO_NUM;
    cfg.pin_sccb_sda = SIOD_GPIO_NUM;
    cfg.pin_sccb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn = PWDN_GPIO_NUM;
    cfg.pin_reset = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size = FRAMESIZE_UXGA;            // 1600x1200 per FW-XIAO §4
    cfg.jpeg_quality = CAMERA_JPEG_QUALITY;
    cfg.fb_count = 2;                           // double-buffer for steady capture
    cfg.fb_location = CAMERA_FB_IN_PSRAM;       // 1600x1200 ≫ 320 KB SRAM
    cfg.grab_mode = CAMERA_GRAB_LATEST;

    const esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        log_e("camera_http: esp_camera_init failed (0x%x)", err);
        return false;
    }
    log_i("camera_http: OV2640 init OK (UXGA, JPEG quality %d)",
          CAMERA_JPEG_QUALITY);

    g_http.on("/capture", HTTP_GET, handle_capture);
    g_http.on("/latest.jpg", HTTP_GET, handle_latest);
    g_http.onNotFound(handle_not_found);
    g_http.begin();
    log_i("camera_http: HTTP listening on :%u", HTTP_PORT);
    return true;
}

void camera_http_tick() {
    g_http.handleClient();
}

}  // namespace xiao_bridge
