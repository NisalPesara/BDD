// Wi-Fi telemetry to the AERO-CUFF website (tourniquet-monitor/, POST /api/telemetry/hardware-feed).
// The ESP32 runs its OWN access point; the server laptop joins it. When the laptop gets its DHCP
// address the ESP32 remembers that IP and sends to http://<laptop-ip>:SERVER_PORT automatically.
//
// Timing: an HTTP POST blocks for 50-300 ms, longer than the MAX30102 FIFO can wait (256 ms).
// So the sampling loop (core 1) only copies a finished 1 s batch into a 1-slot queue (never
// blocks; a newer batch replaces an unsent one) and a separate task on core 0 does the network.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include "config.h"

#if __has_include("device_secrets.h")
#include "device_secrets.h"            // DEVICE_API_KEY + DEVICE_PATIENT_ID (not in source control)
#else
#error "Copy device_secrets.example.h to device_secrets.h and set DEVICE_API_KEY (same as tourniquet-monitor/.env)."
#endif

// One value set per batch, filled by the sketch. NAN = not available -> field is omitted.
struct TelemetrySnapshot {
  float ppg = NAN, vpg = NAN, apg = NAN;
  float heartRate = NAN, spo2 = NAN, perfusionIndex = NAN;
  float relativeAmplitude = NAN, pulseArea = NAN, pulseAgeS = NAN;
  bool  pulseDetected = false, motion = false;
  const char* quality = "unavailable";   // usable | weak_signal | motion | sensor_fault | stale | unavailable
};

class WifiTelemetry {
public:
  static constexpr int MAX_SAMPLES = 128;

  // Status for the sketch to print (written by the network task, read by the loop).
  volatile uint32_t sent = 0, failed = 0;
  volatile int      lastHttp = 0;
  volatile uint32_t serverIp = 0;        // laptop address (0 = not joined yet)
  volatile uint8_t  stations = 0;

  void begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    IPAddress fixed;
    if (fixed.fromString(SERVER_IP)) serverIp = (uint32_t)fixed;   // SERVER_IP "" -> auto-detect
    WiFi.onEvent([this](WiFiEvent_t, WiFiEventInfo_t info) {
      if (strlen(SERVER_IP) == 0) serverIp = info.wifi_ap_staipassigned.ip.addr;
    }, ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);
    queue_ = xQueueCreate(1, sizeof(Batch));
    xTaskCreatePinnedToCore(taskEntry, "wifi_tx", 8192, this, 1, nullptr, 0);
  }

  // Called by the loop for every WIFI_PPG_DECIMATE-th valid sample.
  void addSample(float ppg) { if (fill_.n < MAX_SAMPLES) fill_.wave[fill_.n++] = ppg; }

  // Called by the loop once per WIFI_SEND_PERIOD_MS: hand the batch to the network task.
  void submit(const TelemetrySnapshot& s) {
    fill_.snap = s;
    if (queue_) xQueueOverwrite(queue_, &fill_);
    fill_.n = 0;
  }

private:
  struct Batch { TelemetrySnapshot snap; int n = 0; float wave[MAX_SAMPLES]; };
  Batch fill_, send_;
  QueueHandle_t queue_ = nullptr;
  char json_[3072];

  static void taskEntry(void* self) { static_cast<WifiTelemetry*>(self)->run(); }

  void run() {
    WiFiClient client;
    HTTPClient http;
    http.setReuse(true);                 // keep-alive: one TCP connection for all batches
    for (;;) {
      if (xQueueReceive(queue_, &send_, portMAX_DELAY) != pdTRUE) continue;
      stations = WiFi.softAPgetStationNum();
      if (stations == 0 || serverIp == 0) continue;     // laptop not on the ESP32 network yet
      buildJson(send_);
      char url[96];
      snprintf(url, sizeof url, "http://%s:%d/api/telemetry/hardware-feed",
               IPAddress((uint32_t)serverIp).toString().c_str(), SERVER_PORT);
      http.setConnectTimeout(1500);
      http.setTimeout(2000);
      if (!http.begin(client, url)) { failed++; lastHttp = -1; continue; }
      http.addHeader("Content-Type", "application/json");
      http.addHeader("X-Device-Key", DEVICE_API_KEY);
      lastHttp = http.POST((uint8_t*)json_, strlen(json_));
      if (lastHttp >= 200 && lastHttp < 300) sent++; else failed++;
      if (lastHttp > 0) http.getString();                 // drain the reply so the connection is reusable
      http.end();
    }
  }

  // ---- JSON (fields the server validates; out-of-range values are omitted, not sent) ----
  size_t len_ = 0;
  void put(const char* fmt, ...) {
    if (len_ >= sizeof json_) return;
    va_list ap; va_start(ap, fmt);
    const int w = vsnprintf(json_ + len_, sizeof json_ - len_, fmt, ap);
    va_end(ap);
    if (w > 0) len_ += size_t(w);
  }
  void num(const char* key, float v, float lo, float hi) {
    if (isfinite(v) && v >= lo && v <= hi) put(",\"%s\":%.3f", key, (double)v);
  }
  void buildJson(const Batch& b) {
    const TelemetrySnapshot& s = b.snap;
    len_ = 0;
    put("{\"device\":\"%s\",\"patient_id\":\"%s\"", WIFI_DEVICE_NAME, DEVICE_PATIENT_ID);
    num("ppg", s.ppg, -1e6f, 1e6f);
    num("ppg_first_derivative", s.vpg, -1e6f, 1e6f);
    num("ppg_second_derivative", s.apg, -1e6f, 1e6f);
    num("heart_rate", s.heartRate, 0, 300);
    num("spo2", s.spo2, 0, 100);
    num("perfusion_index", s.perfusionIndex, 0, 100);
    num("relative_systolic_amplitude", s.relativeAmplitude, 0, 1e6f);
    num("pulse_area_auc", s.pulseArea, 0, 1e6f);
    num("pulse_age_seconds", s.pulseAgeS, 0, 86400);
    put(",\"pulse_detected\":%s,\"motion_detected\":%s", s.pulseDetected ? "true" : "false", s.motion ? "true" : "false");
    put(",\"spo2_validity\":\"%s\",\"measurement_quality\":\"%s\"", isfinite(s.spo2) ? "valid" : "unavailable", s.quality);
    int n = 0;
    for (int i = 0; i < b.n; i++) if (isfinite(b.wave[i]) && fabsf(b.wave[i]) < 1e6f) n++;
    if (n > 0) {
      put(",\"ppg_waveform\":[");
      bool first = true;
      for (int i = 0; i < b.n; i++) {
        if (!isfinite(b.wave[i]) || fabsf(b.wave[i]) >= 1e6f) continue;
        put(first ? "%.2f" : ",%.2f", (double)b.wave[i]); first = false;
      }
      put("]");
    }
    put("}");
  }
};
