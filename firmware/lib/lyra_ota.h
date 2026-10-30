// =============================================================================
// lyra_ota.h  -  Firmware update over BLE (the ring is sealed: no USB after assembly)
// =============================================================================
// Two characteristics added to the ring's existing LYRA service:
//   OTA_CTRL (write, with response): "BEGIN <bytes>"  "END"  "ABORT"
//   OTA_DATA (write, with response): raw .bin chunks, in order (<= MTU - 3 bytes each)
// Progress / result go out as text lines on the normal TX characteristic:
//   "OTA READY", "OTA <percent>", "OTA OK" (then reboot), "OTA ERR <reason>"
// Host side: tools/ota_ring.py (Python + bleak) sends the .bin from Arduino's
// "Export Compiled Binary". Update.end(true) checks the image before it is marked bootable,
// and the ESP32 bootloader keeps the old app if the new one fails to start, so a broken
// transfer can't brick the ring.
// Power: refused below OTA_MIN_PCT unless charging (flash writes + BLE for ~1 min).
#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Update.h>

#define OTA_CTRL_UUID "6E400204-B5A3-F393-E0A9-E50E24DCCA9E"
#define OTA_DATA_UUID "6E400205-B5A3-F393-E0A9-E50E24DCCA9E"

static constexpr uint8_t OTA_MIN_PCT = 30;

namespace lyra_ota {
  inline void (*sendLine)(const char*) = nullptr;   // set to the sketch's send()
  inline bool (*powerOk)() = nullptr;              // battery / charging check
  inline bool active = false;
  inline size_t total = 0, written = 0;
  inline int lastPct = -1;
  inline volatile bool rebootPending = false;

  inline void say(const char* s) { if (sendLine) sendLine(s); }

  class CtrlCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&) override {
      std::string v = ch->getValue();
      if (v.rfind("BEGIN ", 0) == 0) {
        size_t n = strtoul(v.c_str() + 6, nullptr, 10);
        if (powerOk && !powerOk()) { say("OTA ERR battery"); return; }
        if (active) Update.abort();
        if (n == 0 || !Update.begin(n)) { active = false; say("OTA ERR begin"); return; }
        active = true; total = n; written = 0; lastPct = -1;
        say("OTA READY");
      } else if (v == "END") {
        if (!active) { say("OTA ERR not started"); return; }
        active = false;
        if (written != total) { Update.abort(); say("OTA ERR size"); return; }
        if (!Update.end(true)) { say("OTA ERR verify"); return; }
        say("OTA OK");
        rebootPending = true;                       // reboot from loop(), after the reply is sent
      } else if (v == "ABORT") {
        if (active) Update.abort();
        active = false; say("OTA ABORTED");
      }
    }
  };

  class DataCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&) override {
      if (!active) return;
      std::string v = ch->getValue();
      if (written + v.size() > total || Update.write((uint8_t*)v.data(), v.size()) != v.size()) {
        Update.abort(); active = false; say("OTA ERR write"); return;
      }
      written += v.size();
      int pct = (int)(written * 100 / total);
      if (pct / 10 != lastPct / 10) { lastPct = pct; char b[16]; snprintf(b, sizeof(b), "OTA %d", pct); say(b); }
    }
  };

  // call while building the service, before svc->start()
  inline void addTo(NimBLEService* svc) {
    svc->createCharacteristic(OTA_CTRL_UUID, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CtrlCB());
    svc->createCharacteristic(OTA_DATA_UUID, NIMBLE_PROPERTY::WRITE)->setCallbacks(new DataCB());
  }
  // call from loop(): reboots into the new firmware once "OTA OK" has gone out
  inline void service() {
    static uint32_t t0 = 0;
    if (!rebootPending) return;
    if (!t0) t0 = millis();
    if (millis() - t0 > 500) ESP.restart();
  }
  inline bool busy() { return active || rebootPending; }
}
