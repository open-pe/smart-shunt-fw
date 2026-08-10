#pragma once

#include <Arduino.h>
#include <EEPROM.h>

#ifdef TARGET_STM32H5
#include "esp_compat.h"
constexpr uint8_t AUX_PIN = PA8;
#else
#include <driver/gpio.h>
#include <esp_log.h>

/*
 * Aux switch: one general-purpose GPIO output, driven from BLE or the console.
 *
 * GPIO 9 is free (undeclared in settings.h), not a strapping pin, not USB, not flash, and is
 * RTC-capable -- that last part is what lets it hold its level through deep sleep. Confirm the pin
 * is actually broken out on the module before wiring: hw/ documents the INA228 sensor board (whose
 * MCU sheet is an ESP-12F), not the ESP32-S3 module the firmware runs on.
 *
 * The pin FLOATS from reset until auxBegin() runs. Nothing in firmware can cover that window, so the
 * driver gate needs an external pulldown to keep the load off through boot.
 */

constexpr gpio_num_t AUX_PIN = GPIO_NUM_9;
#endif
constexpr bool AUX_ACTIVE_HIGH = true;

// Raw EEPROM/NVS byte offsets -- deliberately NOT slot indices. Calibration occupies bytes 16..111
// today and its assert(ecIndex <= 16) reserves through 151, and the slot scheme is already
// over-subscribed (slot 4 is shared by INA226 range and INA228@0x42, slot 8 by TMP117@0x4B and an
// INA228 range), so allocating another index would be asking for a third collision.
constexpr int AUX_NVS_MAGIC_ADDR = 152;
constexpr int AUX_NVS_STATE_ADDR = 153;
constexpr uint8_t AUX_NVS_MAGIC = 0x5A;
constexpr size_t AUX_NVS_SIZE = 256; // same image size every other EEPROM user opens

static bool auxState = false;

inline bool auxGet() { return auxState; }

/// Read the persisted state without trusting an unwritten blob. A virgin NVS image reads back as
/// zeros, so without the magic byte "never written" and "written off" are indistinguishable -- and
/// any encoding where the *absence* of a record could mean "on" would energise a load on a blank
/// device. Off is the only safe reading of "I don't know".
inline bool auxReadPersisted() {
#ifdef TARGET_STM32H5
    uint8_t magic = EEPROM.read(AUX_NVS_MAGIC_ADDR);
    uint8_t st = EEPROM.read(AUX_NVS_STATE_ADDR);
    if (magic != AUX_NVS_MAGIC) return false;
    return st == 1;
#else
    EEPROM.begin(AUX_NVS_SIZE);
    uint8_t magic = EEPROM.read(AUX_NVS_MAGIC_ADDR);
    uint8_t st = EEPROM.read(AUX_NVS_STATE_ADDR);
    EEPROM.end();
    if (magic != AUX_NVS_MAGIC) return false;
    return st == 1;
#endif
}

inline void auxWritePersisted(bool on) {
#ifdef TARGET_STM32H5
    EEPROM.write(AUX_NVS_MAGIC_ADDR, AUX_NVS_MAGIC);
    EEPROM.write(AUX_NVS_STATE_ADDR, on ? 1 : 0);
#else
    EEPROM.begin(AUX_NVS_SIZE);
    EEPROM.write(AUX_NVS_MAGIC_ADDR, AUX_NVS_MAGIC);
    EEPROM.write(AUX_NVS_STATE_ADDR, on ? 1 : 0);
    EEPROM.commit();
    EEPROM.end();
#endif
}

inline void auxDrive(bool on) {
    digitalWrite(AUX_PIN, (on == AUX_ACTIVE_HIGH) ? HIGH : LOW);
}

/// Call early in setup(), before the long init work -- on a deep-sleep wake the load has been
/// carried across by the pad hold and should be handed back to the firmware promptly.
inline void auxBegin() {
    auxState = auxReadPersisted();

    // Order matters coming out of deep sleep, where the pad is still held at its old level:
    // configure the peripheral FIRST, while the hold masks it, then release. Releasing first would
    // let the pad follow its reset default for the microseconds until digitalWrite lands -- a blink
    // of the load on every wake.
    pinMode(AUX_PIN, OUTPUT);
    auxDrive(auxState);
#ifdef TARGET_STM32H5
    // No deep-sleep pad hold on STM32H5 (Stop mode not implemented yet)
#else
    gpio_hold_dis(AUX_PIN);
#endif

    ESP_LOGI("aux", "aux switch on GPIO%d = %s (restored)", (int) AUX_PIN, auxState ? "ON" : "off");
}

/// App-task only. Persists only on an actual change: EEPROM.commit() rewrites the whole 256-byte
/// blob, and re-asserting a value that is already stored would burn a flash cycle for nothing.
inline void auxSet(bool on) {
    if (on == auxState) return;
    auxState = on;
    auxDrive(on);
    auxWritePersisted(on);
    ESP_LOGI("aux", "aux switch -> %s", on ? "ON" : "off");
}

/// Latch the pad so the level survives deep sleep and the wake reset. Without this the switch would
/// drop out for the whole boot interval every time the idle sleep cycles -- every 15 minutes.
inline void auxArmDeepSleepHold() {
#ifdef TARGET_STM32H5
    // No deep-sleep pad hold on STM32H5 yet
#else
    gpio_hold_en(AUX_PIN);
    gpio_deep_sleep_hold_en();
#endif
}

/// Parse a command payload. Accepts the ASCII digits and raw bytes a generic BLE app would send as
/// well as words, so the switch is usable from something other than our own tooling.
/// Returns false if nothing recognisable was found; `out` is only written on success.
inline bool auxParseCommand(const uint8_t *data, size_t len, bool &out) {
    if (!data || !len) return false;
    if (len == 1 && (data[0] == 0 || data[0] == 1)) { out = data[0] != 0; return true; }

    String s;
    for (size_t i = 0; i < len; ++i) s += (char) data[i];
    s.trim();
    s.toLowerCase();

    if (s == "1" || s == "on" || s == "true") { out = true; return true; }
    if (s == "0" || s == "off" || s == "false") { out = false; return true; }
    if (s == "toggle") { out = !auxGet(); return true; }
    return false;
}
