
#include <Arduino.h>

#include "tmp117.h"

#include <esp32-hal.h>

#include "../i2c.h"

static constexpr char TAG[] = "tmp117";

/// Log the first failure and then every 64th, so a permanently dead sensor does
/// not drown the log -- but never goes quiet either.
static bool shouldLog(uint32_t consecutiveErrors) {
    return consecutiveErrors == 1 || (consecutiveErrors % 64) == 0;
}

bool PowerSampler_TMP117::init() {
    uint8_t buf[2];

    // Checked here because getStorageId() derives an EEPROM slot from the address,
    // and the caller only calls it once init() has returned true.  Fail closed: an
    // out-of-range part is never registered, so it can never index out of range.
    if (address < ADDR_MIN || address > ADDR_MAX) {
        ESP_LOGE(TAG, "address 0x%02hhX outside TMP117 range 0x%02X..0x%02X", address, ADDR_MIN, ADDR_MAX);
        return false;
    }

    if (i2c_read_buf(0, address, REG_DEVICE_ID, buf, 2) != ESP_OK) {
        ESP_LOGE(TAG, "0x%02hhX: no response reading DEVICE_ID", address);
        return false;
    }
    const uint16_t id = ((uint16_t) buf[0] << 8 | buf[1]) & 0x0FFF;
    if (id != DEVICE_ID) {
        ESP_LOGE(TAG, "0x%02hhX: DEVICE_ID 0x%03X, expected 0x%03X", address, id, DEVICE_ID);
        return false;
    }

    // Soft reset to clear alert/comparator state and start from a known mode.  It
    // does NOT rescue us from an EEPROM the old library may have programmed: reset
    // RELOADS the config register from EEPROM.  What neutralizes a stale stored
    // config is the unconditional write below plus the readback -- do not drop
    // either as "redundant because reset handles it".  Reset takes 1.5 ms.
    if (i2c_write_short(0, address, REG_CONFIGURATION, CONFIG_SOFT_RESET) != ESP_OK) {
        ESP_LOGE(TAG, "0x%02hhX: soft reset failed", address);
        return false;
    }
    delay(5);

    // EEPROM stays locked, so this write is volatile and costs no EEPROM cycles.
    if (i2c_write_short(0, address, REG_CONFIGURATION, CONFIG) != ESP_OK) {
        ESP_LOGE(TAG, "0x%02hhX: config write failed", address);
        return false;
    }

    if (i2c_read_buf(0, address, REG_CONFIGURATION, buf, 2) != ESP_OK) {
        ESP_LOGE(TAG, "0x%02hhX: config readback failed", address);
        return false;
    }
    const uint16_t cfg = ((uint16_t) buf[0] << 8 | buf[1]) & CONFIG_RW_MASK;
    if (cfg != CONFIG) {
        ESP_LOGE(TAG, "0x%02hhX: config is 0x%04X, wrote 0x%04X", address, cfg, CONFIG);
        return false;
    }

    ESP_LOGI(TAG, "0x%02hhX: init ok (id=0x%03X, config=0x%04X)", address, id, cfg);
    return true;
}

float PowerSampler_TMP117::readTemperature() {
    uint8_t buf[2];
    if (i2c_read_buf(0, address, REG_TEMP_RESULT, buf, 2) != ESP_OK) {
        ++readErrors;
        if (shouldLog(readErrors)) ESP_LOGE(TAG, "0x%02hhX: temp read failed (%lu)", address, readErrors);
        return NAN;
    }

    const float t = (float) (int16_t) ((uint16_t) buf[0] << 8 | buf[1]) * RESOLUTION;
    if (!(t >= TEMP_MIN && t <= TEMP_MAX)) {
        ++readErrors;
        if (shouldLog(readErrors)) ESP_LOGE(TAG, "0x%02hhX: temp %.4f out of range (%lu)", address, t, readErrors);
        return NAN;
    }

    readErrors = 0;
    return t;
}

bool PowerSampler_TMP117::hasData() {
    // Both flags are sticky until getSample() consumes them, so re-polling here
    // would be pointless traffic as well as losing the pending state.
    if (dataReady || pollFailed) return true;

    // Rate limit: see the note in tmp117.h.  This gate may only ever suppress the
    // *poll*; it must not suppress a result that was already found, which is why
    // it sits below the sticky-flag check and not above it.
    if (micros() - tLastPoll < POLL_INTERVAL_US) return false;
    tLastPoll = micros();

    uint8_t buf[2];
    if (i2c_read_buf(0, address, REG_CONFIGURATION, buf, 2) != ESP_OK) {
        ++readErrors;
        if (shouldLog(readErrors)) ESP_LOGE(TAG, "0x%02hhX: config poll failed (%lu)", address, readErrors);
        // Report data so getSample() publishes a NAN.  Going quiet instead would
        // make a dead sensor indistinguishable from an idle one.  getSample() will
        // NOT retry the read: the bus just failed, and a second doomed transaction
        // per loop iteration is exactly the load this rate limit exists to avoid.
        pollFailed = true;
        return true;
    }

    // DRDY is cleared by this very read, hence the sticky flag.
    dataReady = (((uint16_t) buf[0] << 8 | buf[1]) & CONFIG_DRDY) != 0;
    return dataReady;
}

Sample PowerSampler_TMP117::getSample() {
    const bool failed = pollFailed;
    dataReady = false;
    pollFailed = false;

    Sample sample{};
    // NAN if unreadable; Sample::temp defaults to NAN anyway.
    sample.temp = failed ? NAN : readTemperature();
    sample.setTimeNow();
    return sample;
}
