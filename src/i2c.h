#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/*
 * ONE I2C stack, and one lock around it.
 *
 * This file used to drive I2C_NUM_0 through the raw IDF driver
 * (i2c_master_cmd_begin) while adc/ina228.h drove the SAME port through Arduino
 * Wire, from a different task on a different core, with nothing serialising them.
 * i2c_write_short() is reachable at runtime from the serial console (`calibrate`,
 * `ina22x-resistor-range` -> setResistorRange), handled in nonRealTimeTask on core
 * 0, while realTimeTask on core 1 reads the INA228s continuously. Interleaving a
 * raw transaction into Wire's in-flight state can corrupt the transfer or wedge the
 * bus, and every failure here lands in an ESP_ERROR_CHECK -> abort(). There is no
 * bus-recovery routine anywhere in this firmware, so a wedge means a power cycle.
 *
 * So: everything goes through Wire, and every complete transaction takes i2cLock().
 * Wire has its own per-call lock, but a register read is beginTransmission ->
 * endTransmission(false) -> requestFrom, and that SEQUENCE has to be atomic too --
 * a repeated-start read is not three independent operations.
 */

class I2cLock {
    static SemaphoreHandle_t mutex() {
        static SemaphoreHandle_t m = xSemaphoreCreateRecursiveMutex();
        return m;
    }

    bool held;

public:
    // Recursive, so a helper that locks may call another that locks.
    explicit I2cLock(TickType_t wait = pdMS_TO_TICKS(2000))
        : held(xSemaphoreTakeRecursive(mutex(), wait) == pdTRUE) {
        if (!held) ESP_LOGE("i2c", "bus lock timeout");
    }

    ~I2cLock() { if (held) xSemaphoreGiveRecursive(mutex()); }

    explicit operator bool() const { return held; }

    I2cLock(const I2cLock &) = delete;
    I2cLock &operator=(const I2cLock &) = delete;
};

/// Write `command` then `len` bytes. Port is always the one Wire.begin() opened.
inline esp_err_t i2c_write_buf(uint8_t, uint8_t address, uint8_t command, const uint8_t *data, uint8_t len) {
    I2cLock lock;
    if (!lock) return ESP_ERR_TIMEOUT;

    Wire.beginTransmission(address);
    if (Wire.write(command) != 1) {
        Wire.endTransmission();
        return ESP_FAIL;
    }
    for (uint8_t i = 0; i < len; ++i) {
        if (Wire.write(data[i]) != 1) {
            Wire.endTransmission();
            return ESP_FAIL;
        }
    }
    return Wire.endTransmission() == 0 ? ESP_OK : ESP_FAIL;
}

inline esp_err_t i2c_write_short(uint8_t port, uint8_t address, uint8_t command, uint16_t data) {
    const uint8_t buf[2] = {(uint8_t) (data >> 8), (uint8_t) (data & 0xFF)};
    esp_err_t ret = i2c_write_buf(port, address, command, buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGE("i2c", "i2c_write_short(addr=0x%02hhX,cmd=0x%02hhX) failed", address, command);
    }
    return ret;
}

/// Register read: write the pointer, repeated start, read `len` bytes.
inline esp_err_t i2c_read_buf(uint8_t, uint8_t address, uint8_t command, uint8_t *buffer, uint8_t len) {
    I2cLock lock;
    if (!lock) return ESP_ERR_TIMEOUT;

    Wire.beginTransmission(address);
    if (Wire.write(command) != 1) {
        Wire.endTransmission();
        return ESP_FAIL;
    }
    if (Wire.endTransmission(false) != 0) return ESP_FAIL; // repeated start, no stop
    if (Wire.requestFrom(address, len) != len) return ESP_FAIL;
    for (uint8_t i = 0; i < len; ++i) buffer[i] = Wire.read();
    return ESP_OK;
}

/// Big-endian 16-bit register. Returns 0 on failure -- callers only use this for
/// ID registers, where 0 is already treated as "not present" (see INA228::init).
inline uint16_t i2c_read_short(uint8_t port, uint8_t address, uint8_t command) {
    uint8_t buf[2] = {0, 0};
    if (i2c_read_buf(port, address, command, buf, 2) != ESP_OK) return 0;
    return (uint16_t) ((buf[0] << 8) | buf[1]);
}

inline bool i2c_test_address(uint8_t addr) {
    I2cLock lock;
    if (!lock) return false;
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}
