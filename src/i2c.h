#pragma once

#include <Arduino.h>
#include <Wire.h>
#ifdef TARGET_STM32H5
#include "esp_compat.h"
#include <FreeRTOS.h>
#include <semphr.h>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
#include "i2c_recovery.h"

#if defined(ESP32) && !defined(TARGET_STM32H5)
#define I2C_RECOVERY 1
#include <esp_timer.h>
#include <esp32-hal-i2c.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
#include <driver/i2c_master.h>
#endif
#else
#define I2C_RECOVERY 0
#endif

/*
 * One Wire stack per hardware controller, with one lock per stack.
 *
 * This file used to drive I2C_NUM_0 through the raw IDF driver
 * (i2c_master_cmd_begin) while adc/ina228.h drove the SAME port through Arduino
 * Wire, from a different task on a different core, with nothing serialising them.
 * i2c_write_short() is reachable at runtime from the serial console (`calibrate`,
 * `ina22x-resistor-range` -> setResistorRange), handled in nonRealTimeTask on core
 * 0, while realTimeTask on core 1 reads the INA228s continuously. Interleaving a
 * raw transaction into Wire's in-flight state can corrupt the transfer or wedge the
 * bus, and every failure here landed in an ESP_ERROR_CHECK -> abort().
 *
 * So: everything goes through the selected TwoWire instance, and every complete
 * transaction takes its bus lock. Wire has its own per-call lock, but a register read is beginTransmission ->
 * endTransmission(false) -> requestFrom, and that SEQUENCE has to be atomic too --
 * a repeated-start read is not three independent operations.
 *
 * BUS RECOVERY (ESP32 only). Every transfer reports its outcome to i2c_note(). A
 * streak of failures (I2cRecoveryPolicy: >= 4 failures spanning >= 200 ms, with a
 * 0.5..30 s backoff between attempts) runs i2c_recover_bus() inline, in the task that
 * saw the failure, under the bus lock: reset the IDF driver, tear Wire down, clock SCL
 * up to 9 times until a slave holding SDA lets go, send a STOP, re-begin Wire with
 * the same pins and clock. Then the bus generation is bumped, and each sampler that
 * sees a new generation rewrites its device configuration (a part upset badly enough
 * to wedge the bus may also have reset). Seen 2026-10-10: a boost converter switching
 * hard next to the board made every transfer fail with 259 (ESP_ERR_INVALID_STATE,
 * which this driver also returns for a NACK) and stopped all telemetry for ~26 s.
 * Never call any of this from an ISR -- the INA228 ALERT handlers only set a flag.
 * Console `i2c` shows the counters; `i2c recover` forces one.
 */

/// Physical-layer check of the two bus pins, BEFORE Wire claims them.
///
/// Exists because every electrical fault on this bus reaches the driver as the same
/// opaque ESP_ERR_INVALID_STATE (259) on every transfer, which reads nothing like
/// the NACK an absent device produces -- shorted lines, a line stuck low and a part
/// with no supply are indistinguishable from firmware once the driver is running.
/// These four assertions separate them while the pins are still plain GPIOs.
///
/// Returns true only if every check actually ran and passed. It never returns true
/// for "could not tell": there is no such path here, since each step is a direct
/// GPIO read. A failure is logged and reported, not fatal -- on the shunt-adc board
/// the ADS1262 is on SPI and must still come up with the I2C harness unplugged.
///
/// What it CANNOT see: a device with no supply that is phantom-powered through its
/// ESD diodes. That part still lets both lines idle high and still ACKs its address,
/// so it passes every check here and then fails every real transfer. If this passes
/// and transfers still fail with 259, measure V+ at the device -- see i2c_read_buf.
inline bool i2c_check_pins(uint8_t sda, uint8_t scl) {
    const char *TAG = "i2c";
    bool ok = true;

    // 1. Both lines idle high on the internal pull-ups. Low here = short to GND, a
    //    device holding the line, or a wire on the wrong pin.
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delayMicroseconds(50); // ~45k against line capacitance
    const int sdaIdle = digitalRead(sda), sclIdle = digitalRead(scl);
    if (!sdaIdle || !sclIdle) {
        ESP_LOGE(TAG, "STUCK LOW: SDA(%hhu)=%d SCL(%hhu)=%d -- short to GND, or a device "
                      "holding the line", sda, sdaIdle, scl, sclIdle);
        ok = false;
    }

    // 2/3. Drive each line low in turn; the OTHER must stay high. If it follows, the
    //      two are shorted together -- the classic swapped/bridged-pin fault, which
    //      otherwise looks exactly like a dead bus.
    struct { uint8_t drive, watch; const char *dn, *wn; } pairs[2] = {
        {sda, scl, "SDA", "SCL"}, {scl, sda, "SCL", "SDA"},
    };
    for (auto &p : pairs) {
        pinMode(p.drive, OUTPUT);
        digitalWrite(p.drive, LOW);
        delayMicroseconds(50);
        const int driven = digitalRead(p.drive), other = digitalRead(p.watch);
        pinMode(p.drive, INPUT_PULLUP);
        delayMicroseconds(50);

        // 4. The driven line must actually reach 0. If it cannot, something is
        //    holding it high -- a short to 3V3, or a push-pull driver on the net.
        if (driven != 0) {
            ESP_LOGE(TAG, "%s(%hhu) will not go low -- short to 3V3 or a push-pull "
                          "driver on the net", p.dn, p.drive);
            ok = false;
        }
        if (other == 0) {
            ESP_LOGE(TAG, "%s(%hhu) and %s(%hhu) are SHORTED -- pulling %s low pulls "
                          "%s with it", p.dn, p.drive, p.wn, p.watch, p.dn, p.wn);
            ok = false;
        }
    }

    if (ok) ESP_LOGI(TAG, "pin check ok: SDA=%hhu SCL=%hhu idle high, independent, "
                          "both drivable low (internal pullups)", sda, scl);
    return ok;
}

inline TwoWire &i2c_wire(uint8_t port) {
#if defined(ESP32) && !defined(TARGET_STM32H5)
    return port == 0 ? Wire : Wire1;
#else
    (void) port;
    return Wire;
#endif
}

class I2cLock {
    static SemaphoreHandle_t mutex(uint8_t port) {
        static SemaphoreHandle_t mutexes[2] = {nullptr, nullptr};
#if defined(ESP32) && !defined(TARGET_STM32H5)
        const uint8_t lockIndex = port == 0 ? 0 : 1;
#else
        const uint8_t lockIndex = 0;
#endif
        SemaphoreHandle_t &m = mutexes[lockIndex];
        if (!m) m = xSemaphoreCreateRecursiveMutex();
        return m;
    }

    SemaphoreHandle_t handle;
    bool held;

public:
    // Recursive, so a helper that locks may call another that locks.
    explicit I2cLock(uint8_t port = 0, TickType_t wait = pdMS_TO_TICKS(2000))
        : handle(mutex(port)), held(xSemaphoreTakeRecursive(handle, wait) == pdTRUE) {
        if (!held) ESP_LOGE("i2c", "bus %hhu lock timeout", port);
    }

    ~I2cLock() { if (held) xSemaphoreGiveRecursive(handle); }

    explicit operator bool() const { return held; }

    I2cLock(const I2cLock &) = delete;
    I2cLock &operator=(const I2cLock &) = delete;
};

/// Per-controller recovery state. Written only under that port's I2cLock.
struct I2cBus {
    int8_t sda = -1, scl = -1; // -1: not started through i2c_begin(), never recovered
    uint32_t freq = 0;
    I2cRecoveryPolicy policy;
    uint32_t errors = 0;          // failed transfers, total
    uint32_t recoveries = 0;      // attempts that ended with idle lines and Wire running
    uint32_t failedRecoveries = 0;
    uint32_t generation = 0;      // bumped by every attempt; samplers re-configure on change
    bool recovering = false;
};

inline I2cBus &i2c_bus(uint8_t port) {
    static I2cBus buses[2];
    return buses[port ? 1 : 0];
}

inline uint32_t i2c_bus_generation(uint8_t port) { return i2c_bus(port).generation; }

/// Upper bound on one Wire transfer. Arduino's default is 50 ms; a 3-byte register read at
/// 100..400 kHz takes well under 1 ms, so this only shortens how long a dead bus can hold
/// the real-time task per attempt.
static constexpr uint16_t I2C_TIMEOUT_MS = 20;

#if I2C_RECOVERY
/// Pin check + Wire.begin + record pins/clock for recovery. Use instead of Wire.begin().
inline bool i2c_begin(uint8_t port, int8_t sda, int8_t scl, uint32_t freq) {
    i2c_check_pins(sda, scl);
    TwoWire &wire = i2c_wire(port);
    const bool ok = wire.begin(sda, scl, freq);
    wire.setTimeOut(I2C_TIMEOUT_MS);
    I2cBus &b = i2c_bus(port);
    b.sda = sda, b.scl = scl, b.freq = freq;
    return ok;
}

/// Clear a stuck bus and restart Wire on it. Task context only; takes the bus lock
/// (recursive, so callers already inside a transfer helper are fine). Returns true when
/// both lines end idle high and Wire is running again.
inline bool i2c_recover_bus(uint8_t port, const char *trigger) {
    const char *TAG = "i2c";
    I2cBus &b = i2c_bus(port);
    if (b.sda < 0 || b.scl < 0 || b.recovering || xPortInIsrContext()) return false;
    I2cLock lock{port};
    if (!lock) return false;
    b.recovering = true;
    const uint32_t streak = b.policy.streak;
    const int64_t spanMs = streak ? (esp_timer_get_time() - b.policy.firstFailUs) / 1000 : 0;
    TwoWire &wire = i2c_wire(port);

    // The IDF driver refuses to remove its devices -- so Wire.end() cannot delete the bus --
    // while its status says a transfer is in flight. This resets that status (and runs the
    // controller's own clear-bus) so the teardown below can succeed. If it fails, leave the
    // driver alone: a half-done Wire.end() frees Wire's buffers but keeps the port "started",
    // and Wire.begin() then never reallocates them.
    bool resetOk = true;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
    if (auto h = (i2c_master_bus_handle_t) i2cBusHandle(port)) resetOk = i2c_master_bus_reset(h) == ESP_OK;
#endif
    if (resetOk) wire.end();
    const bool released = resetOk && !i2cIsInit(port);

    bool sdaStuck = false, sclStuck = false, idle = false;
    int pulses = 0;
    if (released) {
        constexpr uint32_t HALF_US = 5; // 100 kHz
        const uint8_t sda = b.sda, scl = b.scl;
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, OUTPUT_OPEN_DRAIN | PULLUP);
        digitalWrite(scl, HIGH);
        delayMicroseconds(50);
        sclStuck = !digitalRead(scl); // short, or a part stretching the clock forever
        sdaStuck = !digitalRead(sda);
        // A slave mid-byte holds SDA low until it has clocked out its bits; release it.
        while (!digitalRead(sda) && pulses < 9) {
            digitalWrite(scl, LOW);
            delayMicroseconds(HALF_US);
            digitalWrite(scl, HIGH);
            delayMicroseconds(HALF_US);
            ++pulses;
        }
        // STOP: SDA low -> high while SCL is high.
        digitalWrite(scl, LOW);
        pinMode(sda, OUTPUT_OPEN_DRAIN | PULLUP);
        digitalWrite(sda, LOW);
        delayMicroseconds(HALF_US);
        digitalWrite(scl, HIGH);
        delayMicroseconds(HALF_US);
        digitalWrite(sda, HIGH);
        delayMicroseconds(HALF_US);
        idle = digitalRead(sda) && digitalRead(scl);
    }

    const bool running = released && wire.begin(b.sda, b.scl, b.freq);
    wire.setTimeOut(I2C_TIMEOUT_MS);
    const bool ok = running && idle;
    ++b.generation;
    ++(ok ? b.recoveries : b.failedRecoveries);
    b.policy.onRecoveryAttempt(esp_timer_get_time());
    b.recovering = false;

    ESP_LOGW(TAG, "bus %hhu recovery (%s, %lu errors in %lld ms): SDA %s, SCL %s, %d pulses -> %s",
             port, trigger, (unsigned long) streak, (long long) spanMs,
             released ? (sdaStuck ? "stuck low" : "high") : "?",
             released ? (sclStuck ? "stuck low" : "high") : "?", pulses,
             ok ? "ok" : !released ? "FAILED, driver would not release the bus"
                       : !idle ? "FAILED, lines still low" : "FAILED, Wire re-init");
    return ok;
}
#endif

/// Report a transfer outcome. Call while holding the bus lock.
inline void i2c_note(uint8_t port, bool ok) {
#if I2C_RECOVERY
    I2cBus &b = i2c_bus(port);
    if (ok) {
        b.policy.onSuccess();
        return;
    }
    ++b.errors;
    if (b.policy.onFailure(esp_timer_get_time())) i2c_recover_bus(port, "error streak");
#else
    (void) port, (void) ok;
#endif
}

/// Write `command` then `len` bytes on the selected hardware controller.
inline esp_err_t i2c_write_buf(uint8_t port, uint8_t address, uint8_t command, const uint8_t *data, uint8_t len) {
    I2cLock lock{port};
    if (!lock) return ESP_ERR_TIMEOUT;

    TwoWire &wire = i2c_wire(port);
    wire.beginTransmission(address);
    if (wire.write(command) != 1) {
        wire.endTransmission();
        return ESP_FAIL;
    }
    for (uint8_t i = 0; i < len; ++i) {
        if (wire.write(data[i]) != 1) {
            wire.endTransmission();
            return ESP_FAIL;
        }
    }
    const bool ok = wire.endTransmission() == 0;
    i2c_note(port, ok);
    return ok ? ESP_OK : ESP_FAIL;
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
///
/// A STOP-separated fallback (pointer write closed with a STOP, then a separate
/// read) lived here briefly and was REMOVED after measurement. Recorded because the
/// symptom is misleading enough to invite the same fix again:
///
/// During TMP117 bring-up every combined transfer failed immediately -- ~1 ms, so
/// not a timeout -- with ESP_ERR_INVALID_STATE, for every address, while
/// i2c_master_probe() ACKed two of the parts and plain writes also failed. That
/// looks like a driver quirk, and lowering the clock 400k -> 100k -> 20k changed
/// nothing, which seems to rule out the wiring too. It was in fact a MISSING
/// GROUND: the parts were phantom-powered through their ESD diodes off the
/// pull-ups, with enough current to ACK an address but not to carry a data byte.
/// i2c_check_pins() above cannot see that, and says so.
///
/// With the ground connected, repeated start works. Instrumented to log whenever
/// the fallback succeeded where repeated start had failed, it fired ZERO times
/// across hundreds of reads from three TMP117s, and was entered only for the
/// address with no part on it. So the fallback bought nothing measurable and cost a
/// second doomed transaction on every read from an absent or failing device --
/// exactly the bus load the header of this file warns about, and on the INA228
/// boards that would sit on the continuously-sampled path.
///
/// If this failure is ever seen again: check the ground before changing this code.
inline esp_err_t i2c_read_buf(uint8_t port, uint8_t address, uint8_t command, uint8_t *buffer, uint8_t len) {
    I2cLock lock{port};
    if (!lock) return ESP_ERR_TIMEOUT;

    TwoWire &wire = i2c_wire(port);
    wire.beginTransmission(address);
    if (wire.write(command) != 1) {
        wire.endTransmission();
        return ESP_FAIL;
    }
    const bool ok = wire.endTransmission(false) == 0 // repeated start, no stop
                    && wire.requestFrom(address, len) == len;
    i2c_note(port, ok);
    if (!ok) return ESP_FAIL;
    for (uint8_t i = 0; i < len; ++i) buffer[i] = wire.read();
    return ESP_OK;
}

/// Big-endian 16-bit register. Returns 0 on failure -- callers only use this for
/// ID registers, where 0 is already treated as "not present" (see INA228::init).
inline uint16_t i2c_read_short(uint8_t port, uint8_t address, uint8_t command) {
    uint8_t buf[2] = {0, 0};
    if (i2c_read_buf(port, address, command, buf, 2) != ESP_OK) return 0;
    return (uint16_t) ((buf[0] << 8) | buf[1]);
}

/// Address probe. Deliberately not reported to i2c_note(): a NACK is its normal answer.
inline bool i2c_test_address(uint8_t addr, uint8_t port = 0) {
    I2cLock lock{port};
    if (!lock) return false;
    TwoWire &wire = i2c_wire(port);
    wire.beginTransmission(addr);
    return wire.endTransmission() == 0;
}
