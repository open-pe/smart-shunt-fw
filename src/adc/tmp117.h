#pragma once


#include <Arduino.h>

#include "sampling.h"


/// TMP117 driver, direct on the shared Wire bus (src/i2c.h).
///
/// This replaced NilsMinor/TMP117-Arduino, which was unusable here for three
/// independent reasons -- all of them silent:
///
///  1. Its CONSTRUCTOR called Wire.begin() with no arguments.  The sampler is a
///     global, so that ran during static init, before setup(), and latched the bus
///     onto the variant default pins (S3: SDA=8/SCL=9) at 100 kHz.  In
///     arduino-esp32 3.x a second TwoWire::begin() on an already-initialised port
///     logs one warning and returns TRUE without changing anything, so main()'s
///     Wire.begin(settings.Pin_I2C_SDA, settings.Pin_I2C_SCL, 400000) became a
///     no-op and EVERY device on the bus (the INA228s included) went dark.
///     Nothing here may call Wire.begin(); main() owns the bus.
///
///  2. Its register read checked no return code at all -- not endTransmission(),
///     not requestFrom().  On a failed read Wire.available() is 0, which satisfied
///     its `if (available() <= 2)`, so it read the empty buffer twice, got -1/-1,
///     and returned 0xFFFF -> int16 -1 -> -0.0078 degC.  A sensor that was not
///     responding at all reported very nearly zero, forever.  Absence of evidence
///     must not read as a plausible measurement: every failure here yields NAN,
///     which is what Sample::temp already defaults to.
///
///  3. Its writeConfig() bracketed every config write in unlockEEPROM()/
///     lockEEPROM().  With the EEPROM unlocked the TMP117 PROGRAMS config-register
///     writes into EEPROM, so init() burned four EEPROM cycles per boot (and left
///     the part busy ~7 ms after each, ignoring traffic).  Config is written here
///     with the EEPROM left locked, i.e. volatile only.
///
/// Data-ready is taken from the config register's DRDY bit rather than a timer.
/// The old timer was wrong anyway: CONV=125ms with AVG=8 is a 125 ms conversion
/// cycle (datasheet Table "Conversion Cycle Time in CC Mode"), not the 125*8 ms
/// the wrapper waited for.
///
/// That DRDY poll is itself an I2C transaction, so it is rate-limited the same way
/// PowerSampler_INA228::hasData() is: realTimeTask() spins `for (ec : counters)
/// ec.update()` with no delay, so an ungated poll would put one transaction per
/// loop iteration on the bus for the ~120 ms of every conversion cycle where DRDY
/// is still clear -- and two per iteration forever if the part stops answering,
/// which would throttle the INA228 channels that share the loop and the bus.
///
/// This sampler never sets u/i/p, so its EnergyCounter's mean power is NaN forever;
/// measuresPower() returns false to keep it out of the idle-sleep vote, which reads
/// a non-finite mean as "cannot judge -> stay awake" and would otherwise pin the
/// whole board awake for good.
class PowerSampler_TMP117 : public PowerSampler {
public:
    // Registers.
    static constexpr uint8_t REG_TEMP_RESULT = 0x00;
    static constexpr uint8_t REG_CONFIGURATION = 0x01;
    static constexpr uint8_t REG_DEVICE_ID = 0x0F;

    static constexpr uint16_t DEVICE_ID = 0x0117;
    static constexpr float RESOLUTION = 7.8125e-3f; // degC per LSB

    // MOD=00 (continuous), CONV=001 (125 ms), AVG=01 (8 samples), alert pin unused.
    static constexpr uint16_t CONFIG = (0u << 10) | (1u << 7) | (1u << 5);
    // Bits 15..12 are read-only status (HIGH/LOW alert, DRDY, EEPROM busy).
    static constexpr uint16_t CONFIG_RW_MASK = 0x0FFF;

    static constexpr uint16_t CONFIG_DRDY = 1u << 13;
    static constexpr uint16_t CONFIG_SOFT_RESET = 1u << 1;

    // Datasheet operating range is -55..150 degC; anything outside is a bad read
    // (0x8000, the reset/invalid pattern, lands at -256).
    static constexpr float TEMP_MIN = -60.f, TEMP_MAX = 160.f;

    // Conversion cycle is 125 ms, so this bounds DRDY latency to +25 ms while
    // keeping the poll off the bus for the rest of the cycle.
    static constexpr unsigned long POLL_INTERVAL_US = 25000;

    explicit PowerSampler_TMP117(uint8_t addr) : address(addr) {
    }

    bool init() override;

    void startReading() override {
    }

    bool hasData() override;

    Sample getSample() override;

    uint8_t getStorageId() const override {
        return 5;
    }

    /// Temperature only -- u/i/p are never set, so this counter's mean power is NaN
    /// by construction and must not vote on idle-sleep.  See PowerSampler.
    bool measuresPower() const override { return false; }

private:
    /// NAN on any I2C failure or out-of-range value.
    float readTemperature();

    uint8_t address;
    bool dataReady{false};   ///< sticky: reading CONFIG clears DRDY in the chip
    bool pollFailed{false};  ///< sticky: DRDY poll failed, getSample() owes a NAN
    unsigned long tLastPoll{0};
    uint32_t readErrors{0};  ///< consecutive failures, for rate-limited logging
};


class PowerSampler_Dummy : public PowerSampler {
    unsigned long tLastRead = 0;

public:
    PowerSampler_Dummy() {
    }

    bool init() override {
        //tStart = micros();
        return true;
    };

    void startReading() override {
    }

    bool hasData() override {
        return micros() - tLastRead > (125 * 8 * 1000); // ct=125ms, avg=8
    }

    Sample getSample() override {
        Sample sample{};
        sample.u = sinf(static_cast<float>(micros()) * .33e-6f);
        sample.setTimeNow();
        tLastRead = micros();
        return sample;
    }

    uint8_t getStorageId() const override {
        return 5;
    }
};
