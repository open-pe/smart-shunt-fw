#pragma once

#include <cmath>
#ifdef TARGET_STM32H5
#include "esp_compat.h"
#endif
#include "i2c.h"
#include "sampling.h"
#include "settings.h"
#include "util.h"

// INA228 register addresses (subset needed for mux operation).
// Duplicated from ina228.h to avoid including the full header, which defines
// non-inline free functions and a global ina228_instance[] array that would
// cause multiple-definition errors if included from another translation unit.
#define MUX_INA228_CONFIG           0x00
#define MUX_INA228_ADC_CONFIG       0x01
#define MUX_INA228_SHUNT_CAL        0x02
#define MUX_INA228_VBUS             0x05
#define MUX_INA228_DIETEMP          0x06
#define MUX_INA228_DIAG_ALRT        0x0B
#define MUX_INA228_MANUFACTURER_ID  0x3E
#define MUX_INA228_DEVICE_ID        0x3F

// Single-shot MODE values (bits [15:12] of ADC_CONFIG).
// Bit 0 = bus voltage, bit 1 = shunt voltage, bit 2 = temperature, bit 3 = continuous.
constexpr auto MUX_MODE_SingleShot_BusV      = 0x1;
constexpr auto MUX_MODE_SingleShot_BusV_Temp = 0x5;

constexpr auto MUX_CT_84u   = 0x1;
constexpr auto MUX_CT_4120u = 0x7;
constexpr auto MUX_AVG_16   = 0x2;
constexpr auto MUX_BitPos_MODE = 12;

class INA228MuxBackend;

INA228MuxBackend *ina228_mux_instance = nullptr;
void IRAM_ATTR ina228_mux_alert();

class INA228MuxBackend {
public:
    enum class Target : uint8_t { CH_A, CH_B, ZERO };

    struct MuxSample {
        float u;
        float temp;
        uint32_t diag;
    };

private:
    uint8_t i2c_addr;
    uint8_t alert_pin;
    uint8_t mux_pin_a, mux_pin_b, mux_pin_zero;

    volatile bool new_data = false;
    bool initialized = false;
    TaskNotification notification;

    friend class PowerSampler_MuxChannel;

    enum class State : uint8_t { IDLE, WAITING, READY };
    State state = State::IDLE;
    Target currentTarget = Target::CH_A;
    Target nextTarget = Target::CH_A;

    uint16_t adcConfig = 0;
    float zeroOffset = 0.0f;
    uint16_t zeroCalCounter = 0;
    // Zero-offset calibration interval in alternations. 0 disables.
    uint16_t zeroCalInterval = 100;

    float lastVoltage[2] = {NAN, NAN}; // per-channel (CH_A, CH_B) for jump detection
    uint32_t implausibleCount = 0;
    int64_t lastImplausibleLog = 0;
    uint32_t lastDiag = 0;

    unsigned long tLastDataCheck = 0;
    bool inOverflow = false;
    bool shuntLv = false;
    float current_LSB = 0;

public:
    INA228MuxBackend(uint8_t addr, uint8_t alert, uint8_t pin_a, uint8_t pin_b, uint8_t pin_zero)
        : i2c_addr(addr), alert_pin(alert), mux_pin_a(pin_a), mux_pin_b(pin_b), mux_pin_zero(pin_zero) {}

    bool init() {
        ESP_LOGI("ina228_mux", "Manufacturer ID: 0x%04X",
                 i2c_read_short(0, i2c_addr, MUX_INA228_MANUFACTURER_ID));
        auto deviceId = i2c_read_short(0, i2c_addr, MUX_INA228_DEVICE_ID);
        ESP_LOGI("ina228_mux", "Device ID:       0x%04X", deviceId);
        if (deviceId != 0x2280 && deviceId != 0x2281) {
            if (deviceId != 0) ESP_LOGW("ina228_mux", "This is not an INA228 device!");
            return false;
        }

        if (i2c_write_short(0, i2c_addr, MUX_INA228_CONFIG, 0x8000) != ESP_OK) return false;

        // EEPROM slot 9 = 8 + (0x41 - 0x40). Shared by both mux channels (one physical chip).
        float resistor = 2e-3f, range = 38.0f;
        if (readCalibrationFactors(9, resistor, range)) {
            ESP_LOGI("ina228_mux", "Restore resistor/range: %.6f/%.6f", resistor, range);
        } else {
            ESP_LOGI("ina228_mux", "Default resistor/range: %.6f/%.6f", resistor, range);
        }
        setResistorRange(resistor, range, false);

        // DIAG_ALRT: enable CNVR on ALERT pin. Written once; not re-written per trigger.
        uint16_t diagAlrt = 0;
        diagAlrt |= 0x1 << 14;
        if (i2c_write_short(0, i2c_addr, MUX_INA228_DIAG_ALRT, diagAlrt) != ESP_OK) {
            ESP_LOGE("ina228_mux", "DIAG_ALRT write failed");
            return false;
        }

        // ADC_CONFIG for single-shot bus+temp. Writing this register triggers a conversion,
        // so we don't write it here — trigger() writes it when a conversion is needed.
        adcConfig = 0;
        adcConfig |= MUX_MODE_SingleShot_BusV_Temp << MUX_BitPos_MODE;
        adcConfig |= MUX_CT_4120u << 9;  // VBUSCT = 4120us
        adcConfig |= MUX_CT_84u << 3;    // DIETICT = 84us
        adcConfig |= MUX_AVG_16 << 0;    // AVG = 16

        pinMode(alert_pin, INPUT_PULLUP);
        ina228_mux_instance = this;
        attachInterrupt(digitalPinToInterrupt(alert_pin), ina228_mux_alert, FALLING);

        pinMode(mux_pin_a, OUTPUT);
        pinMode(mux_pin_b, OUTPUT);
        pinMode(mux_pin_zero, OUTPUT);
        digitalWrite(mux_pin_a, LOW);
        digitalWrite(mux_pin_b, LOW);
        digitalWrite(mux_pin_zero, LOW);

        ESP_LOGI("ina228_mux", "initialized: addr=0x%02hhX alert=%d mux=%d/%d/%d zeroCalInterval=%hu",
                 i2c_addr, alert_pin, mux_pin_a, mux_pin_b, mux_pin_zero, zeroCalInterval);
        initialized = true;
        return true;
    }

    void setResistorRange(float resistor, float range, bool store = true) {
        float maxExpectedVoltage = resistor * range;
        assert(maxExpectedVoltage > 1e-3f);
        assert(maxExpectedVoltage < 163.84e-3f);

        shuntLv = (maxExpectedVoltage < 40.96e-3f);
        uint16_t config = 0;
        config |= (shuntLv ? 0x1 : 0x0) << 4;
        i2c_write_short(0, i2c_addr, MUX_INA228_CONFIG, config);

        current_LSB = range / std::pow(2.f, 19.f);
        auto shuntCal = 13107.2e6f * current_LSB * resistor;
        if (shuntLv) shuntCal *= 4;
        auto shuntCalShort = (uint16_t) std::lround(shuntCal);
        current_LSB = (float) shuntCalShort / 13107.2e6f / resistor;

        ESP_LOGI("ina228_mux", "Set shuntCal %hu, Vmax_exp=%.1fmV", shuntCalShort, maxExpectedVoltage * 1e3f);
        i2c_write_short(0, i2c_addr, MUX_INA228_SHUNT_CAL, shuntCalShort);

        if (store) storeCalibrationFactors(9, resistor, range);
    }

    void alertNewDataFromISR() {
        new_data = true;
        notification.notifyFromIsr();
    }

    /// Idempotent stall recovery. Resets to IDLE, re-arms DIAG_ALRT.
    /// Safe to call from either MuxChannel's EnergyCounter.
    void startReading() {
        state = State::IDLE;
        new_data = false;
        uint16_t diagAlrt = 0;
        diagAlrt |= 0x1 << 14;
        i2c_write_short(0, i2c_addr, MUX_INA228_DIAG_ALRT, diagAlrt);
    }

    bool hasDataFor(Target ch) {
        // If a ZERO conversion just completed, consume it and trigger the next.
        // Only checks the volatile flag — no I2C, no notification wait.
        if (state == State::WAITING && currentTarget == Target::ZERO && new_data) {
            consumeZeroReading();
        }

        if (state == State::IDLE) {
            setMux(nextTarget);
            trigger();
            currentTarget = nextTarget;
            state = State::WAITING;
            new_data = false;
            tLastDataCheck = micros();
            return false;
        }

        if (state == State::WAITING) {
            // Non-target channel: return immediately. No I2C, no notification, no flag touch.
            if (ch != currentTarget) return false;

            if (!new_data) {
                notification.subscribe();
                if (!notification.wait(1) || !new_data) {
                    if (micros() - tLastDataCheck < 50000)
                        return false;
                }
            }

            tLastDataCheck = micros();
            new_data = false;

            uint16_t diagAlrt = i2c_read_short(0, i2c_addr, MUX_INA228_DIAG_ALRT);
            bool CNVRF = (diagAlrt >> 1) & 0x1;
            bool BUSOL = (diagAlrt >> 4) & 0x1;
            bool MATHOF = (diagAlrt >> 9) & 0x1;

            if (BUSOL) ESP_LOGW("ina228_mux", "Bus-Voltage over-load!");
            if (MATHOF) {
                if (!inOverflow) ESP_LOGW("ina228_mux", "Math over-flow!");
                inOverflow = true;
            } else if (inOverflow) {
                ESP_LOGW("ina228_mux", "Math over-flow resolved!");
                inOverflow = false;
            }

            if (CNVRF) {
                state = State::READY;
                return true;
            }
            return false;
        }

        // READY
        if (ch != currentTarget) return false;
        return true;
    }

    MuxSample getSample(Target ch) {
        MuxSample ms{};
        ms.u = read_voltage_raw(currentTarget) - zeroOffset;
        ms.temp = read_dietemp();
        ms.diag = lastDiag;
        lastDiag = 0;

        advanceTarget();
        state = State::IDLE;
        new_data = false;
        return ms;
    }

    float read_dietemp() {
        uint8_t buf[2];
        if (i2c_read_buf(0, i2c_addr, MUX_INA228_DIETEMP, buf, 2) != ESP_OK) {
            ESP_LOGE("ina228_mux", "err reading dietemp");
            return NAN;
        }
        int16_t regVal = (int16_t)((buf[0] << 8) | buf[1]);
        return regVal * 7.8125e-3f;
    }

private:
    void setMux(Target t) {
        // All switches OFF first (break-before-make for independent SPST switches).
        digitalWrite(mux_pin_a, LOW);
        digitalWrite(mux_pin_b, LOW);
        digitalWrite(mux_pin_zero, LOW);
        delayMicroseconds(1); // >50ns turn-off time
        switch (t) {
            case Target::CH_A:  digitalWrite(mux_pin_a, HIGH); break;
            case Target::CH_B:  digitalWrite(mux_pin_b, HIGH); break;
            case Target::ZERO:  digitalWrite(mux_pin_zero, HIGH); break;
        }
        delayMicroseconds(100); // charge injection settling
    }

    void trigger() {
        // Write only ADC_CONFIG (not DIAG_ALRT) to start one single-shot conversion.
        if (i2c_write_short(0, i2c_addr, MUX_INA228_ADC_CONFIG, adcConfig) != ESP_OK) {
            ESP_LOGE("ina228_mux", "ADC_CONFIG write failed");
        }
    }

    void consumeZeroReading() {
        uint16_t diagAlrt = i2c_read_short(0, i2c_addr, MUX_INA228_DIAG_ALRT);
        bool CNVRF = (diagAlrt >> 1) & 0x1;
        if (!CNVRF) return;

        float v = read_voltage_raw(Target::ZERO);
        zeroOffset = v;
        new_data = false;
        zeroCalCounter = 0;
        nextTarget = Target::CH_A;
        state = State::IDLE;
        ESP_LOGI("ina228_mux", "Zero-offset calibrated: %.6f V", zeroOffset);
    }

    void advanceTarget() {
        if (nextTarget == Target::CH_A) {
            nextTarget = Target::CH_B;
        } else if (nextTarget == Target::CH_B) {
            if (zeroCalInterval > 0 && ++zeroCalCounter >= zeroCalInterval) {
                nextTarget = Target::ZERO;
            } else {
                nextTarget = Target::CH_A;
            }
        } else {
            nextTarget = Target::CH_A;
            zeroCalCounter = 0;
        }
    }

    float read_voltage_raw(Target t) {
        uint8_t buf[3];
        if (i2c_read_buf(0, i2c_addr, MUX_INA228_VBUS, buf, 3) != ESP_OK) {
            ESP_LOGE("ina228_mux", "0x%02hhX: err reading voltage", i2c_addr);
            return NAN;
        }

        // INA228 VBUS: 24-bit register, 20-bit ADC left-justified (bits 23:4).
        // Bytes are MSB-first: buf[0]=bits23:16, buf[1]=bits15:8, buf[2]=bits7:0.
        uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
        bool sign = raw & 0x800000;
        uint32_t adcCode = (raw >> 4) & 0xFFFFF;
        int32_t code = (int32_t)adcCode;
        if (sign) code += 0xFFF00000;
        float fBusVoltage = (float)code * 0.0001953125f;

        // Jump detection: compare against the same channel's previous reading.
        // ZERO readings skip the jump check (offset is always ~0V).
        const char *reason = nullptr;
        uint8_t reasonCode = 0;
        if (fBusVoltage > 85.0f || fBusVoltage < -1.0f) {
            reason = "out-of-range";
            reasonCode = 1;
        } else if (t != Target::ZERO) {
            uint8_t vi = (t == Target::CH_A) ? 0 : 1;
            if (!std::isnan(lastVoltage[vi]) && std::fabs(fBusVoltage - lastVoltage[vi]) > 10.0f) {
                reason = "jump";
                reasonCode = 2;
            }
            lastVoltage[vi] = fBusVoltage;
        }
        if (reason) {
            lastDiag = (reasonCode << 24) | ((uint32_t)sign << 20) | (adcCode & 0xFFFFF);
            int64_t now = esp_timer_get_time();
            if (now - lastImplausibleLog > 1000000) {
                lastImplausibleLog = now;
                ++implausibleCount;
                ESP_LOGW("ina228_mux", "VBUS %s: %.3fV raw=[%02hhX %02hhX %02hhX] code=0x%05X sign=%d count=%lu",
                         reason, fBusVoltage, buf[0], buf[1], buf[2],
                         adcCode, (int)sign, implausibleCount);
            }
        }
        return fBusVoltage;
    }
};

void IRAM_ATTR ina228_mux_alert() {
    if (ina228_mux_instance) ina228_mux_instance->alertNewDataFromISR();
}


class PowerSampler_MuxChannel : public PowerSampler {
    INA228MuxBackend &backend;
    INA228MuxBackend::Target channel;
    uint8_t storageId_;
    Sample lastSample{};

public:
    PowerSampler_MuxChannel(INA228MuxBackend &b, INA228MuxBackend::Target ch, uint8_t sid)
        : backend(b), channel(ch), storageId_(sid) {}

    bool init() override {
        return backend.initialized;
    }

    void startReading() override {
        backend.startReading();
    }

    bool hasData() override {
        return backend.hasDataFor(channel);
    }

    Sample getSample() override {
        auto ms = backend.getSample(channel);
        lastSample.setTimeNow();
        lastSample.u = ms.u;
        lastSample.i = NAN;
        lastSample.temp = ms.temp;
        lastSample.diag = ms.diag;
        return lastSample;
    }

    uint8_t getStorageId() const override { return storageId_; }
};
