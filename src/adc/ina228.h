#pragma once

#include <SPI.h> // not sure why this is needed

#include <INA226_WE.h>
#include "i2c.h"

#include "settings.h"
#include "sampling.h"
#include "util.h"


class PowerSampler_INA228;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void IRAM_ATTR ina228_alert0();

void IRAM_ATTR ina228_alert1();

void IRAM_ATTR ina228_alert2();

PowerSampler_INA228 *ina228_instance[3] = {nullptr, nullptr, nullptr};


//#define INA228_SLAVE_ADDRESS        0x40

#define INA228_CONFIG            0x00
#define INA228_ADC_CONFIG        0x01
#define INA228_SHUNT_CAL        0x02
#define INA228_SHUNT_TEMPCO        0x03
#define INA228_VSHUNT            0x04
#define INA228_VBUS            0x05
#define INA228_DIETEMP            0x06
#define INA228_CURRENT            0x07
#define INA228_POWER            0x08
#define INA228_ENERGY            0x09
#define INA228_CHARGE            0x0A
#define INA228_DIAG_ALRT        0x0B
#define INA228_SOVL            0x0C
#define INA228_SUVL            0x0D
#define INA228_BOVL            0x0E
#define INA228_BUVL            0x0F
#define INA228_TEMP_LIMIT        0x10
#define INA228_PWR_LIMIT        0x11
#define INA228_MANUFACTURER_ID    0x3E
#define INA228_DEVICE_ID        0x3F


class PowerSampler_INA228 : public PowerSampler {
    static constexpr auto I2C_A0 = 0x40;

    volatile bool new_data = false;

    //int readUICycle = 0;
    //bool readingU = false;

    Sample lastSample{};

    uint8_t i2c_port = 0;

    TaskNotification notification;

    float current_LSB{};

public:
    const uint8_t storageId;
    const uint8_t i2c_addr;

    uint8_t getStorageId() const override { return storageId; };

    explicit PowerSampler_INA228(uint8_t i2c_addr) : storageId{(uint8_t) (2 + i2c_addr - I2C_A0)}, i2c_addr(i2c_addr) {}

    bool init() {

        if (ina228_instance[i2c_addr - I2C_A0]) {
            return false;
        }


        ESP_LOGI("ina228", "Manufacturer ID:    0x%04X",
                 i2c_read_short(i2c_port, i2c_addr, INA228_MANUFACTURER_ID));
        auto deviceId = i2c_read_short(i2c_port, i2c_addr, INA228_DEVICE_ID);
        ESP_LOGI("ina228", "Device ID:          0x%04X", deviceId);

        if (deviceId != 0x2280 && deviceId != 0x2281) {
            ESP_LOGW("ina228", "This is not an INA228 device!");
            return false;
        }

        // reset
        if (i2c_write_short(i2c_port, i2c_addr, INA228_CONFIG, 0x8000) != ESP_OK) {
            return false;
        }

        int inaAddrIdx = i2c_addr - I2C_A0;


        float resistor = 2e-3f, range = 38.0f;// default: vishay 2mOhm, .1%, 3W
        if (readCalibrationFactors(8 + inaAddrIdx, resistor, range)) {
            ESP_LOGI("ina228", "Restore resistor/range settings: %.6f/%.6f", resistor, range);
        } else {
            ESP_LOGI("ina228", "Default resistor/range settings: %.6f/%.6f", resistor, range);
        }
        setResistorRange(resistor, range, false);


        std::array<uint8_t, 3> alertPins = {
                settings.Pin_INA22x_ALERT,
                settings.Pin_INA22x_ALERT2,
                settings.Pin_INA22x_ALERT3
        };
        std::array<void (*)(void), 3> alerts = {&ina228_alert0, &ina228_alert1, &ina228_alert2};

        assert(inaAddrIdx < alertPins.size());
        uint8_t readyPin = alertPins[inaAddrIdx];
        pinMode(readyPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(readyPin), alerts[inaAddrIdx], FALLING);


        uint16_t adc_config = 0;
        adc_config |= 0xB << 12; // MODE   = Continuous shunt and bus voltage
        // conversion times see https://www.ti.com/lit/ds/symlink/ina229.pdf#page=22
        // 4h = 540 µs, 0x5: 1052µs,
        adc_config |= 0x5 << 9; // VBUSCT  = 1052 µs (bus voltage conversion time) // << 9 => 1052
        adc_config |= 0x5 << 6; // VSHCT   = 1052 µs (shunt voltage conversion time)
        adc_config |= 0x0 << 0; // AVG     = (0x0:1, 0x1:4)
        // total 1/((1052-6+1052-6) * 1)
        ESP_ERROR_CHECK(i2c_write_short(i2c_port, i2c_addr, INA228_ADC_CONFIG, adc_config));


        uint16_t diagAlrt = 0;
        diagAlrt |= 0x1 << 14; // CNVR: enable conversion ready flag on ALERT pin
        ESP_ERROR_CHECK(i2c_write_short(i2c_port, i2c_addr, INA228_DIAG_ALRT, diagAlrt));

        ina228_instance[i2c_addr - I2C_A0] = this;

        return true;
    }

    bool shuntLv = false;

    void setShuntLowVoltageRange(bool _40_96_mV) {
        uint16_t config = 0;
        auto adc_range = (_40_96_mV ? 0x1 : 0x0);
        config |= adc_range << 4; // ADC shunt voltage range: 0h = ±163.84 mV, 1h = ± 40.96 mV
        ESP_ERROR_CHECK(i2c_write_short(i2c_port, i2c_addr, INA228_CONFIG, config));
        shuntLv = _40_96_mV;
        ESP_LOGI("ina228", "Set shunt %s mV range", shuntLv ? "40.96" : "163.84");
    }


    void setResistorRange(float resistor, float range, bool store = true) {
        // https://www.ti.com/lit/ds/symlink/ina228.pdf#page=31

        float maxExpectedVoltage = resistor * range;

        assert(maxExpectedVoltage > 1e-3f);
        assert(maxExpectedVoltage < 163.84e-3f);

        setShuntLowVoltageRange((maxExpectedVoltage < 40.96e-3f));

        current_LSB = range / std::pow(2.f, 19.f);
        //auto lsbRound =  std::pow(2.f, std::ceil(std::log2(current_LSB))); // round
        //ESP_LOGI("ina228", "round current_LSB %.7f to %.7f", current_LSB, lsbRound);

        //current_LSB = (float) range / (float) (2 << (19 - 1));
        //currentDivider_mA = 0.001/current_LSB;
        // pwrMultiplier_mW = 1000.0*25.0*current_LSB;

        auto shuntCal = 13107.2e6f * current_LSB * resistor;
        if (shuntLv) // adc_range = 1
            shuntCal *= 4;
        // auto shuntCalShort = (uint16_t) std::lround(shuntCal);
        auto shuntCalShort = (uint16_t) std::lround(shuntCal);

        current_LSB = (float) shuntCalShort / 13107.2e6f / resistor;

        ESP_LOGI("ina228", "Set shuntCal %hu (from %f), Vmax_exp=%.1fmV, current_LSB=%.7f", shuntCalShort, shuntCal,
                 maxExpectedVoltage * 1e3f, current_LSB);

        ESP_ERROR_CHECK(i2c_write_short(i2c_port, i2c_addr, INA228_SHUNT_CAL, shuntCalShort));

        int inaAddrIdx = i2c_addr - I2C_A0;
        if (store)
            storeCalibrationFactors(8 + inaAddrIdx, resistor, range);
    }

    void startReading() {
        // pass
    }

    void alertNewDataFromISR() {
        new_data = true;
        notification.notifyFromIsr();
    }

    bool inOverflow = false;

    bool hasData() override {
        if (!new_data) {
            notification.subscribe();
            if (!notification.wait(1) || !new_data)
                return false;
        }

        new_data = false;

        auto diagAlrt = i2c_read_short(i2c_port, i2c_addr, INA228_DIAG_ALRT);
        bool CNVRF = (diagAlrt >> 1) & 0x1;
        bool BUSOL = (diagAlrt >> 4) & 0x1;
        bool SHNTOL = (diagAlrt >> 6) & 0x1;
        bool MATHOF = (diagAlrt >> 9) & 0x1;

        if (BUSOL) {
            ESP_LOGW("ina228", "Bus-Voltage over-load!");
        }
        if (SHNTOL) {
            ESP_LOGW("ina228", "Shunt-Voltage over-load!");
        }
        if (MATHOF) {
            if (!inOverflow)
                ESP_LOGW("ina228", "Math over-flow!");
            inOverflow = true;
        } else if (inOverflow) {
            ESP_LOGW("ina228", "Math over-flow resolved!");
            inOverflow = false;
        }
        return CNVRF;
    }

    float read_voltage() {
        int32_t iBusVoltage;
        float fBusVoltage;
        bool sign;

        i2c_read_buf(i2c_port, i2c_addr, INA228_VBUS, (uint8_t *) &iBusVoltage, 3);
        sign = iBusVoltage & 0x80;
        iBusVoltage = __bswap32(iBusVoltage & 0xFFFFFF) >> 12;
        if (sign) iBusVoltage += 0xFFF00000;
        fBusVoltage = (iBusVoltage) * 0.0001953125;

        return (fBusVoltage);
    }

    float read_current() {
        int32_t iCurrent;
        float fCurrent;
        bool sign;

        i2c_read_buf(i2c_port, i2c_addr, INA228_CURRENT, (uint8_t *) &iCurrent, 3);
        sign = iCurrent & 0x80;
        iCurrent = __bswap32(iCurrent & 0xFFFFFF) >> 12;
        if (sign) iCurrent += 0xFFF00000;
        if (shuntLv) // adc_range = 1?
            iCurrent = iCurrent / 4;

        fCurrent = (float) (iCurrent) * current_LSB;

        return (fCurrent);
    }

    float read_dietemp() {
        int16_t regVal;
        i2c_read_buf(i2c_port, i2c_addr, INA228_DIETEMP, (uint8_t *) &regVal, 2);
        regVal = __bswap16(regVal);
        return regVal * 7.8125e-3f;
    }

    Sample getSample() {

        lastSample.setTimeNow();

        lastSample.u = read_voltage();
        lastSample.i = read_current();
        lastSample.temp = read_dietemp();

        //float busP = ina226.getBusPower();
        //float pErr = std::abs(lastSample.p() - busP) / busP;
        //if (pErr > 0.001f) {
        //ESP_LOGW("ina226", "Bus power %.3fW deviates from computed %.3fW ", busP, lastSample.p());
        //}

        return lastSample;
    }
};

void IRAM_ATTR ina228_alert0() {
    if (ina228_instance[0])ina228_instance[0]->alertNewDataFromISR();
}

void IRAM_ATTR ina228_alert1() {
    if (ina228_instance[1])ina228_instance[1]->alertNewDataFromISR();
}

void IRAM_ATTR ina228_alert2() {
    if (ina228_instance[2])ina228_instance[2]->alertNewDataFromISR();
}

