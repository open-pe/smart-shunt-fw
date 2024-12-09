#include <SPI.h> // not sure why this is needed

#include <INA226_WE.h>

#include "settings.h"
#include "sampling.h"
#include "ina228.h"

class PowerSampler_INA226;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void ina226_alert();

PowerSampler_INA226 *ina226_instance = nullptr;


class PowerSampler_INA226 : public PowerSampler {

    INA226_WE ina226;
    volatile bool new_data = false;
    Sample lastSample{};


public:
    const uint8_t storageId = 1;

    uint8_t getStorageId() const override { return storageId; };

    bool init() override {
        if (ina226_instance) {
            return false;
        }

        auto addr = INA226_WE::INA226_ADDRESS;

        if(!i2c_test_address(addr))
            return false;

        auto mfrId = i2c_read_short(0, addr, INA228_MANUFACTURER_ID);
        auto deviceId = i2c_read_short(0, addr, INA228_DEVICE_ID);

        ESP_LOGI("ina228", "MfrID: 0x%04X, DeviceID: 0x%04X", mfrId, deviceId);

        if (deviceId != 0x2260) {
            ESP_LOGW("ina226", "This is not an INA226 device!");
            return false;
        }

        if (!ina226.init())
            return false;

        ina226.setAverage(AVERAGE_4);
        //ina226.setConversionTime(CONV_TIME_204);
        //ina226.setConversionTime(CONV_TIME_588, CONV_TIME_588);
        ina226.setConversionTime(CONV_TIME_588);

        // analog filters:
        // use RC with cutoff at sampling rate
        // with conversion time 588, sampling rate is about 1700 Hz.
        // 2x10ohm and 4,7uF. => 1693 cutoff

        //ina226.setAverage(AVERAGE_16);
        //ina226.setConversionTime(CONV_TIME_1100, CONV_TIME_204);

        //ina226.setAverage(AVERAGE_1024);
        //ina226.setConversionTime(CONV_TIME_8244);

        // ^^^ Conversion ready after conversion time x number of averages x 2


        //ina226.setResistorRange(50e-3f / 100, 20.0f);
        //ina226.setResistorRange(75e-3f / 20, 20.0f); // 20 A shunt
        // ina226.setResistorRange(75e-3f / 40, 40.0f); // radiomag 40 A shunt

        float resistor = 1e-3f, range = 80.0f;// default: 1mOhm, 80A (ina226 shunt voltage range is 81.92mV)
        if (readCalibrationFactors(4, resistor, range)) {
            ESP_LOGI("ina226", "Restore resistor/range settings: %.6f/%.6f", resistor, range);
        } else {
            ESP_LOGI("ina226", "Default resistor/range settings: %.6f/%.6f", resistor, range);
        }
        delay(10);

        ina226.setResistorRange(resistor, range);

        ina226.enableConvReadyAlert();

        ina226_instance = this;

        uint8_t READY_PIN = settings.Pin_INA22x_ALERT;
        pinMode(READY_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(READY_PIN), ina226_alert, FALLING);
        ESP_LOGI("ina226", "Setup ALERT interrupt pin %hhu", READY_PIN);

        return true;
    }

    void setResistorRange(float res, float range) {
        ina226.setResistorRange(res, range);
        storeCalibrationFactors(4, res, range);
    }

    void startReading() override {
        ina226.setMeasureMode(CONTINUOUS);
    }

    void alertNewDataFromISR() {
        new_data = true;
    }

    bool hasData() override{
        //return true;

        if (!new_data)
            return false;

        ina226.readAndClearFlags();
        if (ina226.overflow) {
            ESP_LOGW("ina226", "Overflow!");
        }

        if (ina226.limitAlert) {
            ESP_LOGW("ina226", "Limit Alert!");
        }
        return ina226.convAlert;
    }

    Sample getSample() override {

        lastSample.setTimeNow();

        // float shuntCorr = 0.99947179931f; // 40A radiomag
        // float shuntCorr = 1.000867f; // resistor today 0.5mOhm

        lastSample.i = ina226.getCurrent_A(); //
        //lastSample.u = ina226.getBusVoltage_V() * (44.604f / 18.321f * 1.0137095f / 26217.f * 26269.f);
        lastSample.u = ina226.getBusVoltage_V();

        float busP = ina226.getBusPower();
        float pErr = std::abs(lastSample.p() - busP) / busP;
        if (pErr > 0.001f) {
            //ESP_LOGW("ina226", "Bus power %.3fW deviates from computed %.3fW ", busP, lastSample.p());
        }

        return lastSample;
    }
};

void IRAM_ATTR ina226_alert() {
    if (ina226_instance)
        ina226_instance->alertNewDataFromISR();
}
