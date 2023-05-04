#include <SPI.h> // not sure why this is needed

#include <INA226_WE.h>

#include "sampling.h"

class PowerSampler_INA226;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
void IRAM_ATTR ina225_alert();

PowerSampler_INA226 *ina226_instance = nullptr;


class PowerSampler_INA226 : public PowerSampler {

    INA226_WE ina226;

    volatile bool new_data = false;


    int readUICycle = 0;
    bool readingU = false;

    Sample lastSample;


public:
    bool init() {
        if (ina226_instance) {
            return false;
        }

        ina226_instance = this;

        if (!ina226.init())
            return false;

        ina226.setAverage(AVERAGE_1);
        ina226.setConversionTime(CONV_TIME_1100);

        //ina226.setAverage(AVERAGE_1024);
        //ina226.setConversionTime(CONV_TIME_8244);

        // ^^^ Conversion ready after conversion time x number of averages x 2

        ina226.setMeasureMode(CONTINUOUS);

        //ina226.setResistorRange(50e-3f / 100, 20.0f);
        //ina226.setResistorRange(75e-3f / 20, 20.0f); // 20 A shunt
        ina226.setResistorRange(75e-3f / 40, 20.0f); // 20 A shunt


        uint8_t READY_PIN = 19;
        pinMode(READY_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(READY_PIN), ina225_alert, FALLING);

        ina226.enableConvReadyAlert();

        return true;
    }

    void startReading() {
        // pass
    }

    void alertNewDataFromISR() {
        new_data = true;
    }

    bool hasData() {
        if(!new_data)
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

    Sample getSample() {

        lastSample.setTimeNow();

        float shuntCorr = 0.99619f; // 40A radiomag
        // float shuntCorr = 1.000867f; // resistor today 0.5mOhm

        lastSample.i = ina226.getCurrent_A() * shuntCorr; //
        lastSample.u = ina226.getBusVoltage_V() * (44.604f / 18.321f * 31.018f / 31.011f * 1.0006088f);

        float busP = ina226.getBusPower();
        float pErr = std::abs(lastSample.p() - busP) / busP;
        if (pErr > 0.001f) {
            //ESP_LOGW("ina226", "Bus power %.3fW deviates from computed %.3fW ", busP, lastSample.p());
        }

        return lastSample;
    }
};

void IRAM_ATTR ina225_alert() {
    if (ina226_instance)
        ina226_instance->alertNewDataFromISR();
}
