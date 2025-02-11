#include <SPI.h>
#include <ADS1220_WE.h>

#include "sampling.h"
#include "util.h"
//#include "../../.pio/libdeps/adafruit_feather_esp32s3/ADS1220_WE/src/ADS1220_WE.h"


void IRAM_ATTR ads1220_alert();

class PowerSampler_ADS1220;

PowerSampler_ADS1220 *ads1220_instance = nullptr;


class PowerSampler_ADS1220 : public PowerSampler {

    ADS1220_WE ads{settings.Pin_ADS1220_CS, settings.Pin_ADS1220_DRDY};

    TaskNotification notification;
    volatile bool new_data = false;


    //int readUICycle = 0;
    //bool readingU = false;

    Sample lastSample{};

    //const std::array<adsGain_t, 6> gains = {GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN};
    //adsGain_t gainI = GAIN_EIGHT, gainU = GAIN_TWO;
    //const std::array<int16_t, 6> adcThres = {0u, 17476u, 8738u, 4370u, 2185u, 1093u};

public:
    const uint8_t storageId = 0;

    uint8_t getStorageId() const override { return storageId; };

    bool init() {
        if (ads1220_instance)
            return false;
        ads1220_instance = this;

        SPI.begin(13, 12, 11, settings.Pin_ADS1220_CS);

        if (!ads.init())
            return false;

        // listen to the ADC's ALERT pin
        pinMode(settings.Pin_ADS1220_DRDY, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(settings.Pin_ADS1220_DRDY), ads1220_alert, FALLING);

        ESP_LOGI("ads1220", "Interrupt attached to pin %d", settings.Pin_ADS1220_DRDY);

        ads.setCompareChannels(ADS1220_MUX_2_AVSS); // ch3 singled-ended to GND
        ads.setGain(ADS1220_GAIN_1);
        ads.bypassPGA(true);
        ads.setDataRate(ADS1220_DR_LVL_2); // todo  ADS1220_DR_LVL_5
        ads.setOperatingMode(ADS1220_NORMAL_MODE); // todo duty cycle, turbo
        ads.setConversionMode(ADS1220_CONTINUOUS);
        ads.setDrdyMode(ADS1220_DRDY);// ALERT TODO ADS1220_DOUT_DRDY

        ads.start();

        return true;
    }

    void startReading() {
        // TODO read register
    }


    bool hasData() {
        if (!new_data) {
            notification.subscribe();
            if (!notification.wait(1) || !new_data) return false;
        }

        new_data = false;
        auto v = ads.getVoltage_mV();
        lastSample.u = v * (0.001f * 205.5f/5.f);
        lastSample.setTimeNow();
        return true; // !readingU
    }

    Sample getSample() {
        return lastSample;
    }



    void alertNewDataFromISR() {
        new_data = true;
        notification.notifyFromIsr();
    }
};


void IRAM_ATTR ads1220_alert() {
    if (ads1220_instance)
        ads1220_instance->alertNewDataFromISR();
}