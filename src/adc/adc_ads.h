#include <SPI.h> // not sure why this is needed

#include <Adafruit_ADS1X15.h>

#include "sampling.h"

#if CONFIG_IDF_TARGET_ESP32S3
constexpr int ADS_READY_PIN = 6; // ESP32-S3: fugu=6;
#else
constexpr int ADS_READY_PIN = 34; // ESP32: default=12, fugu=34
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

void IRAM_ATTR ads_alert();

class PowerSampler_ADS;

PowerSampler_ADS *ads_instance = nullptr;


class PowerSampler_ADS : public PowerSampler {

    Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
    //Adafruit_ADS1015 ads; /* Use this for the 12-bit version */

    volatile bool new_data = false;


    int readUICycle = 0;
    bool readingU = false;

    Sample lastSample;

    const std::array<adsGain_t, 6> gains = {GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN};
    adsGain_t gainI = GAIN_EIGHT, gainU = GAIN_TWO;


    const std::array<int16_t, 6> adcThres = {0u, 17476u, 8738u, 4370u, 2185u, 1093u};

    adsGain_t chooseGain(int16_t adc) {
        int16_t absAdc = abs(adc);
        uint8_t newGain = 0;
        for (uint8_t i = adcThres.size() - 1; i >= 0; --i) {
            if (absAdc < adcThres[i]) {
                newGain = i;
                return gains[i];
            }
        }
    }

public:
    const uint8_t storageId = 0;

    uint8_t getStorageId() const override { return storageId; };

    bool init() {
        // RATE_ADS1115_128SPS (default)
        // RATE_ADS1115_250SPS, RATE_ADS1115_475SPS

        //ads.setDataRate(RATE_ADS1115_250SPS);
        //ads.setDataRate(RATE_ADS1115_475SPS);

        if (ads_instance)
            return false;

        ads_instance = this;

        if (std::is_same<decltype(ads), Adafruit_ADS1115>::value) {
            ads.setDataRate(RATE_ADS1115_860SPS);
            ESP_LOGI("ads", "ADS1115");
        } else {

            //ads.setDataRate(RATE_ADS1015_1600SPS); // -> sps=1050 (1+1/40 s/ps) i2c sdc @800khz
            //ads.setDataRate(RATE_ADS1015_3300SPS); // -> sps=1650 (1+1/40 s/ps)
            //ads.setDataRate(RATE_ADS1015_3300SPS);
            // for some reason RATE_ADS1115_860SPS gives higher sampling rate (105 power samples)
            ESP_LOGI("ads", "ADS1015");
        }


        if (!ads.begin())
            return false;

        // listen to the ADC's ALERT pin
        pinMode(ADS_READY_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(ADS_READY_PIN), ads_alert, FALLING);

        ESP_LOGI("ads", "Interrupt attached to pin %d", ADS_READY_PIN);

        return true;
    }

    void startReading() {
        //if ((++readUICycle % 4) == 0) {
        //if(++readUICycle > 3) {
        if ((++readUICycle % 40) == 0) {
            // 1.25 ADC_Sample/Power_Sample
            //if (random(0, 4) == 0) {  // random sampling better than cyclic
            // occasionally sample U
            startReadingU();
        } else {
            startReadingI();
        }
    }

    void startReadingI() {
        readingU = false;
        ads.setGain(gainI);
        ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
    }

    void startReadingU() {
        readingU = true;
        ads.setGain(gainU);
        ads.startADCReading(MUX_BY_CHANNEL[2], /*continuous=*/false);
    }

    float computeVolts(int16_t counts, adsGain_t gain) {
        uint8_t m_bitShift = std::is_same<decltype(ads), Adafruit_ADS1115>::value ? 0 : 4;
        // see data sheet Table 3
        float fsRange;
        switch (gain) {
            case GAIN_TWOTHIRDS:
                fsRange = 6.144f;
                break;
            case GAIN_ONE:
                fsRange = 4.096f;
                break;
            case GAIN_TWO:
                fsRange = 2.048f;
                break;
            case GAIN_FOUR:
                fsRange = 1.024f;
                break;
            case GAIN_EIGHT:
                fsRange = 0.512f;
                break;
            case GAIN_SIXTEEN:
                fsRange = 0.256f;
                break;
            default:
                fsRange = 0.0f;
        }
        return counts * (fsRange / (32768 >> m_bitShift));
    }

    void alertNewDataFromISR() {
        new_data = true;
    }

    bool hasData() {
        if (new_data) {
            int16_t adc = ads.getLastConversionResults();
            new_data = false;
            bool readU = readingU;
            startReading();
            processSampleFromADC(adc, readU);
            return !readU;
        }
        return false;
    }

    Sample getSample() {
        return lastSample;
    }


    void processSampleFromADC(int16_t adc, bool readU) {

        //auto &newGain(readU ? gainU : gainI);
        //newGain = chooseGain(adc);
        // TODO detect clipping

        if (readU) {
            constexpr auto adsImpedance = 6e6f; // see datasheet https://www.ti.com/lit/ds/symlink/ads1115.pdf#page=7
            constexpr auto ladderRLow = 5e3f;
            constexpr auto ladderRHigh = 200e3f;
            constexpr auto rLow = 1 / (1 / ladderRLow + 1 / adsImpedance);
            lastSample.u = computeVolts(adc, gainU) * ((ladderRHigh + rLow) / rLow);
        } else {
            lastSample.setTimeNow();
            lastSample.i = computeVolts(adc, gainI) * (1000.0f / 12.5f);
        }
    }
};


void IRAM_ATTR ads_alert() {
    if (ads_instance)
        ads_instance->alertNewDataFromISR();
}