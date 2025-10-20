#include <SPI.h>
#include "ads131/ADS131M0x.h"

#include "sampling.h"
#include "util.h"


void IRAM_ATTR ads131_alert();

class PowerSampler_ADS131;

PowerSampler_ADS131 *ads131_instance = nullptr;

bool adc15_set_frequency(uint32_t frequency);

class PowerSampler_ADS131 : public PowerSampler {
    ADS131M0x ads{};

    TaskNotification notification;
    volatile bool new_data = false;

    char SPI_RX_Buff[10];
    long ads131_rx_Data[10];
    int SPI_RX_Buff_Count = 0;

    Sample lastSample{};

public:
    static constexpr auto VREF = 2.50f;

    const uint8_t storageId = 0;

    uint8_t getStorageId() const override { return storageId; };

    SPIClass SpiADC {HSPI};

    bool init() {
        if (ads131_instance)
            return false;
        ads131_instance = this;

        //SPI.begin(13, 12, 11, settings.Pin_ADS1220_CS);

        // if (!ads.())
        //    return false;


        /*
        pinMode(settings.Pin_ADS131_Rst, OUTPUT);
        for (int i = 0; i < 10; i++) {
            ESP_LOGI("ads", "gpio test signal");
            digitalWrite(settings.Pin_ADS131_Rst, HIGH);
            delay(1000);
            digitalWrite(settings.Pin_ADS131_Rst, LOW);
            delay(1000);
        }
        ESP_LOGI("ads", "gpio test signal ENDS");
*/

        constexpr auto ADC15_FREQ_MODE_HIGH_RESOLUTION = 8192000;
        adc15_set_frequency(ADC15_FREQ_MODE_HIGH_RESOLUTION);


        ESP_LOGI("ads", "Init ads131");
        ads.setClockSpeed(ADC15_FREQ_MODE_HIGH_RESOLUTION); // SPI clock
        new_data = false;
        attachInterrupt(digitalPinToInterrupt(settings.Pin_ADS131_Drdy), ads131_alert, FALLING);
        ESP_LOGI("ads1220", "Interrupt attached to pin %d", settings.Pin_ADS131_Drdy);
        ads.reset(settings.Pin_ADS131_Rst);
        ads.begin(&SpiADC, settings.Pin_ADS131_Clk, settings.Pin_ADS131_Miso, settings.Pin_ADS131_Mosi,
                  settings.Pin_ADS131_Cs,
                  settings.Pin_ADS131_Drdy);

        ads.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);

        ads.setChannelPGA(0, CHANNEL_PGA_1); // pga gain=1 => ±1.2 V FSR

        ESP_LOGI("ads131", "waiting  for ads to become ready..");
        while (!digitalRead(settings.Pin_ADS131_Drdy)) vTaskDelay(1);
        //while (!new_data) vTaskDelay(1); //  Wait for DRDY to go high indicating it is ok// to talk to ADC
        new_data = false;

        // check number of channels
        while (true) {
            auto idReg = ads.readRegister(REG_ID);
            uint8_t chan_cnt = (idReg >> 8) & 0b1111;
            ESP_LOGI("ads", "Read Channel Count %hhu (registerVal=%hu)", chan_cnt, idReg);
            if (chan_cnt != 0) break;
            delay(100);
        }

        // calibrate ESP32_ADS131M02 U *

        assert(ads.setPowerMode(POWER_MODE_HIGH_RESOLUTION));
        ads.setGlobalChop(1);
        ads.setGlobalChopDelay(16); // default:16
        // In global-chop mode, noise is improved by a factor of √ 2.
        //ads.setDrdyFormat(1); // 1: any channel
        // * Changing the DRDY_SEL[1:0] bits has no effect on DRDY behavior in global-chop mod because phase calibration is automatically disabled in global-chop mode.
        ads.setOsr(OSR_16384); // OSR (Modulator oversampling ratio selection) https://www.ti.com/lit/ds/symlink/ads131m02.pdf#page=50
        ads.setChannelEnable(0, 1);
        ads.setChannelEnable(1, 0);
        ads.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);

        return true;
    }

    void startReading() {
        // TODO read register
    }


    bool firstRead = true;

    bool hasData() {
        if (!new_data && !ads.isDataReady()) {
            notification.subscribe();
            //new_data = true;
            if (!notification.wait(4000) || !new_data) return false;
        }
        new_data = false;


        lastSample.setTimeNow();
        if (firstRead) {
            ads.readADC();
            firstRead = false;
        }
        auto dat = ads.readADC();
        lastSample.u = (float) dat.ch0 * 1.2f / (float) ((2 << (23 - 1)) - 1) * (101.505f / 1.505f) * 1.00458259;
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


// ReSharper disable once CppNonInlineFunctionDefinitionInHeaderFile
void IRAM_ATTR ads131_alert() {
    if (ads131_instance)
        ads131_instance->alertNewDataFromISR();
}


bool adc15_ltc_write(uint8_t oct, uint16_t dac, uint8_t cfg) {
    uint16_t temp_data = 0;
    temp_data |= (uint16_t) (oct & 0xF) << 12;
    temp_data |= (uint16_t) (dac & 0x3FF) << 2;
    temp_data |= (uint16_t) (cfg & 0x3) << 0;
    uint8_t tx_data[2] = {temp_data >> 8, temp_data};

    SPI.setBitOrder(MSBFIRST);
    SPI.setFrequency(4000000);
    SPI.setDataMode(SPI_MODE0);
    SPI.begin(settings.Pin_ADS131_Clk,
              settings.Pin_ADS131_Miso,
              settings.Pin_ADS131_Mosi);
    pinMode(settings.Pin_ADS131_CsClk, OUTPUT);
    digitalWrite(settings.Pin_ADS131_CsClk, LOW);
    delayMicroseconds(1);
    SPI.writeBytes(tx_data, 2);
    delayMicroseconds(1);
    digitalWrite(settings.Pin_ADS131_CsClk, HIGH);
    SPI.end();
    return true;
}

bool adc15_set_frequency(uint32_t frequency) {
    constexpr auto OCT_DIVIDER = 1039.0;
    constexpr auto OCT_RES = 3.322;
#define DAC_OFFSET      2048.0
#define DAC_RES         2078
#define DAC_OCT_OFFSET  10
#define ADC15_LDC_CFG_POWER_ON      2
#define ADC15_LDC_CFG_POWER_DOWN    3
    uint8_t oct = OCT_RES * log10(frequency / OCT_DIVIDER);
    uint16_t dac = DAC_OFFSET - ((DAC_RES * pow(2, DAC_OCT_OFFSET + oct)) / (float) frequency);
    return adc15_ltc_write(oct, dac, ADC15_LDC_CFG_POWER_ON);
}

