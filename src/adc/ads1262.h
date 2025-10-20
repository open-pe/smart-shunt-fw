#include <SPI.h>
#include "ads1262/ads1262.h"

#include "sampling.h"
#include "util.h"


void IRAM_ATTR ads1262_alert();

class PowerSampler_ADS1262;

PowerSampler_ADS1262 *ads1262_instance = nullptr;


class PowerSampler_ADS1262 : public PowerSampler {

    ads1262 ads{};

    TaskNotification notification;
    volatile bool new_data = false;

    char SPI_RX_Buff[10];
    long ads1262_rx_Data[10];
    int SPI_RX_Buff_Count = 0;

    Sample lastSample{};

public:
    static constexpr auto VREF = 2.50f;

    const uint8_t storageId = 0;

    uint8_t getStorageId() const override { return storageId; };

    bool init() {
        if (ads1262_instance)
            return false;
        ads1262_instance = this;

        SPI.begin(13, 12, 11, settings.Pin_ADS1220_CS);

        // if (!ads.())
        //    return false;

        // listen to the ADC's ALERT pin
        pinMode(settings.Pin_ADS1220_DRDY, INPUT_PULLUP);
        pinMode(settings.Pin_ADS1262_START, OUTPUT);

        attachInterrupt(digitalPinToInterrupt(settings.Pin_ADS1220_DRDY), ads1262_alert, FALLING);

        ESP_LOGI("ads1220", "Interrupt attached to pin %d", settings.Pin_ADS1220_DRDY);


        ads.ads1262_Init();

        // use Use RUNMODE (bit 6, MODE0) (pg61)
// 0 => continouse (default)
//

        ads.ads1262_Enable_Start();

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


        lastSample.setTimeNow();

        auto SPI_RX_Buff_Ptr = ads.ads1262_Read_Data();
        memcpy(SPI_RX_Buff, SPI_RX_Buff_Ptr, 5);

        ads1262_rx_Data[0] = (unsigned char) SPI_RX_Buff[1];  // read 4 bytes adc count
        ads1262_rx_Data[1] = (unsigned char) SPI_RX_Buff[2];
        ads1262_rx_Data[2] = (unsigned char) SPI_RX_Buff[3];
        ads1262_rx_Data[3] = (unsigned char) SPI_RX_Buff[4];

        auto uads1262Count = (signed long) (((unsigned long) ads1262_rx_Data[0] << 24) |
                                            ((unsigned long) ads1262_rx_Data[1] << 16) | (ads1262_rx_Data[2] << 8) |
                                            ads1262_rx_Data[3]);//get the raw 32-bit adc count out by shifting
        auto sads1262Count = (signed long) (uads1262Count);      // get signed value
        auto resolution = (double) VREF / (2 << 30);       //resolution= Vref/(2^n-1) , Vref=2.5, n=no of bits
        auto volt_V = (resolution) * (float) sads1262Count;     // voltage = resolution * adc count
        auto volt_mV = volt_V * 1000;                           // voltage in mV

        lastSample.u = -volt_V * (102.37f / 2.37f);


        //auto v = ads.getVoltage_mV();


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


void IRAM_ATTR ads1262_alert() {
    if (ads1262_instance)
        ads1262_instance->alertNewDataFromISR();
}