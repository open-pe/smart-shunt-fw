#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_adc/adc_continuous.h>


/**
 * Docs
 * - https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/api-reference/peripherals/adc_continuous.html
 */
constexpr int ESP32_ADC_DMA_READ_LEN        =            256;

class PowerSampler_ESP32_DMA : public PowerSampler {
    static const uint8_t PIN_I0 = 4;
    static const uint8_t PIN_I1 = 5;
    static const uint8_t PIN_U = 6;

    esp_adc_cal_characteristics_t adc_chars;

    uint32_t readADC_Cal(int ADC_Raw) {
        return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
    }

    // https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#adc-attenuation
    float raw2V(int raw) {
        //constexpr float Vmax = 1.750f; // 6db
        //return raw * Vmax / 4095.0f;
        return esp_adc_cal_raw_to_voltage(raw, &adc_chars) * 1e-3f;
    }

public:

    bool init() {
        adc_continuous_handle_t handle = NULL;
        adc_continuous_handle_cfg_t adc_config = {
                .max_store_buf_size = 1024,
                .conv_frame_size = ESP32_ADC_DMA_READ_LEN,
        };

        adc_continuous_new_handle(&adc_config, &handle);

        adc1_config_width(ADC_WIDTH_BIT_12);

        adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
        adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_6);
        adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_6);

        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 1100, &adc_chars);

        return true;
    }

    void startReading() { /*nop*/}

    bool hasData() {
        // we do the actual read in getSample(), so always have data to sample
        return true;
    }

    Sample getSample() {
        Sample s;
        auto i0 = adc1_get_raw(ADC1_CHANNEL_3);
        auto i1 = adc1_get_raw(ADC1_CHANNEL_4);
        auto u = adc1_get_raw(ADC1_CHANNEL_5);
        // TODO detect clipping
        s.setTimeNow();
        s.i = (raw2V(i0) - raw2V(i1)) * (1000.0f / 12.5f) * (20.4f / 20.32f) * (23.855f / 23.911f);
        s.u = raw2V(u) * ((222.0f + 10.13f) / 10.13f * (10.0f / 9.9681f) * (30999.f / 30751.f) * (16043.f / 15662.f));
        return s;
    }
};
