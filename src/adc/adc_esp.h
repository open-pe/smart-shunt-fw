#include <driver/adc.h>
#include <esp_adc_cal.h>

class PowerSampler_ESP32 : public PowerSampler {
  //static const int PIN_I0 = 36;
  //static const int PIN_I1 = 37;
  //static const int PIN_U = 38;

    static const uint8_t PIN_I0 = 4;
  static const uint8_t PIN_I1 = 5;
  static const uint8_t PIN_U = 6;

  esp_adc_cal_characteristics_t adc_chars;



  uint32_t readADC_Cal(int ADC_Raw) {
    return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
  }

  // https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#adc-attenuation
  float raw2V(int raw) {
    constexpr float Vmax = 1.750f; // 6db
    return raw * Vmax / 4095.0f;
  }

public:

  bool init() {

    adc1_config_width(ADC_WIDTH_BIT_12 );

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
    s.i = (raw2V(i1) - raw2V(i0)) * (1000.0f / 12.5f) * (20.4f / 20.32f);
    s.u = raw2V(u) * ((222.0f + 10.13f) / 10.13f * (10.0f / 9.9681f));
    return s;
  }
};
