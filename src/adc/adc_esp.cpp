class PowerSampler_ESP32 : public PowerSampler {
  constexpr int PIN_I0 = 36;
  constexpr int PIN_I1 = 37;
  constexpr int PIN_U = 38;

  esp_adc_cal_characteristics_t adc_chars;



  uint32_t readADC_Cal(int ADC_Raw) {
    return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
  }

public:

  void init() {
    adcAttachPin(PIN_I0);                             // adc1ch0 SENSOR_VP
    adcAttachPin(PIN_I1);                             // adc1ch1 
    adcAttachPin(PIN_U);                              // adc1ch2 
    analogSetPinAttenuation(PIN_I0, ADC_ATTEN_DB_6);  // 150 mV ~ 1750 mV
    analogSetPinAttenuation(PIN_I1, ADC_ATTEN_DB_6);  // 150 mV ~ 1750 mV
    analogSetPinAttenuation(PIN_U, ADC_ATTEN_DB_6);   // 150 mV ~ 1750 mV
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  }

  void hasData() {
    return true;
  }

  Sample getSample() {
    Sample s;
    auto i0 = analogRead(PIN_I0);
    auto i1 = analogRead(PIN_I1) auto u = analogRead(PIN_U);
    // TODO detect clipping
    s.setTimeNow();
    s.i = (readADC_Cal(i1) - readADC_Cal(i0)) * (1000.0f / 12.5f) * (20.4f / 20.32f);
    s.u = readADC_Cal(analogRead(u)) * ((222.0f + 10.13f) / 10.13f * (10.0f / 9.9681f));
    s.setTimeNow();
    return s;
  }
}
