class PowerSampler_ADS : public PowerSampler {

  Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
  //Adafruit_ADS1015 ads; /* Use this for the 12-bit version */

volatile bool new_data = false;


int readUICycle = 0;
bool readingU = false;

  float LastVoltage = 0.0f;

  const std::array<adsGain_t, 6> gains = { GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN };
  adsGain_t gainI = GAIN_EIGHT, gainU = GAIN_TWO;




  const std::array<int16_t, 6> adcThres = { 0u, 17476u, 8738u, 4370u, 2185u, 1093u };

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

  bool init() {
    // RATE_ADS1115_128SPS (default)
    // RATE_ADS1115_250SPS, RATE_ADS1115_475SPS

    //ads.setDataRate(RATE_ADS1115_250SPS);
    //ads.setDataRate(RATE_ADS1115_475SPS);

    ads.setDataRate(RATE_ADS1115_860SPS);
    return ads.begin()
  }




  void startReading() {
    if ((++readUICycle % 4) == 0) {
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
    uint8_t m_bitShift = 0;  // ads1115
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

  void alertNewDataFromIRS() {
    new_data=true;
  }

bool hasData() {
  return new_data;
}

  Sample getSample() {
    int16_t adc = ads.getLastConversionResults();
    new_data = false;
    bool readU = readingU;
    ads.startReading();
    processSampleFromADC(adc, readU);
  }


  void processSampleFromADC(int16_t adc, bool readU) {

    //auto &newGain(readU ? gainU : gainI);
    //newGain = chooseGain(adc);
    // TODO detect clipping

    if (readU) {
      lastVoltage = computeVolts(adc, gainU) * ((222.0f + 10.13f) / 10.13f * (10.0f / 9.9681f));
    } else {
      Sample s;
      s.setTimeNow();
      s.i = computeVolts(adc, gainI) * (1000.0f / 12.5f) * (20.4f / 20.32f);
      s.u = lastVoltage;
      return s;

    }
  }
}
