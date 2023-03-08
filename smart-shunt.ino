#include <Adafruit_ADS1X15.h>
#include <InfluxDbClient.h>
//# include "SafeQueue.h"


/**
* TODO:
- i2c speed?
- auto-gain
- sample I more often than U
- SampleRate?
- AC RMS
- Min-Max
- Peak-to-Peak
- Auto-correlation, max Frequency, DC+AC


- do i need to pull one current sense input to V_DD/2 ?
If VOUT is connected to the ADC positive input (AINP) and VCM is connected to the ADC negative input (AINN),
VCM appears as a common-mode voltage to the ADC. This configuration allows pseudo-differential
measurements and uses the maximum dynamic range of the ADC if VCM is set at midsupply (VDD / 2). A resistor
divider from VDD to GND followed by a buffer amplifier can be used to generate VCM.

- Therefore, use a first-order RC filter with a cutoff frequency set at the output data rate or
10x higher as a generally good starting point for a system design. . Limit the filter resistor values to below 1 kΩ.
  fC = 1 / [2π · (R5 + R6) · CDIFF] 
Two common-mode filter capacitors (CCM1 and CCM2) are also added to offer attenuation of high-frequency,
common-mode noise components. Select a differential capacitor, CDIFF, that is at least an order of magnitude
(10x) larger than these common-mode capacitors because mismatches in these common-mode capacitors can
convert common-mode noise into differential noise. The best ceramic chip capacitors are C0G
(NPO), which have stable properties and low-noise characteristics.
- Analog inputs with differential connections must have a capacitor placed differentially across the inputs. Best
input combinations for differential measurements use adjacent analog input lines such as AIN0, AIN1 and
AIN2, AIN3. The differential capacitors must be of high quality. The best ceramic chip capacitors are C0G
(NPO), which have stable properties and low-noise characteristics.

*/

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads; /* Use this for the 12-bit version */

constexpr int READY_PIN = 12;

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

volatile bool new_data = false;
//void IRAM_ATTR
void ICACHE_RAM_ATTR NewDataReadyISR() {
  new_data = true;
  // Serial.println("ALERT!");
}

struct Sample {
  float u, i, p, e;
  time_t t;
};

InfluxDBClient client;

unsigned long StartTime = 0;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello!");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  //ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV


  // ads.setDataRate

  // RATE_ADS1115_128SPS (default)
  // RATE_ADS1115_250SPS, RATE_ADS1115_475SPS

  //ads.setDataRate(RATE_ADS1115_860SPS);
  ads.setDataRate(RATE_ADS1115_250SPS);
  Wire.setClock(400000UL);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }

  pinMode(READY_PIN, INPUT_PULLUP);
  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);

  // const String &serverUrl, const String &db, const String &user, const String &password, const char *certInfo
  client.setConnectionParamsV1("http://homeassistant.local:8086", /*db*/ "open_pe", "home_assistant", "h0me");
  client.setWriteOptions(WriteOptions()
                           .writePrecision(WritePrecision::MS)
                           .batchSize(1000)
                           .bufferSize(10000)
                           .flushInterval(2)
                           .retryInterval(5));

  connect_wifi();
  timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");

  StartTime = micros();
  // when multiplexing channels, TI recommnads single-shot mode (https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/563147/ads1115-ads1115-in-continuous-mode-with-alternating-channels)
  // Start continuous conversions.
}

float energy = 0;


unsigned long LastTime = 0;
unsigned long LastTimeOut = 0;
unsigned long NumSamples = 0;


void waiting_for_adc() {
  client.checkBuffer();
}



void loop(void) {
  Sample s;

  //ads.setGain(GAIN_FOUR);   // +- 80 Ampere
  ads.setGain(GAIN_EIGHT);  // +- 40 Ampere

  int16_t adc01 = ads.readADC_Differential_0_1();

  //new_data = false;
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  //while (!new_data) waiting_for_adc();
  //int16_t adc01 =  ads.getLastConversionResults();

  s.i = ads.computeVolts(adc01) * (1000.0f / 12.5f) * (20.4f / 20.32f);

  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV

  int16_t adc2 = ads.readADC_SingleEnded(2);

  //new_data = false;
  //ads.startADCReading(MUX_BY_CHANNEL[2], /*continuous=*/false);
  //while (!new_data) waiting_for_adc();
  //int16_t adc2 =  ads.getLastConversionResults();

  s.u = ads.computeVolts(adc2) * ((222.0f + 10.13f) / 10.13f * (10.0f / 9.9681f));

  unsigned long NowTime = micros();
  ++NumSamples;
  s.p = s.i * s.u;

  if (LastTime != 0) {
    unsigned long dt_us = NowTime - LastTime;
    energy += s.p * (dt_us * (1e-6f / 3600.f));
    s.e = energy;
  }
  LastTime = NowTime;

  Point point("smart_shunt");
  point.addTag("device", "ESP8266_proto1");
  point.addField("I", s.i);
  point.addField("U", s.u);
  point.addField("P", s.p);
  point.addField("E", s.e);
  client.writePoint(point);

  //SampleQ.enqueue(s);

  if (NowTime - LastTimeOut > 1e6) {
    float sps = NumSamples / ((NowTime - StartTime) * 1e-6);

    Serial.print("U=");
    Serial.print(s.u, 4);
    Serial.print("V, I=");
    Serial.print(s.i, 3);
    Serial.print("A, P=");
    Serial.print(s.p, 3);
    Serial.print("W, E=");
    Serial.print(s.e);
    Serial.print("Wh, N=");
    Serial.print(NumSamples);
    Serial.print(", SPS=");
    Serial.print(sps, 1);
    Serial.print(", T=");
    Serial.print((NowTime - StartTime) * 1e-6, 1);
    Serial.print("s");
    //Serial.print(adc2, BIN);
    Serial.println("");
    LastTimeOut = NowTime;
  }

  if (new_data) {
    // Serial.println("new_data");
    //new_data = false;
  }
}
