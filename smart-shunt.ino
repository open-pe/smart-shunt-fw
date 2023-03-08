#include <Adafruit_ADS1X15.h>
#include <InfluxDbClient.h>
//# include "SafeQueue.h"


/**
* TODO:
- auto-gain
- sample I more often than U
- SampleRate?
- AC RMS
- Min-Max
- Peak-to-Peak
- Auto-correlation, max Frequency, DC+AC
*/

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads; /* Use this for the 12-bit version */

struct Sample {
  float u, i, p, e;
  time_t t;
}

//SafeQueue SampleQ<Sample>;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello!");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

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

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }

  // when multiplexing channels, TI recommnads single-shot mode (https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/563147/ads1115-ads1115-in-continuous-mode-with-alternating-channels)
  // Start continuous conversions.
}

float energy = 0;

unsigned long LastTime = 0;
unsigned long LastTimeOut = 0;
unsigned long NumSamples = 0;



void loop(void) {
  Sample s;

  //ads.setGain(GAIN_FOUR);   // +- 80 Ampere
  ads.setGain(GAIN_EIGHT);  // +- 40 Ampere
  int16_t adc01 = ads.readADC_Differential_0_1();
  s.i = ads.computeVolts(adc01) * (1000.0f / 12.5f) * (20.4f / 20.32f);

  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  int16_t adc2 = ads.readADC_SingleEnded(2);
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

  SampleQ.enqueue(s);

  if (NowTime - LastTimeOut > 1e6) {
    float sps = NumSamples / (NowTime * 1e-6);

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
    Serial.print(NowTime * 1e-6, 1);
    Serial.print("s");
    //Serial.print(adc2, BIN);
    Serial.println("");
    LastTimeOut = NowTime;
  }
}


void write_loop() {
  connect_wifi();
  InfluxDBClient client("http://homeassistant.local:8086", /*db*/ "open_pe");

  // Define data point with measurement name 'device_status`
  Point pointDevice("smart_shunt");
  pointDevice.addTag("device", "ESP8266_proto1");
  pointDevice.addField("uptime", millis());

  // Write data
  //client.writePoint(pointDevice);
}