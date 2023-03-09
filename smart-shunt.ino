#include <Adafruit_ADS1X15.h>
#include <InfluxDbClient.h>
//#include <RingBuf.h>

#include <queue>


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

struct Sample {
  float u, i, p, e;
  time_t t;
};

volatile bool new_data = false;
volatile unsigned long NumSamples = 0;

int readUICycle = 0;
bool readingU = false;

unsigned long LastTime = 0;
float LastVoltage = 0.0f;
float Energy = 0;

Sample LastSample;


class PointDefaultConstructor : public Point {
public:
  PointDefaultConstructor()
    : Point("smart_shut") {}
  PointDefaultConstructor(const Point &p)
    : Point(p) {}

  PointDefaultConstructor &operator=(const PointDefaultConstructor &p) {
    Point::operator=(p);
    return *this;
  }
};

//RingBuf<PointDefaultConstructor, 400> pointBuf;

std::queue<PointDefaultConstructor> pointBuf;

void startReading() {
  if ((++readUICycle % 3) == 0) {
    // occasionally sample U
    startReadingU();
  } else {
    startReadingI();
  }
}

//void IRAM_ATTR
void ICACHE_RAM_ATTR NewDataReadyISR() {
  int16_t adc = ads.getLastConversionResults();

  if (readingU) {
    LastVoltage = ads.computeVolts(adc) * ((222.0f + 10.13f) / 10.13f * (10.0f / 9.9681f));
    startReading();
  } else {
    Sample &s(LastSample);
    s.i = ads.computeVolts(adc) * (1000.0f / 12.5f) * (20.4f / 20.32f);
    s.u = LastVoltage;
    s.p = s.i * s.u;

    unsigned long nowTime = micros();
    if (LastTime != 0) {
      unsigned long dt_us = nowTime - LastTime;
      Energy += s.p * (dt_us * (1e-6f / 3600.f));
      s.e = Energy;
    } else {
      s.e = 0.0f;
    }

    startReading();


    Point point("smart_shunt");
    point.addTag("device", "ESP8266_proto1");
    point.addField("I", s.i);
    point.addField("U", s.u);
    point.addField("P", s.p);
    point.addField("E", s.e);
    point.setTime(WritePrecision::MS);
    pointBuf.push(point);

    ++NumSamples;
    LastTime = nowTime;
    new_data = true;
  }
}


InfluxDBClient client;

unsigned long StartTime = 0;

void setup(void) {
  Serial.begin(9600);


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
                           .batchSize(40)
                           .bufferSize(400)
                           .flushInterval(.5f)
                           .retryInterval(5));

  connect_wifi();
  timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");

  StartTime = micros();
  // when multiplexing channels, TI recommnads single-shot mode (https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/563147/ads1115-ads1115-in-continuous-mode-with-alternating-channels)
  // Start continuous conversions.

  startReadingU();
}





unsigned long LastTimeOut = 0;



void waiting_for_adc() {
  client.checkBuffer();
}

void startReadingI() {
  readingU = false;
  //ads.setGain(GAIN_FOUR);   // +- 80 Ampere
  ads.setGain(GAIN_EIGHT);  // +- 40 Ampere
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
}

void startReadingU() {
  readingU = true;
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  ads.startADCReading(MUX_BY_CHANNEL[2], /*continuous=*/false);
}

std::vector<PointDefaultConstructor> pointFrame(40);

void loop(void) {


  if (new_data) {
    //Serial.println("New Data!");
    //Serial.println(pointBuf.size());
    uint16_t i = 0;
    //PointDefaultConstructor p;

    noInterrupts();
    while (i < pointFrame.size() && !pointBuf.empty()) {
      pointFrame[i] = pointBuf.front();
      pointBuf.pop();
      ++i;
    }
    new_data = false;
    interrupts();

    if (i == 0) {
      Serial.println("new_data but 0 points in buf!");
    }

    for (uint16_t j = 0; j < i; ++j)
      client.writePoint(pointFrame[j]);
  }

  client.checkBuffer();

  unsigned long nowTime = micros();

  if (nowTime - LastTimeOut > 1e6) {
    float sps = NumSamples / ((nowTime - StartTime) * 1e-6);
    Sample s = LastSample;

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
    Serial.print((nowTime - StartTime) * 1e-6, 1);
    Serial.print("s");
    //Serial.print(adc2, BIN);
    Serial.println("");
    LastTimeOut = nowTime;
  }
}