#include <Adafruit_ADS1X15.h>
#include <InfluxDbClient.h>
//#include <RingBuf.h>

#include <queue>
#include <WiFiUDP.h>


/**
* TODO:
- linearity test
- 160 mhz
- handle integer flip over
- auto-gain
- AC RMS
- Min-Max
- Peak-to-Peak
- Auto-correlation, max Frequency, DC+AC
- Linear Calibration (atomated)
- trapezoidal integration

- Telemetry https://github.com/Overdrivr/Telemetry
- MQTT ?

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

// minimize memory footprint
// - use raw ADC samples
// - remove p, e
// - instead of timeval store dt to the prev sample (uint_16)
struct Sample {
  float u, i, e;
  unsigned long long t;  // 8byte

  inline float p() const {
    return u * i;
  }
};


volatile bool new_data = false;
volatile unsigned long NumSamples = 0;

int readUICycle = 0;
bool readingU = false;

volatile unsigned long LastTime = 0;
float LastVoltage = 0.0f;
double Energy = 0;

float LastP = 0.0f;


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

constexpr int sampleBufMaxSize = 200;
std::queue<Sample> sampleBuf;
uint32_t numDropped = 0;

const std::array<adsGain_t, 6> gains = { GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN };
adsGain_t gainI = GAIN_EIGHT, gainU = GAIN_TWO;

void startReading() {
  //if ((++readUICycle % 4) == 0) {
  if (random(0, 4) == 0) {  // random sampling better than cyclic
    // occasionally sample U
    startReadingU();
  } else {
    startReadingI();
  }
}

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

//void IRAM_ATTR
void ICACHE_RAM_ATTR NewDataReadyISR() {
  int16_t adc = ads.getLastConversionResults();

  //auto &newGain(readingU ? gainU : gainI);
  //newGain = chooseGain(adc);

  // TODO detect clipping

  if (readingU) {
    LastVoltage = ads.computeVolts(adc) * ((222.0f + 10.13f) / 10.13f * (10.0f / 9.9681f));
  } else {
    Sample s;
    struct timeval u_time;
    gettimeofday(&u_time, NULL);
    s.t = getTimeStamp(&u_time, 3);
    s.i = ads.computeVolts(adc) * (1000.0f / 12.5f) * (20.4f / 20.32f);
    s.u = LastVoltage;

    unsigned long nowTime = micros();
    auto P = s.p();
    if (LastTime != 0) {
      // we use simple trapezoidal rule here
      unsigned long dt_us = nowTime - LastTime;
      Energy += (double)((LastP + P) * 0.5f * (dt_us * (1e-6f / 3600.f)));
      s.e = Energy;
    } else {
      s.e = 0.0f;
    }

    while (sampleBuf.size() > sampleBufMaxSize) {
      sampleBuf.pop();
      ++numDropped;
    }
    sampleBuf.push(s);


    ++NumSamples;
    LastTime = nowTime;
    LastP = P;
    new_data = true;
  }

  startReading();
}


InfluxDBClient client;

unsigned long StartTime = 0;
unsigned long LastTimeOut = 0;
unsigned long NSamplesLastTimeOut = 0;

void setup(void) {
  Serial.begin(9600);

  connect_wifi_async();

  // RATE_ADS1115_128SPS (default)
  // RATE_ADS1115_250SPS, RATE_ADS1115_475SPS

  ads.setDataRate(RATE_ADS1115_860SPS);
  //ads.setDataRate(RATE_ADS1115_250SPS);
  //ads.setDataRate(RATE_ADS1115_475SPS);
  Wire.setClock(400000UL);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1) yield();
  }

  // listen to the ADC's ALERT pin
  pinMode(READY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);

  client.setConnectionParamsV1("http://homeassistant.local:8086", /*db*/ "open_pe", "home_assistant", "h0me");
  client.setWriteOptions(WriteOptions()
                           .writePrecision(WritePrecision::MS)
                           .batchSize(200)
                           .bufferSize(400)
                           .flushInterval(1)  // uint16! min is 1
                           .retryInterval(0)  // 0=disable retry
  );

  wait_for_wifi();
  timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");

  StartTime = micros();

  // when multiplexing channels, TI recommnads single-shot mode
  startReading();
}

void waiting_for_adc() {
  //client.checkBuffer();
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

std::vector<PointDefaultConstructor> pointFrame(10);
uint16_t maxBufSize = 0;

struct MeanWindow {
  // TODO trapezoidal sum?
  float sum;
  uint32_t num;

  float getMean() const {
    return sum / num;
  }
  void clear() {
    sum = 0.f;
    num = 0.f;
  }
  void add(float x) {
    sum += x;
    ++num;
  }

  float pop() {
    float m = getMean();
    clear();
    return m;
  }

  MeanWindow() {
    clear();
  }
};


void printTime() {
  char buffer[26];
  int millisec;
  struct timeval tv;

  gettimeofday(&tv, NULL);

  millisec = lrint(tv.tv_usec / 1000.0);  // Round to nearest millisec
  if (millisec >= 1000) {                 // Allow for rounding up to nearest second
    millisec -= 1000;
    tv.tv_sec++;
  }

  strftime(buffer, 26, "%H:%M:%S", localtime(&tv.tv_sec));
  // printf("%s.%03d\n", buffer, millisec);
  Serial.print(buffer);
  Serial.print('.');
  Serial.print(millisec);
  Serial.print(' ');

  /*
  char buff[100];
    time_t now = time (0);
    strftime (buff, 100, "%H:%M:%S.000", localtime (&now));
    printf ("%s\n", buff);
    return 0;
  */
}

struct MeanWindow MeanI {};
struct MeanWindow MeanU {};
struct MeanWindow MeanP {};
unsigned long long windowTimestamp = 0;

WiFiUDP udp;

void loop(void) {

  constexpr bool hfWrites = false;

  unsigned long nowTime = micros();

  if (new_data) {
    uint16_t i = 0;
    noInterrupts();
    {
      if (sampleBuf.size() > maxBufSize) maxBufSize = sampleBuf.size();
      while (i < pointFrame.size() && !sampleBuf.empty()) {
        const Sample &s(sampleBuf.front());
        MeanI.add(s.i);
        MeanU.add(s.u);
        MeanP.add(s.p());
        windowTimestamp = s.t;

        if (hfWrites) {
          Point point("smart_shunt");
          point.addTag("device", "ESP8266_proto1");
          point.addField("I", s.i, 3);
          point.addField("U", s.u, 3);
          point.addField("P", s.p(), 3);
          point.addField("E", s.e, 3);
          point.setTime(s.t);
          pointFrame[i] = point;
        }

        sampleBuf.pop();
        ++i;
      }
    }
    new_data = false;
    interrupts();

    if (i == 0) {
      Serial.println("new_data but 0 points in buf!");
    }

    if (hfWrites)
      influxWritePointsUDP(&pointFrame[0], pointFrame.size());
    //for (uint16_t j = 0; j < i; ++j)
    //  client.writePoint(pointFrame[j]);
  } else {
    auto lastTime = LastTime;
    if (nowTime > lastTime && (nowTime - lastTime) > 2e6) {
      Serial.println("");
      Serial.println("Timeout waiting for new sample!");
      Serial.println((nowTime - lastTime) * 1e-6);
      Serial.println(lastTime);
      Serial.println(nowTime);
      Serial.println("");
      noInterrupts();
      startReading();  // TODO this is for some reason not necessary
      interrupts();
    }
  }

  //if (hfWrites)
  //  client.checkBuffer();

  if (nowTime - LastTimeOut > 1e6) {
    // capture
    auto nSamples = NumSamples;
    auto energy = Energy;

    // compute
    float sps = (nSamples - NSamplesLastTimeOut) / ((nowTime - LastTimeOut) * 1e-6);
    float i_mean = MeanI.pop(), u_mean = MeanU.pop(), p_mean = MeanP.pop();

/*
    if (!hfWrites) {
      Point point("smart_shunt");
      point.addTag("device", "ESP8266_proto1");
      point.addField("I", i_mean, 4);
      point.addField("U", u_mean, 4);
      point.addField("P", p_mean, 4);
      point.addField("E", energy, 4);
      point.setTime(windowTimestamp);
      //influxWritePointsUDP(&point, 1);
      //client.writePoint(point);
    }
*/
    printTime();
    Serial.print("U=");
    Serial.print(u_mean, 4);
    Serial.print("V, I=");
    Serial.print(i_mean, 3);
    Serial.print("A, P=");
    Serial.print(p_mean, 3);
    Serial.print("W, E=");
    Serial.print(energy, 3);
    Serial.print("Wh, N=");
    Serial.print(NumSamples);
    Serial.print(", SPS=");
    Serial.print(sps, 1);
    Serial.print(", T=");
    Serial.print((nowTime - StartTime) * 1e-6, 1);
    Serial.print("s");
    Serial.print(", maxBufSize=");
    Serial.print(maxBufSize);
    Serial.print(", numDropped=");
    Serial.print(numDropped);
    //Serial.print(adc2, BIN);
    Serial.println();

    NSamplesLastTimeOut = nSamples;
    LastTimeOut = nowTime;
  }
}
