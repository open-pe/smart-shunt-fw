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



constexpr int READY_PIN = 12;

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif


volatile bool new_data = false;
volatile unsigned long NumSamples = 0;

int readUICycle = 0;
bool readingU = false;

volatile unsigned long LastTime = 0;

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


void ICACHE_RAM_ATTR NewDataReadyISR() {
  ads.alertNewDataFromISR();
}

InfluxDBClient client;

unsigned long StartTime = 0;
unsigned long LastTimeOut = 0;
unsigned long NSamplesLastTimeOut = 0;


void setup(void) {
  Serial.begin(9600);

  connect_wifi_async();

  Wire.setClock(400000UL);

  if (!ads.init()) {
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
  ads.startReading();
}


std::vector<PointDefaultConstructor> pointFrame(10);
uint16_t maxBufSize = 0;


struct EnergyCounter {
  PowerSampler *sampler;

  struct MeanWindow winI {};
  struct MeanWindow winU {};
  struct MeanWindow winP {};

  unsigned long long windowTimestamp = 0;

  void update() {
    unsigned long nowTime = micros();

    PowerSampler &ps(*sampler);
    if (ps.hasData()) {
      Sample s = ps.getSample();

      unsigned long nowTime = micros();
      auto P = s.p();
      if (lastTime != 0) {
        // we use simple trapezoidal rule here
        unsigned long dt_us = nowTime - lastTime;
        Energy += (double)((LastP + P) * 0.5f * (dt_us * (1e-6f / 3600.f)));
        s.e = Energy;
      } else {
        s.e = 0.0f;
      }

          ++NumSamples;
    LastTime = nowTime;
    LastP = P;

      winI.add(s.i);
      winU.add(s.u);
      winP.add(s.p());
      windowTimestamp = s.t;
    } else {
auto lastTime = LastTime;
  if (nowTime > lastTime && (nowTime - lastTime) > 4e6) {
    Serial.println("");
    Serial.println("Timeout waiting for new sample!");
    Serial.println((nowTime - lastTime) * 1e-6);
    Serial.println(lastTime);
    Serial.println(nowTime);
    Serial.println("");
    startReading();  // TODO this is for some reason not necessary
  }
    }
  }
};




WiFiUDP udp;



void loop(void) {
  constexpr bool hfWrites = false;

  unsigned long nowTime = micros();

  for (auto ec : energyCounters) {
    ec.update();
  }

  
  /*    if (hfWrites) {
        Point point("smart_shunt");
        samplePoint(point, s, "ESP8266_proto1");
        pointFrame[i] = point;
      }

  if (hfWrites) influxWritePointsUDP(&pointFrame[0], pointFrame.size()); */


if (nowTime - LastTimeOut > 500e3) {
  // capture
  auto nSamples = NumSamples;
  auto energy = Energy;

  // compute
  float sps = (nSamples - NSamplesLastTimeOut) / ((nowTime - LastTimeOut) * 1e-6);
  float i_mean = MeanI.pop(), u_mean = MeanU.pop(), p_mean = MeanP.pop();


  if (!hfWrites) {
    Point point("smart_shunt");
    point.addTag("device", "ESP8266_proto1");
    point.addField("I", i_mean, 4);
    point.addField("U", u_mean, 4);
    point.addField("P", p_mean, 4);
    point.addField("E", energy, 4);
    point.setTime(windowTimestamp);
    influxWritePointsUDP(&point, 1);
    //client.writePoint(point);
  }

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
