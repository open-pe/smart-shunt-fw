//#include <queue>

#include <InfluxDbClient.h>
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




//constexpr int sampleBufMaxSize = 200;
//std::queue<Sample> sampleBuf;
//uint32_t numDropped = 0;

WiFiUDP udp;

InfluxDBClient client;

unsigned long StartTime = 0;
unsigned long LastTimeOut = 0;

PowerSampler_ADS ads;
PowerSampler_ESP32 esp_adc;

std::vector<EnergyCounter> energyCounters;

void ICACHE_RAM_ATTR NewDataReadyISR() {
  ads.alertNewDataFromISR();
}

void setup(void) {
  connect_wifi_async();

  Serial.begin(9600);
  Wire.setClock(400000UL);

  energyCounters.emplace_back({ads})
  energyCounters.emplace_back({esp_adc})

  for(auto ec : energyCounters) {
    if(!ec.power_sampler->init()) {
          Serial.println("Failed to initialize ADC.");
          while (1) yield();
    }
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

  // when multiplexing channels, TI recommnads single-shot mode
  ads.startReading();
}



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
    for (auto ec : energyCounters) {
      ec.summary((nowTime - LastTimeOut));
    }
    LastTimeOut = nowTime;
  }
}
