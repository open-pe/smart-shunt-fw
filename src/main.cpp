

#include <InfluxDbClient.h>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "energy_counter.h"
#include "util.h"


constexpr int READY_PIN = 12;

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif



InfluxDBClient client;

PowerSampler_ADS ads;
PowerSampler_ESP32 esp_adc;

unsigned long LastTimeOut = 0;

std::vector<EnergyCounter> energyCounters;

void ICACHE_RAM_ATTR NewDataReadyISR() {
  ads.alertNewDataFromISR();
}

void setup(void) {
  connect_wifi_async();

  Serial.begin(9600);
  Wire.setClock(400000UL);

 // "ESP8266_proto1"
  energyCounters.emplace_back(EnergyCounter{&ads, "ESP32_ADS"});
  energyCounters.emplace_back(EnergyCounter{&esp_adc, "ESP32_ADC"});

  for(auto ec : energyCounters) {
    if(!ec.sampler->init()) {
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
