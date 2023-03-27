

#include <InfluxDbClient.h>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "energy_counter.h"
#include "util.h"

constexpr int READY_PIN = 7;

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
unsigned long LastTimePrint = 0;

std::array<EnergyCounter, 2> energyCounters{
    EnergyCounter{&esp_adc, "ESP32_int"},
    EnergyCounter{&ads, "ESP32_ADS"},
};

void ICACHE_RAM_ATTR NewDataReadyISR()
{
  ads.alertNewDataFromISR();
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Serial begin");

  Wire.setClock(400000UL);
  Wire.setPins(15, 16);

  connect_wifi_async();

  // "ESP8266_proto1"
  // energyCounters.emplace_back(EnergyCounter{&ads, "ESP32_ADS"});
  // energyCounters.emplace_back({&esp_adc, "ESP32_internalADC"});

  for (auto &ec : energyCounters)
  {
    if (!ec.sampler->init())
    {
      Serial.print(ec.name.c_str());
      Serial.println(": Failed to initialize ADC.");
      while (1)
        yield();
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
                             .flushInterval(1) // uint16! min is 1
                             .retryInterval(0) // 0=disable retry
  );

  wait_for_wifi();
  timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");

  // when multiplexing channels, TI recommnads single-shot mode
  for (auto &ec : energyCounters)
  {
    ec.sampler->startReading();
  }
}

std::vector<Point> points_frame;

void loop(void)
{
  constexpr bool hfWrites = false;

  unsigned long nowTime = micros();

  for (auto &ec : energyCounters)
  {
    ec.update();
  }

  /*    if (hfWrites) {
        Point point("smart_shunt");
        samplePoint(point, s, "ESP8266_proto1");
        pointFrame[i] = point;
      }

  if (hfWrites) influxWritePointsUDP(&pointFrame[0], pointFrame.size()); */

  if (nowTime - LastTimeOut > 20e3)
  {
    auto print = nowTime - LastTimePrint > 500e3;

    for (auto &ec : energyCounters)
    {
      if (ec.newSamplesSinceLastSummary())
      {
        auto p = ec.summary((nowTime - LastTimeOut), print);
        points_frame.push_back(p);
      }
    }

    if (print)
    {
      if (energyCounters.size() > 1)
        Serial.println("");
      LastTimePrint = nowTime;
    }

    if (points_frame.size() >= 18)
    {
      influxWritePointsUDP(&points_frame[0], points_frame.size());
      points_frame.clear();
    }

    LastTimeOut = nowTime;
  }

  if (Serial.available() > 0)
  {
    if (Serial.readString().indexOf("r") != -1)
    {
      Serial.println("Reset, delay 1s");
      delay(1000);
      for (auto &ec : energyCounters)
      {
        ec.reset();
      }
    }
  }

  // delay(100);
}
