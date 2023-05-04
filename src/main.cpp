#include <Arduino.h>
#include <Wire.h>

#include <InfluxDbClient.h>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "adc/ina226.h"

//#include "adc/adc_esp_dma.h"

#include "energy_counter.h"
#include "util.h"


InfluxDBClient client;

PowerSampler_ADS ads;
PowerSampler_INA226 ina226;
// PowerSampler_ESP32 esp_adc;


unsigned long LastTimeOut = 0;
unsigned long LastTimePrint = 0;

std::array<std::pair<std::string, PowerSampler *>, 2> samplers{
        std::pair<std::string, PowerSampler *>{"ESP32_ADS", &ads},
        {"ESP32_INA226", &ina226},
};

std::vector<EnergyCounter> energyCounters;

void i2c_scan() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    //delay (2000);           // wait 5 seconds for next scan
    //delay (2000);           // wait 5 seconds for next scan
}

bool disableWifi = false;

void setup(void) {

    Serial.begin(115200);
    Serial.println("Serial begin");


#if CONFIG_IDF_TARGET_ESP32S3
    Wire.setPins(15, 16);
#elif CONFIG_IDF_TARGET_ESP32
    // Wire.setPins(21, 22);
#else
#error "unknown target"
#endif

    Wire.begin();
    Wire.setClock(400000UL);

    if (!disableWifi)
        connect_wifi_async();

    for (auto p: samplers) {
        if (!p.second->init()) {
            Serial.print(p.first.c_str());
            Serial.println(": Failed to initialize sampler.");
        } else {
            energyCounters.emplace_back(EnergyCounter{p.second, p.first});
            ESP_LOGI("main", "Initialized energy counter for %s", p.first.c_str());
        }
    }

    if (energyCounters.empty()) {
        i2c_scan();
    }


    if (!disableWifi) {
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
    }
    // when multiplexing channels, TI recommnads single-shot mode
    for (auto &ec: energyCounters) {
        ec.sampler->startReading();
    }

    Serial.println("setup done.");
}

std::vector<Point> points_frame;

void loop(void) {
    constexpr bool hfWrites = false;

    unsigned long nowTime = micros();

    for (auto &ec: energyCounters) {
        ec.update();
    }

    /*    if (hfWrites) {
          Point point("smart_shunt");
          samplePoint(point, s, "ESP8266_proto1");
          pointFrame[i] = point;
        }

    if (hfWrites) influxWritePointsUDP(&pointFrame[0], pointFrame.size()); */

    if (nowTime - LastTimeOut > 50e3) {
        auto print = nowTime - LastTimePrint > 1000e3;

        for (auto &ec: energyCounters) {
            if (ec.newSamplesSinceLastSummary()) {
                auto p = ec.summary((nowTime - LastTimeOut), print);
                points_frame.push_back(p);
            }
        }

        if (print) {
            if (energyCounters.size() > 1)
                Serial.println("");
            LastTimePrint = nowTime;
        }

        if (points_frame.size() >= 18) {
            if (!disableWifi)
                influxWritePointsUDP(&points_frame[0], points_frame.size());
            points_frame.clear();
        }

        LastTimeOut = nowTime;
    }

    if (Serial.available() > 0) {
        if (Serial.readString().indexOf("r") != -1) {
            Serial.println("Reset, delay 1s");
            delay(1000);
            for (auto &ec: energyCounters) {
                ec.reset();
            }
        }
    }
}
