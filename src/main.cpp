#include <Arduino.h>
#include <Wire.h>

#include <InfluxDbClient.h>
#include <map>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "adc/ina226.h"
#include "adc/ina228.h"

#include "lcd.h"

#include "driver/uart.h"
//#include "adc/adc_esp_dma.h"

#include "energy_counter.h"
#include "util.h"


InfluxDBClient client;

PowerSampler_ADS ads;
PowerSampler_INA226 ina226;
PowerSampler_INA228 ina228;
// PowerSampler_ESP32 esp_adc;


unsigned long LastTimeOut = 0;
unsigned long LastTimePrint = 0;

//std::map
//std::array<std::pair<std::string, PowerSampler *>, 2> samplers{
//        std::pair<std::string, PowerSampler *>{"ESP32_ADS", &ads},
//        {"ESP32_INA226", &ina226},
//};

std::map<std::string, PowerSampler *> samplers{
        {"ESP32_ADS",    &ads},
        {"ESP32_INA226", &ina226},
        {"ESP32_INA228", &ina228},
};

std::vector<EnergyCounter> energyCounters;


bool disableWifi = false;

LCD lcd;

unsigned long timeLastWakeEvent = 0;

void setup(void) {

    //Serial.begin(115200);
//#if CONFIG_IDF_TARGET_ESP32S3
    // for unknown reason need to initialize uart0 for serial reading (see loop below)
    // Serial.available() works under Arduino IDE (for both ESP32,ESP32S3), but always returns 0 under platformio
    // so we access the uart port directly. on ESP32 the Serial.begin() is sufficient (since it uses the uart0)
    uartInit(0);
//#endif


    ESP_LOGI("main", "SmartShunt started");


    Wire.begin(
            settings.Pin_I2C_SDA,
            settings.Pin_I2C_SCL,
            800000UL
    );

    if (!lcd.init()) {
        ESP_LOGW("main", "Failed to initialize LCD");
        //scan_i2c();
    }


    if (!disableWifi) {
        connect_wifi_async();
        wait_for_wifi();
        timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");
    }

    for (auto p: samplers) {
        if (!p.second->init()) {
            ESP_LOGI("main", "%s: Failed to initialize sampler.", p.first.c_str());
        } else {
            energyCounters.emplace_back(EnergyCounter{p.second, p.first, p.second->getStorageId()});
            ESP_LOGI("main", "Initialized energy counter for %s", p.first.c_str());
        }
    }

    if (energyCounters.empty()) {
        scan_i2c();
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


    }
    // when multiplexing channels, TI recommnads single-shot mode
    for (auto &ec: energyCounters) {
        ec.sampler->startReading();
    }


}

std::vector<Point> points_frame;


void loop() {
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
        auto print = nowTime - LastTimePrint > 2000e3;

        for (auto &ec: energyCounters) {
            if (ec.newSamplesSinceLastSummary()) {
                if (std::abs(ec.winPrint.P.getMean()) >= 0.0005f) {
                    timeLastWakeEvent = nowTime;
                }

                auto p = ec.summary((nowTime - LastTimeOut), print);
                points_frame.push_back(p);

                if (print) {
                    lcd.updateValues(ec.printSample);
                }
            }
        }

        if (print) {
            if (energyCounters.size() > 1)
                UART_LOG("");
            LastTimePrint = nowTime;
        }

        if (points_frame.size() >= 18) {
            if (!disableWifi)
                influxWritePointsUDP(&points_frame[0], points_frame.size());
            points_frame.clear();
        }

        LastTimeOut = nowTime;
    }


    // for some reason Serial.available() doesn't work under platformio
    // so access the uart port directly

    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    char data[128];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
    length = uart_read_bytes(uart_num, data, min(length, 127), 100);
    while (length) {
        data[length] = 0;
        String inp(data);
        inp.trim();

        timeLastWakeEvent = nowTime;

        if (inp == "reset") {
            //Serial0.println("Reset, delay 1s");
            ESP_LOGW("main", "Reset in 1s!");
            delay(1000);
            for (auto &ec: energyCounters) {
                ec.reset();
            }
        } else if (inp.startsWith("calibrate ")) {
            std::string samplerName = inp.substring(10, inp.indexOf(' ', 10)).c_str();
            size_t i = 10 + samplerName.size() + 1;

            EnergyCounter *ec = nullptr;
            for (auto &ec_: energyCounters) {
                if (ec_.name == samplerName) {
                    ec = &ec_;
                    break;
                }
            }

            if (!ec) {
                ESP_LOGW("main", "Sampler with name %s not found", samplerName.c_str());
                break;
            }

            auto dim = inp.substring(i, i + 1);
            dim.toUpperCase();
            i += 2;
            bool multiply = (inp.charAt(i) == '*');
            if (multiply)++i;

            auto factor = inp.substring(i).toFloat();

            if (!checkCalibrationFactorBounds(factor)) {
                ESP_LOGW("main", "Calibration factor %.9f out of bounds, rejected", factor);
                break;
            }

            // TODO send factors to influxDB

            UART_LOG("%s: set calibration factor for [%s] = %.8f (was %.8f)", samplerName.c_str(), dim.c_str(), factor,
                     dim == "U" ? ec->calibFactorU : ec->calibFactorI);
            if (dim == "U") {
                ec->setCalibrationFactors(factor, NAN, multiply);
            } else if (dim == "I") {
                ec->setCalibrationFactors(NAN, factor, multiply);
            } else {
                ESP_LOGW("main", "unknown dim %s", dim.c_str());
            }
        } else if (inp.startsWith("ina22x-resistor-range")) {
            size_t i = strlen("ina22x-resistor-range");
            auto resStr = inp.substring(i + 1, inp.indexOf(' ', i + 1));
            i += resStr.length() + 1;
            auto range = inp.substring(i).toFloat();
            auto res = resStr.toFloat();

            if (ina226_instance) {
                UART_LOG("INA226 setResistorRange(%.6f, %.6f)", res, range);
                ina226_instance->setResistorRange(res, range);
            } else if (ina228_instance) {
                UART_LOG("INA228 setResistorRange(%.6f, %.6f)", res, range);
                ina228_instance->setResistorRange(res, range);
            } else {
                ESP_LOGW("main", "No INA22x instance!");
            }

        } else if (inp == "wifi on") {
            connect_wifi_async();
        } else if (inp == "wifi off") {
            WiFi.disconnect(true);
        } else if (inp == "help") {
            UART_LOG("ina22x-resistor-range <resistance> <max expected current>");
        } else {
            UART_LOG("Unknown command. enter 'help' for help");
        }
        break;
    }

    if (nowTime - timeLastWakeEvent > (1e6 * 3600)) {
        UART_LOG("Zero power for 1h, going to sleep");
        ESP.deepSleep(0);
        // TODO setup alarm here to wake up
    }
}


const int BUF_SIZE = 1024;
QueueHandle_t uart_queue;

void uartInit(int port_num) {

    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // UART_HW_FLOWCTRL_CTS_RTS
            .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

// tx=34, rx=33, stack=2048


#if CONFIG_IDF_TARGET_ESP32S3
    //const int PIN_TX = 34, PIN_RX = 33;
    const int PIN_TX = 43, PIN_RX = 44;
#else
    const int PIN_TX = 1, PIN_RX = 3;
#endif

    ESP_ERROR_CHECK(uart_set_pin(port_num, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(port_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(port_num, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, intr_alloc_flags));


/* uart_intr_config_t uart_intr = {
     .intr_enable_mask = (0x1 << 0) | (0x8 << 0),  // UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
     .rx_timeout_thresh = 1,
     .txfifo_empty_intr_thresh = 10,
     .rxfifo_full_thresh = 112,
};
uart_intr_config((uart_port_t) 0, &uart_intr);  // Zero is the UART number for Arduino Serial
*/
}
