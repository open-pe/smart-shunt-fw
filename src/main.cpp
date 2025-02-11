#include <Arduino.h>
#include <Wire.h>

#include <InfluxDbClient.h>
#include <map>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "adc/ina226.h"
#include "adc/ina228.h"
#include "adc/ads1220.h"
#include "adc/ads1262.h"

#include "lcd.h"

#include "driver/uart.h"
//#include "adc/adc_esp_dma.h"
//#if ARDUINO_USB_MODE == 1
#include "USB.h"
//#endif

#include "energy_counter.h"
#include "util.h"


InfluxDBClient client;

PowerSampler_ADS ads;
PowerSampler_ADS1220 ads1220;
//PowerSampler_ADS1262 ads1262;

PowerSampler_INA226 ina226;
PowerSampler_INA228 ina228_40{0x40};
PowerSampler_INA228 ina228_41{0x41};
PowerSampler_INA228 ina228_42{0x42};
// PowerSampler_ESP32 esp_adc;


unsigned long LastTimeOut = 0;
unsigned long LastTimePrint = 0;

//std::map
//std::array<std::pair<std::string, PowerSampler *>, 2> samplers{
//        std::pair<std::string, PowerSampler *>{"ESP32_ADS", &ads},
//        {"ESP32_INA226", &ina226},
//};

std::map<std::string, PowerSampler *> samplers{
        {"ESP32_ADS",      &ads},
        {"ESP32_ADS1220",  &ads1220},
        //{"ESP32_ADS1262",  &ads1262},

        {"ESP32_INA226",   &ina226},

        {"ESP32_INA228",   &ina228_40},
        {"ESP32_INA228_2", &ina228_41},
        {"ESP32_INA228_3", &ina228_42},
};

std::vector<EnergyCounter> energyCounters;


bool disableWifi = false;

LCD lcd;

unsigned long timeLastWakeEvent = 0;

[[noreturn]] void realTimeTask(void *arg);

[[noreturn]] void nonRealTimeTask(void *arg);

constexpr auto RT_CORE = 1;
constexpr auto RT_PRIO = 20;  // highest priority is 24

void vTaskGetRunTimeStats();

void setup(void) {

    Serial.begin(115200);
//#if CONFIG_IDF_TARGET_ESP32S3
    // for unknown reason need to initialize uart0 for serial reading (see loop below)
    // Serial.available() works under Arduino IDE (for both ESP32,ESP32S3), but always returns 0 under platformio
    // so we access the uart port directly. on ESP32 the Serial.begin() is sufficient (since it uses the uart0)
    uartInit(0);
//#endif

    //USBSerial.begin();
    // USB.begin();


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

    xTaskCreatePinnedToCore(realTimeTask, "loopRt", 4096 * 4, NULL, RT_PRIO, NULL, RT_CORE);
    xTaskCreatePinnedToCore(nonRealTimeTask, "loopy", 4096 * 4, NULL, 1, NULL, RT_CORE - 1);

    //TaskStatus_t *pxTaskStatusArray[20];
    //uxTaskGetSystemState(pxTaskStatusArray, 20);
    //vTaskGetRunTimeStats();
}

std::vector<Point> points_frame;

[[noreturn]] void realTimeTask(void *arg) {
    assert(xPortGetCoreID() == RT_CORE);
    vTaskPrioritySet(nullptr, RT_PRIO);

    while (true) {
        for (auto &ec: energyCounters) {
            ec.update();
        }
    }
}

void loop() {
    vTaskDelay(1000);
}

void handleConsoleInput(const String &buf) {
    while (1) {
        String inp(buf);
        inp.trim();

        if (inp.isEmpty()) break;

        if (inp == "reset") {
            //Serial0.println("Reset, delay 1s");
            ESP_LOGW("main", "Reset in 1s!");
            delay(1000);
            for (auto &ec: energyCounters) {
                ec.reset();
            }
        } else if (inp.startsWith("calibrate ")) {
            // calibrate ESP32_INA228 I 1.0028870
            // calibrate ESP32_INA228 I *0.9997247927345282
            // calibrate ESP32_INA228 U *1.000983433436737
            // calibrate ESP32_ADS U *1.0003957914179227
            // calibrate ESP32_ADS I *1.0017
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

            float was = dim == "U" ? ec->calibFactorU : ec->calibFactorI;
            UART_LOG("%s: set calibration factor for [%s] = %.8f (was %.8f)", samplerName.c_str(), dim.c_str(),
                     multiply ? (was * factor) : factor, was);
            if (dim == "U") {
                ec->setCalibrationFactors(factor, NAN, multiply);
            } else if (dim == "I") {
                ec->setCalibrationFactors(NAN, factor, multiply);
            } else {
                ESP_LOGW("main", "unknown dim %s", dim.c_str());
            }
        } else if (inp.startsWith("ina22x-resistor-range")) {
            int devIdx = 0;
            size_t i = strlen("ina22x-resistor-range");
            if (inp.length() > i && (inp[i] >= '2' and inp[i] <= '3')) {
                devIdx = 1 + (inp[i] - '2');
                ++i;
            }
            auto resStr = inp.substring(i + 1, inp.indexOf(' ', i + 1));
            i += resStr.length() + 1;
            auto range = inp.substring(i).toFloat();
            auto res = resStr.toFloat();

            if (ina226_instance) {
                UART_LOG("INA226 setResistorRange(%.6f, %.6f)", res, range);
                ina226_instance->setResistorRange(res, range);
            } else if (ina228_instance[devIdx]) {
                UART_LOG("INA228[%i] setResistorRange(%.3fmΩ, %.3fA)", devIdx, res * 1e3f, range);
                ina228_instance[devIdx]->setResistorRange(res, range);
            } else {
                ESP_LOGW("main", "No INA22x instance!");
            }

        } else if (inp == "wifi on") {
            disableWifi = false;
            connect_wifi_async();
        } else if (inp == "wifi off") {
            disableWifi = true;
            WiFi.disconnect(true);
        } else if (inp == "help") {
            UART_LOG("ina22x-resistor-range <resistance> <max expected current>");
        } else {
            UART_LOG("Unknown command '%s'. enter 'help' for help", inp.c_str());
        }

        break;
    }
}

void update() {
    constexpr bool hfWrites = false;

    unsigned long nowTime = micros();

    assert(xPortGetCoreID() != RT_CORE);

    //ESP_LOGI("main", "Loop!");

    for (auto &ec: energyCounters) {
        ec.consumeQueue();
    }

    /*    if (hfWrites) {
          Point point("smart_shunt");
          samplePoint(point, s, "ESP8266_proto1");
          pointFrame[i] = point;
        }

    if (hfWrites) influxWritePointsUDP(&pointFrame[0], pointFrame.size()); */

    if (nowTime - LastTimeOut > 19e3) {
        auto print = nowTime - LastTimePrint > 2000e3;

        for (auto &ec: energyCounters) {
            if (ec.newSamplesSinceLastSummary()) {
                if (std::abs(ec.winPrint.P.getMean()) >= 0.0005f) {
                    timeLastWakeEvent = nowTime;
                }

                auto p = ec.summary((nowTime - LastTimeOut), print);
                if(p.hasFields())
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
    static char buf[128];
    static int buf_pos = 0;
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
    if (length > 0) {
        length = uart_read_bytes(uart_num, buf + buf_pos, min(length, 127 - buf_pos), 20);
        uart_write_bytes(uart_num, buf + buf_pos, length); // echo
        buf_pos += length;
    }

    auto eol = strchr(buf, '\n');
    while (eol) { // enable using break below
        *eol = 0; // terminate string at line break
        buf_pos = 0; // reset buf
        handleConsoleInput(buf);
        timeLastWakeEvent = nowTime;
        break;
    }

    static String serialBuf;
    if (Serial.available()) {
        auto r = Serial.readString();
        serialBuf += r;
        Serial.write(r.c_str()); // echo
        Serial.flush();
        int lb;
        while ((lb = serialBuf.indexOf('\n')) != -1) {
            String line = serialBuf.substring(0, lb);
            handleConsoleInput(line);
            serialBuf = serialBuf.substring(lb + 1);
        }
        timeLastWakeEvent = nowTime;
    }

    if (nowTime - timeLastWakeEvent > (1e6 * 3600)) {
        UART_LOG("Zero power for 1h, going to sleep");
        ESP.deepSleep(0);
        // TODO setup alarm here to wake up
    }
}

void nonRealTimeTask(void *arg) {
    while (1) {
        update();
        vTaskDelay(5);
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


// This example demonstrates how a human readable table of run time stats
// information is generated from raw data provided by uxTaskGetSystemState().
// The human readable table is written to pcWriteBuffer
void vTaskGetRunTimeStats() {
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    // Make sure the write buffer does not contain a string.
    //*pcWriteBuffer = 0x00;

    // Take a snapshot of the number of tasks in case it changes while this
    // function is executing.
    uxArraySize = uxTaskGetNumberOfTasks();

    // Allocate a TaskStatus_t structure for each task.  An array could be
    // allocated statically at compile time.
    pxTaskStatusArray = (TaskStatus_t *) pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        // Generate raw status information about each task.
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
// For percentage calculations.
        ulTotalRunTime /= 100UL;

// Avoid divide by zero errors.
        if (ulTotalRunTime > 0) {
// For each populated position in the pxTaskStatusArray array,
// format the raw data as human readable ASCII data
            for (x = 0; x < uxArraySize; x++)
// What percentage of the total run time has the task used?
// This will always be rounded down to the nearest integer.
// ulTotalRunTimeDiv100 has already been divided by 100.
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

            auto aff = xTaskGetAffinity(pxTaskStatusArray[x].xHandle);

            if (ulStatsAsPercentage > 0UL) {
                printf("%s (core#%i)\t\t%lu\t\t%lu%%\r\n", pxTaskStatusArray[x].pcTaskName, aff,
                       pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage);
            } else {
// If the percentage is zero here then the task has
// consumed less than 1% of the total run time.
                printf("%s (core#%i)\t\t%lu\t\t1%%\r\n", pxTaskStatusArray[x].pcTaskName, aff,
                       pxTaskStatusArray[x].ulRunTimeCounter);
            }

            //pcWriteBuffer += strlen((char *) pcWriteBuffer);
        }
    }

// The array is no longer needed, free the memory it consumes.
    vPortFree(pxTaskStatusArray);
}
