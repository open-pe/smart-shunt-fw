#include "util.h"

#include <cmath>
#include <Arduino.h>
#include <Wire.h>

#include "adc/sampling.h"
#include "settings.h"

#ifdef TARGET_STM32H5
#include "esp_compat.h"

std::string timeStr() {
    char buffer[26];
    unsigned long long ms = millis();
    unsigned long sec = (unsigned long)(ms / 1000);
    unsigned long msec = (unsigned long)(ms % 1000);
    snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu.%03lu",
             (sec / 3600) % 24, (sec / 60) % 60, sec % 60, msec);
    return std::string(buffer);
}

unsigned long long getTimeStamp(struct timeval *tv, int secFracDigits) {
    unsigned long long ms = millis();
    switch (secFracDigits) {
        case 0:  return ms / 1000;
        case 6:  return ms * 1000;
        case 9:  return ms * 1000000;
        case 3:
        default: return ms;
    }
}

void scan_i2c() {
    const char *TAG = "scan_i2c";
    byte error, address;
    int nDevices;

    ESP_LOGI(TAG, "Scanning I2C...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            ESP_LOGI(TAG, "Device found at address 0x%02hhX", address);
            nDevices++;
        } else if (error != 2) {
            ESP_LOGW(TAG, "Unknown error %hhu at address 0x%02hhX", error, address);
        }
    }
    if (nDevices == 0)
        ESP_LOGI(TAG, "No I2C devices found");
    else
        ESP_LOGI(TAG, "I2C scan done, %d devices found", nDevices);

    delay(5000);
}

void UART_LOG(const char *fmt, ...) {
    static char UART_LOG_buf[384];

    va_list args;
    va_start(args, fmt);
    vsnprintf(UART_LOG_buf, 380, fmt, args);
    va_end(args);
    auto l = strlen(UART_LOG_buf);
    UART_LOG_buf[l] = '\n';
    UART_LOG_buf[++l] = '\0';
    Serial.print(UART_LOG_buf);
}

#else

#include "secrets.h"
#include <InfluxDbClient.h>
#include <WiFiUDP.h>

#if defined(ESP32)
#include <WiFiMulti.h>
#include <driver/uart.h>
WiFiMulti wifiMulti;
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#include <ESP8266WiFi.h>
#endif

WiFiUDP udp;

void connect_wifi_async() {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
#define ADD_AP(ssid, password) wifiMulti.addAP(ssid, password);
    WIFI_AP_LIST(ADD_AP)
#undef ADD_AP
}

static bool wifiApsAdded = false;

void connect_wifi_async_once() {
    if (wifiApsAdded) return;
    connect_wifi_async();
    wifiApsAdded = true;
}

void wait_for_wifi() {
    while (wifiMulti.run() != WL_CONNECTED) {
        delay(50);
    }
    ESP_LOGI("util", "Connected to WiFi, RSSI %hhi IP=%s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
}

extern volatile int g_wifiRequest;
volatile int g_wifiRequest = 0;

[[noreturn]] void wifiTask(void *) {
    bool apsAdded = false;
    bool wasEnabled = false;
    while (true) {
        if (g_wifiRequest == 0) {
            if (wasEnabled) {
                WiFi.disconnect(true, true);
                WiFi.mode(WIFI_OFF);
                ESP_LOGI("wifi", "disconnected and radio off");
                wasEnabled = false;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        wasEnabled = true;

        if (WiFi.status() == WL_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        if (!apsAdded) {
            connect_wifi_async_once();
            apsAdded = true;
            ESP_LOGI("wifi", "wifiTask: starting connect attempt");
        }

        if (wifiMulti.run() == WL_CONNECTED) {
            ESP_LOGI("wifi", "Connected, RSSI %hhi IP=%s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
        } else {
            ESP_LOGW("wifi", "wifiMulti.run() failed, retrying in 10s");
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
}

void wifi_tick() {}

void udpFlushString(const IPAddress &host, uint16_t port, String &msg) {
    if (msg.length() > CONFIG_TCP_MSS) {
        ESP_LOGW("tele", "Payload len %d > TCP_MSS: %s", msg.length(), msg.substring(0, 200).c_str());
        msg.clear();
        return;
    }
    udp.beginPacket(host, port);
    udp.print(msg);
    udp.endPacket();
    msg.clear();
}

String influxMsgBuf;

void influxWritePointsUDP(const Point *p, uint8_t len, bool flush) {
    for (uint8_t i = 0; i < len; ++i) {
        influxWritePointUDP(p[0], flush && (i == len - 1));
    }
}

void influxWritePointUDP(const Point &p, bool flush) {
    constexpr int MTU = CONFIG_TCP_MSS;
    byte influxdbhost[] = {192, 168, 178, 180};
    auto port = 8086;

    auto lp = p.toLineProtocol();

    /* The wire format, and the only way to see it: these points go out over UDP,
     * which is fire-and-forget -- the device cannot tell a delivered point from one
     * that vanished, so a silent telemetry path looks identical to a working one.
     * ESP_LOGD is compiled out at the default CORE_DEBUG_LEVEL=3, so this costs
     * nothing normally; rebuild with -DCORE_DEBUG_LEVEL=4 to inspect what is
     * actually being sent. Worth checking when a sampler publishes only some
     * fields: Point::addField(float) DROPS NaN silently, so a temperature-only
     * sampler emits a point carrying T with no U/I/P -- correct, but it means a
     * missing field is not by itself evidence of a bug. */
    ESP_LOGD("tele", "influx lp: %s", lp.c_str());

    if (influxMsgBuf.length() + lp.length() >= MTU) {
        udpFlushString(influxdbhost, port, influxMsgBuf);
    }
    influxMsgBuf += lp + '\n';

    if (flush and influxMsgBuf.length() > 0) {
        udpFlushString(influxdbhost, port, influxMsgBuf);
    }
}

std::string timeStr() {
    char buffer[26];
    int millisec;
    struct timeval tv;

    gettimeofday(&tv, NULL);

    millisec = lrint(tv.tv_usec / 1000.0);
    if (millisec >= 1000) {
        millisec -= 1000;
        tv.tv_sec++;
    }

    strftime(buffer, 26, "%H:%M:%S", localtime(&tv.tv_sec));
    return std::string(buffer);
}

void pointFromSample(Point &p, const Sample &s, const char *device) {
    p.addTag("device", device);
    p.addField("I", s.i, 3);
    p.addField("U", s.u, 3);
    p.addField("P", s.p(), 3);
    p.addField("E", s.e, 3);
    p.setTime(s.t);
}

class PointDefaultConstructor : public Point {
public:
    PointDefaultConstructor()
        : Point("smart_shunt") {
    }

    PointDefaultConstructor(const Point &p)
        : Point(p) {
    }

    PointDefaultConstructor &operator=(const PointDefaultConstructor &p) {
        Point::operator=(p);
        return *this;
    }
};

void scan_i2c() {
    const char *TAG = "scan_i2c";
    byte error, address;
    int nDevices;

    ESP_LOGI(TAG, "Scanning I2C... (SDA=%hhu, SCL=%hhu)", settings.Pin_I2C_SDA,
             settings.Pin_I2C_SCL);

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            ESP_LOGI(TAG, "Device found at address 0x%02hhX", address);
            nDevices++;
        } else if (error != 2) {
            ESP_LOGW(TAG, "Unknown error %hhu at address 0x%02hhX", error, address);
        }
    }
    if (nDevices == 0)
        ESP_LOGI(TAG, "No I2C devices found");
    else
        ESP_LOGI(TAG, "I2C scan done, %d devices found", nDevices);

    delay(5000);
}

void UART_LOG(const char *fmt, ...) {
    static char UART_LOG_buf[384];

    va_list args;
    va_start(args, fmt);
    vsnprintf(UART_LOG_buf, 380, fmt, args);
    va_end(args);
    auto l = strlen(UART_LOG_buf);
    UART_LOG_buf[l] = '\n';
    UART_LOG_buf[++l] = '\0';
    printf("%s", UART_LOG_buf);
}

#endif
