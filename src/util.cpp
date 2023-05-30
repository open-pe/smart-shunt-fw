#include "util.h"

#include <cmath>
#include <Arduino.h>
#include <InfluxData.h>
#include <Wire.h>

#include "adc/sampling.h"

#include <InfluxDbClient.h>
#include <WiFiUDP.h>

#define WIFI_SSID "^__^"
#define WIFI_PASSWORD "modellbau"

#define MY_NTP_SERVER "de.pool.ntp.org"
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"



#if defined(ESP32)
#include <WiFiMulti.h>
#include <driver/uart.h>

WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#include <ESP8266WiFi.h>  // we need wifi to get internet access
#endif

WiFiUDP udp;



void connect_wifi_async() {
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
}

void wait_for_wifi() {
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(50);
  }
  ESP_LOGI("util", "Connected to WiFi, RSSI %hhi IP=%s", WiFi.RSSI(), WiFi.localIP().toString().c_str());
}

void udpFlushString(const IPAddress &host, uint16_t port, String &msg) {
    if (msg.length() > CONFIG_TCP_MSS) {
        ESP_LOGW("tele", "Payload len %d > TCP_MSS: %s", msg.length(), msg.substring(0, 200).c_str());
        msg.clear();
        return;
    }

   // bytesSent += asyncUdp.writeTo((uint8_t *) msg.c_str(), msg.length(), host, port);

    udp.beginPacket(host, port);
    udp.print(msg);
    udp.endPacket();

    msg.clear();
}


void influxWritePointsUDP(const Point *p, uint8_t len) {
    constexpr int MTU = CONFIG_TCP_MSS;

    /*
    static IPAddress host{};
    if(uint32_t(host) == 0) {
        ESP_LOGI("tele", "resolving hostname");
        host = MDNS.queryHost("homeassistant.local");
        ESP_LOGI("tele", "resolved to %s",host.toString());
    }
     */

    // byte host[] = {192, 168, 0, 185};
    byte host[] = {192, 168, 178, 28};

    auto port = 8086;


    String msg;

    for (uint8_t i = 0; i < len; ++i) {
        auto lp = p[i].toLineProtocol();
        if (msg.length() + lp.length() >= MTU) {
            udpFlushString(host, port, msg);
        }
        msg += lp + '\n';
    }

    if (msg.length() > 0) {
        udpFlushString(host, port, msg);
    }
}



std::string timeStr() {
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
  return std::string(buffer);
  // printf("%s.%03d\n", buffer, millisec);
  //Serial0.print(buffer);
  //Serial0.print('.');
  //Serial0.print(millisec);
  //Serial0.print(' ');

  /*
  char buff[100];
    time_t now = time (0);
    strftime (buff, 100, "%H:%M:%S.000", localtime (&now));
    printf ("%s\n", buff);
    return 0;
  */
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
    : Point("smart_shut") {}
  PointDefaultConstructor(const Point &p)
    : Point(p) {}

  PointDefaultConstructor &operator=(const PointDefaultConstructor &p) {
    Point::operator=(p);
    return *this;
  }
};


/*
 * const int BUF_SIZE = 1024;
char *uartBuf = (char *) malloc(BUF_SIZE);
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
const int PIN_TX = 34, PIN_RX = 33;
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
* /
}
 */


void scan_i2c() {
    const char *TAG = "scan_i2c";
    byte error, address;
    int nDevices;

    ESP_LOGI(TAG, "Scanning I2C...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        /*
            0	success
            1	data too long to fit in transmit buffer
            2	received NACK on transmit of address
            3	received NACK on transmit of data
            4	other error
         */

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

static char UART_LOG_buf[384];

void UART_LOG(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(UART_LOG_buf, 380, fmt, args);
    va_end(args);
    auto l = strlen(UART_LOG_buf);
    UART_LOG_buf[l] = '\n';
    UART_LOG_buf[++l] = '\0';
    uart_write_bytes(0, UART_LOG_buf, l);
}