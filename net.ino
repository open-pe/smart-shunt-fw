
#define WIFI_SSID "^__^"
#define WIFI_PASSWORD "modellbau"

#define MY_NTP_SERVER "de.pool.ntp.org"
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"


#include <time.h>  // time() ctime()

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#include <ESP8266WiFi.h>  // we need wifi to get internet access
#endif



void connect_wifi_async() {
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
}

void wait_for_wifi() {
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(50);
  }
  Serial.print("Connected to WiFi, RSSI ");
  Serial.print(WiFi.RSSI());
  Serial.print(", IP ");
  Serial.println(WiFi.localIP());
}


void influxWritePointsUDP(const Point *p, uint8_t len) {

    byte host[] = {192, 168, 178, 23};
    udp.beginPacket(host, 8001);
    String msg;
    for(uint8_t i = 0; i < len; ++i) {
      msg += p[i].toLineProtocol() + '\n';
    }
    udp.print(msg);
    udp.endPacket();
}


