
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
#include <ESP8266WiFi.h>            // we need wifi to get internet access
#endif


void connect_wifi() {
   configTime(MY_TZ, MY_NTP_SERVER);

  // Connect WiFi
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("Connected to WiFi, RSSI ");
  Serial.println( WiFi.RSSI());
}