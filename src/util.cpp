#include <cmath>
#include <Arduino.h>
#include <InfluxData.h>

#include "adc/sampling.h"

#include <InfluxDbClient.h>
#include <WiFiUDP.h>

#define WIFI_SSID "^__^"
#define WIFI_PASSWORD "modellbau"

#define MY_NTP_SERVER "de.pool.ntp.org"
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03"



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

WiFiUDP udp;



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



void printTime() {
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
  // printf("%s.%03d\n", buffer, millisec);
  Serial.print(buffer);
  Serial.print('.');
  Serial.print(millisec);
  Serial.print(' ');

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