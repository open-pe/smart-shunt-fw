#include <cstdint>


void printTime();


void connect_wifi_async();
void wait_for_wifi();

class Point;
void influxWritePointsUDP(const Point *p, uint8_t len);