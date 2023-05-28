#pragma once

#include <string>
#include <cstdint>


std::string timeStr();


void connect_wifi_async();

void wait_for_wifi();

class Point;

void influxWritePointsUDP(const Point *p, uint8_t len);


void scan_i2c();

void uartInit(int port_num);



void UART_LOG(const char *fmt, ...);