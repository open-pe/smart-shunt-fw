#pragma once

#include <string>
#include <cstdint>

#ifdef TARGET_STM32H5
#include "esp_compat.h"
#include <FreeRTOS.h>
#include <task.h>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Point.h"
#endif

std::string timeStr();

#ifndef TARGET_STM32H5
void connect_wifi_async();
void wait_for_wifi();
void wifi_tick();
[[noreturn]] void wifiTask(void *);
extern volatile int g_wifiRequest;

class Point;
void influxWritePointUDP(const Point &p, bool flush=false);
void influxWritePointsUDP(const Point *p, uint8_t len, bool flush = false);
void uartInit(int port_num);
#endif

void scan_i2c();

void UART_LOG(const char *fmt, ...);

unsigned long long getTimeStamp(struct timeval *tv, int secFracDigits);

class TaskNotification {
    TaskHandle_t readingTask = nullptr;
public:
    inline void subscribe(bool updateTaskHandle = false) {
        if (updateTaskHandle or unlikely(readingTask == nullptr))
            readingTask = xTaskGetCurrentTaskHandle();
    }

    void unsubscribe() {
        readingTask = nullptr;
    }

    inline bool wait(uint32_t ms) {
        void(this);
        return ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(ms)) == 1;
    }

#ifndef TARGET_STM32H5
    bool IRAM_ATTR notifyFromIsr() {
        BaseType_t higherWokenTask;
        if (readingTask) {
            vTaskNotifyGiveFromISR(readingTask, &higherWokenTask);
            if (higherWokenTask) {
                portYIELD_FROM_ISR();
                return true;
            }
        }
        return false;
    }
#else
    bool notifyFromIsr() {
        if (readingTask) {
            BaseType_t higherWokenTask = pdFALSE;
            xTaskNotifyFromISR(readingTask, 1, eIncrement, &higherWokenTask);
            portYIELD_FROM_ISR(higherWokenTask);
            return (bool)higherWokenTask;
        }
        return false;
    }
#endif

    void notify() {
        if (readingTask) xTaskNotifyGive(readingTask);
    }
};
