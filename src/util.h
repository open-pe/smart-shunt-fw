#pragma once

#include <string>
#include <cstdint>
//#include

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Point.h"

std::string timeStr();


void connect_wifi_async();

void wait_for_wifi();

class Point;

void influxWritePointUDP(const Point &p, bool flush=false);
void influxWritePointsUDP(const Point *p, uint8_t len, bool flush = false);


void scan_i2c();

void uartInit(int port_num);



void UART_LOG(const char *fmt, ...);



class TaskNotification {
    // https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/03-Direct-to-task-notifications/02-As-binary-semaphore
    // notifications are faster than semaphores!
    TaskHandle_t readingTask = nullptr;
public:
    inline void subscribe(bool updateTaskHandle = false) {
        if (updateTaskHandle or unlikely(readingTask == nullptr))
            readingTask = xTaskGetCurrentTaskHandle();
    }

    void unsubscribe() {
        readingTask = nullptr;
    }

    /***
     *
     * @param ms
     * @return false on timeout
     */
    inline bool wait(uint32_t ms) {
        void(this);
        //assert(readingTask and readingTask == xTaskGetCurrentTaskHandle());
        return ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(ms)) == 1; // pdTRUE: binary semaphore
    }

    /***
     *
     * @return true if a higher priority task has been woken
     */
    bool IRAM_ATTR notifyFromIsr() {
        BaseType_t higherWokenTask; // = must yield
        if (readingTask) {
            vTaskNotifyGiveFromISR(readingTask, &higherWokenTask);
            if (higherWokenTask) {
                portYIELD_FROM_ISR();
                return true;
            }
        }
        return false;
    }

    void notify() {
        xTaskNotifyGive(readingTask);
        //vTaskDelay();
    }
};