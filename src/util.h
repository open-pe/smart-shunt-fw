#pragma once

#include <string>
#include <cstdint>
#include <cmath>    // isnan/isinf/fabsf in SiFmt
#include <cstdio>   // snprintf in SiFmt

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

/// Formats a value with an SI prefix, so one format string serves a 7 uV ADC
/// offset and a 1.2 kW load without either rendering as 0.0000.
///
/// Ladder is p, n, µ, m, (unity), k, M, G -- µ is the SI symbol, emitted as
/// UTF-8, matching the "°" the same log line already carries. The small end is
/// the reason this
/// exists: EnergyCounter::summary()'s old "%7.4fV" printed every reading of the
/// shunt-adc zero channel as a flat 0.0000, which reads as "no signal" rather
/// than "too small for this format".
///
/// NaN and Inf print as themselves, NOT as 0 -- a value that could not be
/// computed must not render as a valid small measurement.
///
/// Returns a pointer into its own storage, so use it as a temporary in the
/// printf call itself; the temporary lives until the end of the full
/// expression, which is exactly as long as printf needs it.
struct SiFmt {
    char buf[24];

    SiFmt(float v, const char *unit) {
        if (std::isnan(v)) { snprintf(buf, sizeof buf, "%8s%s", "nan ", unit); return; }
        if (std::isinf(v)) {
            snprintf(buf, sizeof buf, "%8s%s", v < 0 ? "-inf " : "inf ", unit);
            return;
        }

        static const char *const kPrefix[] = {"p", "n", "µ", "m", "", "k", "M", "G"};
        constexpr int kUnity = 4;
        constexpr int kLast = 7;

        int i = kUnity;
        float a = fabsf(v);
        /* Exactly zero has no magnitude to normalise and would spin the second
         * loop down to "0.000 p"; leave it at unity. */
        if (a != 0.0f) {
            while (a >= 1000.0f && i < kLast) { v /= 1000.0f; a /= 1000.0f; ++i; }
            while (a < 1.0f && i > 0) { v *= 1000.0f; a *= 1000.0f; --i; }
        }
        snprintf(buf, sizeof buf, "%7.3f %s%s", (double) v, kPrefix[i], unit);
    }

    const char *c_str() const { return buf; }
};

#ifndef TARGET_STM32H5
void connect_wifi_async();
void connect_wifi_async_once();
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
