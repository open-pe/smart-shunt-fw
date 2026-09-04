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

/* CONSOLE OUTPUT TEE.
 *
 * UART_LOG is what every console command answers with, and until now it went to
 * the serial port and nowhere else. A command arriving over any other transport
 * would therefore execute and reply into a port nobody is reading -- and silence
 * is indistinguishable from success, which is the worst possible failure mode for
 * a remote `calibrate`.
 *
 * So a transport that wants the reply installs a sink for the duration of ONE
 * command and takes it down again. The serial print always happens as well: the
 * UART log is the only view when the other transport is the thing misbehaving.
 *
 * NOT AN esp_log HOOK, deliberately. Mirroring esp_log_set_vprintf would also
 * capture every other task's log lines into whichever reply happened to be open,
 * which is why the console path uses UART_LOG for its responses (including its
 * error responses -- see the note in console.h).
 *
 * Set and cleared on ONE task, the same task that runs the command. There is no
 * lock here and none is wanted: a sink installed from another task would race the
 * command that is already writing through it.
 *
 * WHICH MAKES THIS A RULE ABOUT UART_LOG ITSELF, not just about the sink: ONCE THE
 * TASKS ARE RUNNING, UART_LOG MAY ONLY BE CALLED FROM THE APP TASK. Setup-time calls
 * are fine and there is one -- EnergyCounter's constructor prints stored calibration
 * from SamplerRegistry::initAll(), on the Arduino setup task, before any other task
 * exists and while the sink is null. An earlier version of this comment said "every
 * call site", which was literally false. Every RUNTIME call site obeys it (the
 * energy-counter and telemetry prints all run inside Telemetry::update(), the same
 * chain that runs the console); the sampling and WiFi tasks use ESP_LOGx.
 *
 * A future UART_LOG from realTimeTask would splice an unrelated line into whichever
 * console reply happens to be open -- and would race the shared static format buffer
 * in the implementation, which has no lock either. Use ESP_LOGx from other tasks.
 */
using ConsoleSink = void (*)(const char *text, size_t len);
void consoleSetSink(ConsoleSink sink);
ConsoleSink consoleGetSink();

/* Push whatever the sink has buffered, now, from inside a long-running command.
 * Installed by a transport whose output is queued and drained from its own loop
 * (BLE), so a command that blocks for seconds does not hold its entire reply
 * until it returns. Null for synchronous transports; consoleFlush() is then a
 * no-op, so a command may call it unconditionally. Lifted from
 * fugu-mppt-firmware's consoleFlushHook, which exists for the same reason. */
extern void (*g_consoleFlushHook)();
void consoleFlush();

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
