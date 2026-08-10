#pragma once

#include <Arduino.h>
#include <stdio.h>

typedef int esp_err_t;
#define ESP_OK           0
#define ESP_FAIL        -1
#define ESP_ERR_TIMEOUT -2

#define IRAM_ATTR

#ifndef unlikely
#define unlikely(x) (x)
#endif
#ifndef likely
#define likely(x) (x)
#endif

#ifndef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...)  do { Serial.printf("[%s] " fmt "\r\n", tag, ##__VA_ARGS__); } while(0)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...)  do { Serial.printf("[%s] W: " fmt "\r\n", tag, ##__VA_ARGS__); } while(0)
#endif
#ifndef ESP_LOGE
#define ESP_LOGE(tag, fmt, ...)  do { Serial.printf("[%s] E: " fmt "\r\n", tag, ##__VA_ARGS__); } while(0)
#endif

#define ESP_ERROR_CHECK(x) do { auto _r = (x); if (_r != ESP_OK) { Serial.printf("ESP_ERROR_CHECK failed: %d\r\n", _r); } } while(0)

#include <FreeRTOS.h>
#include <task.h>
#include <stm32h5xx_hal.h>

extern "C" void SystemClock_Config(void);

inline int64_t esp_timer_get_time() {
    static uint32_t last_us = 0;
    static uint64_t base = 0;
    taskENTER_CRITICAL();
    uint32_t now = micros();
    if (now < last_us) {
        base += 0x100000000ULL;
    }
    last_us = now;
    uint64_t result = base + now;
    taskEXIT_CRITICAL();
    return (int64_t)result;
}

#ifndef __bswap32
#define __bswap32(x) __builtin_bswap32(x)
#endif
#ifndef __bswap16
#define __bswap16(x) __builtin_bswap16(x)
#endif

#ifndef usleep
#define usleep(us) delayMicroseconds(us)
#endif
#ifndef sleep
#define sleep(s) delay((s) * 1000)
#endif

struct timeval;
inline int gettimeofday(struct timeval *tv, void *) {
    unsigned long long ms = millis();
    tv->tv_sec = (long)(ms / 1000);
    tv->tv_usec = (long)((ms % 1000) * 1000);
    return 0;
}

inline void esp_restart_impl() { NVIC_SystemReset(); }
#define ESP_restart() esp_restart_impl()

struct esp_partition_t;
struct esp_ota_img_states_t;

inline void eeprom_begin(size_t) {}
inline void eeprom_commit() {}
inline void eeprom_end() {}
#define EEPROM_BEGIN(sz) eeprom_begin(sz)
#define EEPROM_COMMIT() eeprom_commit()
#define EEPROM_END() eeprom_end()
