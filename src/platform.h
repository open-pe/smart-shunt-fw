#pragma once

#include <Arduino.h>
#include <EEPROM.h>

#ifdef TARGET_STM32H5
#include "esp_compat.h"
#else
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#endif

namespace platform {

inline int64_t micros64() {
    return esp_timer_get_time();
}

inline void restart() {
#ifdef TARGET_STM32H5
    NVIC_SystemReset();
#else
    ESP.restart();
#endif
}

inline void gpioHoldEn(int pin) {
#ifdef TARGET_STM32H5
    (void)pin;
#else
    gpio_hold_en((gpio_num_t)pin);
    gpio_deep_sleep_hold_en();
#endif
}

inline void gpioHoldDis(int pin) {
#ifdef TARGET_STM32H5
    (void)pin;
#else
    gpio_hold_dis((gpio_num_t)pin);
#endif
}

inline void deepSleep(uint64_t us) {
#ifdef TARGET_STM32H5
    (void)us;
#else
    ESP.deepSleep(us);
#endif
}

inline void eepromGet(int idx, float &val) {
#ifdef TARGET_STM32H5
    EEPROM.get(idx, val);
#else
    EEPROM.begin(256);
    val = EEPROM.readFloat(idx);
    EEPROM.end();
#endif
}

inline void eepromPut(int idx, float val) {
#ifdef TARGET_STM32H5
    EEPROM.put(idx, val);
#else
    EEPROM.begin(256);
    EEPROM.writeFloat(idx, val);
    EEPROM.commit();
    EEPROM.end();
#endif
}

inline void eepromRead(int idx, uint8_t &val) {
#ifdef TARGET_STM32H5
    val = EEPROM.read(idx);
#else
    EEPROM.begin(256);
    val = EEPROM.read(idx);
    EEPROM.end();
#endif
}

inline void eepromWrite(int idx, uint8_t val) {
#ifdef TARGET_STM32H5
    EEPROM.write(idx, val);
#else
    EEPROM.begin(256);
    EEPROM.write(idx, val);
    EEPROM.commit();
    EEPROM.end();
#endif
}

inline void createRealTimeTask(void (*fn)(void*), const char *name, uint32_t stack, int prio) {
#ifdef TARGET_STM32H5
    xTaskCreate(fn, name, stack, NULL, prio, NULL);
#else
    xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, NULL, 1);
#endif
}

inline void createAppTask(void (*fn)(void*), const char *name, uint32_t stack, int prio) {
#ifdef TARGET_STM32H5
    xTaskCreate(fn, name, stack, NULL, prio, NULL);
#else
    xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, NULL, 0);
#endif
}

}
