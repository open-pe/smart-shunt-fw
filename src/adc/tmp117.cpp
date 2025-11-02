
#include <Arduino.h>

#include "tmp117.h"

#include <esp32-hal.h>
#include <TMP117.h>

PowerSampler_TMP117::PowerSampler_TMP117(uint8_t addr) {
    tmp = new TMP117(addr);
}

bool PowerSampler_TMP117::init()  {
    tmp->init ( nullptr );
    auto t = tmp->getTemperature();
    ESP_LOGI("tmp117", "Temp: %f", t);
    return (int)t != 0; // TODO nasty hack
}


Sample PowerSampler_TMP117::getSample()  {
    tLastRead = micros();
    Sample sample{};
    auto t = tmp->getTemperature();
    sample.temp = int(t) != 0 ? t : 0; // TODO nasty hack
    sample.setTimeNow();
    return sample;
}