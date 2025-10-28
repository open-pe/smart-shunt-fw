
#include <Arduino.h>

#include "tmp117.h"

#include <esp32-hal.h>
#include <TMP117.h>

PowerSampler_TMP117::PowerSampler_TMP117(uint8_t addr) {
    tmp = new TMP117(addr);
}

bool PowerSampler_TMP117::init()  {
    tmp->init ( nullptr );
    return true;
}


Sample PowerSampler_TMP117::getSample()  {
    tLastRead = micros();
    Sample sample{};
    sample.temp = tmp->getTemperature();
    sample.setTimeNow();
    return sample;
}