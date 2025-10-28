#pragma once


#include "sampling.h"


class TMP117;

unsigned long micros();

class PowerSampler_TMP117 : public PowerSampler {
    TMP117 *tmp{nullptr};
    unsigned long tLastRead = 0;

public:
    PowerSampler_TMP117(uint8_t addr);

    bool init() override;

    void startReading() override {
    }

    bool hasData() override {
        return micros() - tLastRead > (125 * 8 * 1000); // ct=125ms, avg=8
    }

    Sample getSample() override;

    uint8_t getStorageId() const override {
        return 5;
    }
};
