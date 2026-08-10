#pragma once

#include "adc/sampling.h"
#include "energy_counter.h"
#include "util.h"
#include <vector>

struct SamplerRegistry {
    struct Entry {
        const char *name;
        PowerSampler *sampler;
    };

    std::vector<EnergyCounter> counters;
    std::vector<Entry> entries;

    void add(const char *name, PowerSampler *sampler) {
        entries.push_back({name, sampler});
    }

    void initAll() {
        for (auto &e : entries) {
            if (!e.sampler->init()) {
                ESP_LOGI("sampler", "%s: init failed", e.name);
            } else {
                counters.emplace_back(EnergyCounter{e.sampler, e.name, e.sampler->getStorageId()});
                ESP_LOGI("sampler", "Initialized energy counter for %s", e.name);
            }
        }
        if (counters.empty()) {
            scan_i2c();
        }
    }

    void startAll() {
        for (auto &ec : counters) {
            ec.sampler->startReading();
        }
    }

    void updateAll() {
        for (auto &ec : counters) {
            ec.update();
        }
    }

    void consumeAllQueues() {
        for (auto &ec : counters) {
            ec.consumeQueue();
        }
    }

    void resetAll() {
        for (auto &ec : counters) {
            ec.reset();
        }
    }

    bool empty() const { return counters.empty(); }
    size_t size() const { return counters.size(); }

    EnergyCounter *findByName(const std::string &name) {
        for (auto &ec : counters) {
            if (ec.name == name) return &ec;
        }
        return nullptr;
    }
};
