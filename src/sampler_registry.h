#pragma once

#include "adc/sampling.h"
#include "board_prefix.h"
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
        /* The board prefix turns a name that is unique within this firmware into one
         * unique across every board reporting into the same database. Checked over
         * ALL registered names -- including the ones whose sampler fails to init --
         * because a collision is a property of the naming scheme, not of which parts
         * happened to answer on this boot, and a part that is missing today will be
         * back tomorrow. */
        std::vector<std::string> composed;
        composed.reserve(entries.size());
        counters.reserve(entries.size()); // never reallocated after boot
        for (auto &e : entries) composed.push_back(boardPrefixApply(e.name));
        boardPrefixCheckNames(composed);

        size_t failed = 0;
        for (size_t i = 0; i < entries.size(); ++i) {
            auto &e = entries[i];
            if (!e.sampler->init()) {
                ++failed;
                if (e.sampler->hotPluggable()) {
                    /* Kept, silent until its part answers: the sampler re-probes from
                     * hasData(). Created HERE, never later -- see hotPluggable(). */
                    counters.emplace_back(EnergyCounter{e.sampler, composed[i], e.sampler->getStorageId()});
                    ESP_LOGI("sampler", "%s: init failed, will re-probe (hot-plug)", composed[i].c_str());
                } else {
                    ESP_LOGI("sampler", "%s: init failed", composed[i].c_str());
                }
            } else {
                counters.emplace_back(EnergyCounter{e.sampler, composed[i], e.sampler->getStorageId()});
                ESP_LOGI("sampler", "Initialized energy counter for %s", composed[i].c_str());
            }
        }
        /* T offsets are keyed by storageId (temp_offset.h). Two offset-taking samplers on
         * one id would read and write the SAME stored correction -- the slot-aliasing bug
         * tmp117.h and main_esp32.cpp document for U/I, one namespace over. Fail closed:
         * neither gets an offset until the ids are made distinct. */
        for (size_t a = 0; a < counters.size(); ++a) {
            if (!counters[a].sampler->acceptsTempOffset()) continue;
            for (size_t b = a + 1; b < counters.size(); ++b) {
                if (!counters[b].sampler->acceptsTempOffset()) continue;
                if (counters[a].storageId() != counters[b].storageId()) continue;
                ESP_LOGE("sampler", "%s and %s share storageId %u: T offset DISABLED for both",
                         counters[a].name.c_str(), counters[b].name.c_str(),
                         (unsigned) counters[a].storageId());
                counters[a].tempOffsetC = 0;
                counters[b].tempOffsetC = 0;
                counters[a].tempOffsetBlocked = true;
                counters[b].tempOffsetBlocked = true;
                counters[a].tempOffsetStatus = temp_offset::Status::Blocked;
                counters[b].tempOffsetStatus = temp_offset::Status::Blocked;
            }
        }
        /* Scan on ANY failure, not only on a total wipeout. The old `counters.empty()`
         * gate meant a board with one working SPI sampler stayed silent while every
         * I2C part on it failed -- exactly the N16R8 clone case, where four TMP117s
         * failed and the ADS1262 counters kept the condition invisible. A sampler
         * that did not come up is worth a bus dump even if its neighbours did. */
        if (failed) {
            ESP_LOGW("sampler", "%u of %u samplers failed to init, scanning I2C",
                     (unsigned) failed, (unsigned) entries.size());
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
