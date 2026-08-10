#pragma once

#include <Arduino.h>
#include "platform.h"
#include "sampler_registry.h"
#include "ble_transport.h"
#include "aux_switch.h"
#include "util.h"

void handleConsoleInput(const String &buf, SamplerRegistry &registry, BleTransport &ble);

class Telemetry {
protected:
    SamplerRegistry &registry;
    BleTransport &ble;

    unsigned long lastTimeOut = 0;
    unsigned long lastTimePrint = 0;
    int64_t timeLastWakeEvent = 0;

    static constexpr int64_t IDLE_SLEEP_AFTER_US = 3600ll * 1000000ll;
    static constexpr int64_t IDLE_SLEEP_WAKE_US = 900ull * 1000000ll;

    static bool looksActive(float meanPower) {
        if (!std::isfinite(meanPower)) return true;
        return std::abs(meanPower) >= 0.0005f;
    }

    int8_t auxPublished = -1;

    virtual void processConsole() {
        if (Serial.available()) {
#ifdef TARGET_STM32H5
            Serial.setTimeout(10);
#endif
            auto r = Serial.readString();
            Serial.print(r);
            Serial.flush();
            int lb;
            while ((lb = r.indexOf('\n')) != -1) {
                String line = r.substring(0, lb);
                handleConsoleInput(line, registry, ble);
                r = r.substring(lb + 1);
            }
            noteWakeEvent();
        }
    }

    virtual void onIdleSleep() {
        UART_LOG("Zero power for %llds, idle (aux=%s)",
                 (long long)(IDLE_SLEEP_AFTER_US / 1000000), auxGet() ? "ON" : "off");
    }

    virtual void onSummary(const WireSample &ws) { (void)ws; }
    virtual void onTelemetryFlush() {}

public:
    Telemetry(SamplerRegistry &reg, BleTransport &b) : registry(reg), ble(b) {}
    void noteWakeEvent() { timeLastWakeEvent = platform::micros64(); }

    void update(bool checkIdle = true) {
        unsigned long nowTime = micros();

        ble.tick();

        if (auxPublished != (auxGet() ? 1 : 0)) {
            ble.publishAux(auxGet());
            auxPublished = auxGet() ? 1 : 0;
        }

        registry.consumeAllQueues();

        if (nowTime - lastTimeOut > 400e3) {
            auto print = nowTime - lastTimePrint > 2000e3;

            bool anyPowerVote = false, anyActive = false;
            for (auto &ec : registry.counters) {
                if (ec.newSamplesSinceLastSummary()) {
                    if (ec.sampler->measuresPower()) {
                        anyPowerVote = true;
                        if (looksActive(ec.winPrint.P.getMean())) anyActive = true;
                    }

                    bool newSample;
                    auto ws = ec.summary((nowTime - lastTimeOut), print, newSample);
                    if (newSample) {
                        ble.send((uint8_t*)&ws, sizeof(ws));
                        onSummary(ws);
                    }
                }
            }

            const bool active = anyActive || !anyPowerVote;
            if (active) noteWakeEvent();

            ble.flush();
            onTelemetryFlush();

            if (print) {
                if (registry.size() > 1) UART_LOG("");
                lastTimePrint = nowTime;
            }

            lastTimeOut = nowTime;
        }

        processConsole();
        if (checkIdle) checkIdleSleep();
    }

    void checkIdleSleep() {
        if (platform::micros64() - timeLastWakeEvent > IDLE_SLEEP_AFTER_US) {
            if (ble.isConnected()) {
                noteWakeEvent();
            } else {
                onIdleSleep();
                noteWakeEvent();
            }
        }
    }
};
