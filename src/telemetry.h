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
    virtual ~Telemetry() = default;
    void noteWakeEvent() { timeLastWakeEvent = platform::micros64(); }

    void update(bool checkIdle = true) {
        unsigned long nowTime = micros();

        ble.tick();

        if (auxPublished != (auxGet() ? 1 : 0)) {
            ble.publishAux(auxGet());
            auxPublished = auxGet() ? 1 : 0;
        }

        registry.consumeAllQueues();

        /* CATCH-UP DRAIN, every pass (~10 ms), not once per 400 ms telemetry tick.
         *
         * flush() ships at most ONE indication per call -- ATT allows only one outstanding
         * indication per connection -- so calling it only inside the 400 ms block below capped
         * the link at 2.5 indications/s no matter how deep the backlog got. That cap is what
         * turned a transient stall into permanent data loss: a tick produces ~4 records (256 B)
         * and an indication carries 7 (448 B), so the surplus is under 200 B per tick. Once a
         * few seconds of dropped acks had filled the queue, clawing it back needed ten clean
         * ticks in a row, and the bench link never gave ten in a row -- the queue simply stayed
         * pinned full, shedding ~3 samples/s indefinitely (measured over 21 s: 67 drops).
         *
         * Calling flush() every pass costs nothing when there is nothing to do (it returns
         * immediately with an empty buffer or an indication in flight) and lets the backlog
         * drain as fast as acks come back. It cannot fragment a tick's batch into one-sample
         * indications -- the eager-flush regression the "DELIBERATELY NO flush() HERE" comment in
         * BleSrv::send() documents -- because the send()
         * loop below runs to completion within a single update() call, so no flush can
         * interleave between its sends. */
        ble.flush();

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
#if defined(ESP32) && ARDUINO_USB_MODE && defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
        // A plugged USB host keeps the device awake -- deep sleep cannot wake on USB input, so
        // sleeping would strand an attached operator. Sampled here every cycle rather than once
        // at the idle boundary: isPlugged() has documented transient false readings (HWCDC.cpp),
        // and a single misread at the 1-hour mark would otherwise spuriously trip sleep. Treating
        // it as wake activity makes the IDLE_SLEEP_AFTER_US window the debounce.
        if (Serial.isPlugged()) noteWakeEvent();
#endif
        if (checkIdle) checkIdleSleep();
    }

    void checkIdleSleep() {
        if (platform::micros64() - timeLastWakeEvent > IDLE_SLEEP_AFTER_US) {
            // AUX energised is not a wake event but a state that must not be frozen: sleeping
            // with the output on locks it until the next wake, with no way to turn it off. The
            // USB-console case is handled as wake activity in the pump above.
            bool stayAwake = ble.isConnected() || auxGet();
            if (stayAwake) {
                noteWakeEvent();
            } else {
                onIdleSleep();
                noteWakeEvent();
            }
        }
    }
};
