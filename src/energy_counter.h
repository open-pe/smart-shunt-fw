#pragma once

#include <Arduino.h>

#include <utility>

#include "adc/sampling.h"
#include <cmath>
#include "util.h"
#include "settings.h"
#include "mean_window.h"

#include "readerwriterqueue.h"

#ifdef TARGET_STM32H5
#include "esp_compat.h"
#endif

/// Apply the stored per-device voltage gain factor (calibFactorU) to published samples.
///
/// Disabled (0). The factor is keyed to the INA228's I2C address, not to the chip, so it
/// survives a chip swap and a reflash and is applied invisibly downstream -- a stored
/// factor from an earlier campaign reads as a gain shift of the sensor itself. Voltage
/// calibration now lives host-side (calibration/polynom.py in pwr-metering), where it is
/// versioned and visible in the fit.
///
/// Setting this back to 1 re-enables the multiply and makes recorded voltages depend on
/// hidden EEPROM state again; if you do, log the boot-time factors alongside the data.
/// The current-channel factor (calibFactorI) is unaffected and still applied.
#define APPLY_U_GAIN_CAL 0

template<typename T>
struct UIP {
    T U{}, I{}, P{}, Temp{};

    template<typename = void>
    void clear() {
        U.clear();
        I.clear();
        P.clear();
        Temp.clear();
    }
};

#pragma pack(push, 1)
struct WireSample {
    uint8_t v;
    uint8_t idx;
    char dev[16];
    Sample data;
    float i_max, u_max;
    uint32_t diag; // encoded diagnostic from last implausible VBUS reading in this window
    uint16_t crc;

#ifndef TARGET_STM32H5
    Point getInfluxDbPoint() const {
        Point point("smart_shunt");
        point.addTag("device", dev);
        //if (nSamples != NSamplesLastSummary) // TODO
            {
            point.addField("I", data.i, 8);
            point.addField("U", data.u, 8);
            point.addField("I_max", i_max, 3);
            point.addField("U_max", u_max, 3);
            point.addField("P", data.p_, 7);
            point.addField("E", data.e, 4);
            point.addField("T", data.temp, 2);
            /*if (maxDt > maxDtReported) {
                point.addField("dt_max", (float) maxDt * 1e-3f, 2);
                maxDtReported = maxDt;
            }

            if (numTimeouts)
                point.addField("timeouts", numTimeouts); */
        }

        point.setTime(data.t);
        return point;
    }
#endif


    static unsigned short compute_crc16(unsigned char buf[], int len) {
        // Start Definition nach Modbus Standard http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
        // Seite 39ff
        unsigned short crc = 0xFFFF;
        // Durchlaufe jedes Byte (char)
        for (int pos = 0; pos < len; pos++) {
            // XOR das erste Byte des Buffers mit dem low-order Byte des 16bit CRC
            // Registers. Das Ergebnis wird ebenfalls im CRC Register gespeichert
            crc ^= (unsigned short) buf[pos];
            // Loop über jedes Bit
            for (int i = 8; i != 0; i--) {
                // Shifte das CRC Register ein Bit nach rechts (Richtung LSB),
                // und fülle das MSB mit einer Null auf (Step 3)
                if ((crc & 0x001) != 0) {
                    // (Wenn LSB 1 ist)
                    crc >>= 1;
                    // XOR das CRC Register mit dem Polynominal Wert 0xA001 (Step 4)
                    crc ^= 0xA001;
                } else {
                    // (Wenn LSB Null war) Shifte CRC Register ein bit nach rechts (Step 3)
                    crc >>= 1;
                }
            }
        }
        return crc;
    }
};
#pragma pack(pop)


class EnergyCounter {
    moodycamel::ReaderWriterQueue<Sample> sampleQueue{};

public:
    PowerSampler *sampler;
    std::string name;

    /// Cumulative sample count. Read by the OTA rollback confirmation as a liveness signal -- it
    /// only counts as evidence if it is seen to *increase*, since a wedged build still reports
    /// whatever it managed before it stopped.
    unsigned long numSamples() const { return NumSamples; }

private:
    unsigned long NumSamples = 0;
    unsigned long NSamplesLastSummary = 0;
    unsigned long NSamplesLastPrint = 0;
    unsigned long tLastPrint = 0;

    double Energy = 0; //Wh
    float LastP = 0.0f;

    float TotalCharge = 0; //Ah
    float LastI = 0.0f;

    unsigned long startTime = 0;
    unsigned long lastTime = 0;
    unsigned long maxDt = 0;
    unsigned long numTimeouts = 0;
    uint32_t numTimeoutsStreak = 0;

    // Stall detection runs on esp_timer (64-bit, never wraps), separately from the
    // micros()-based lastTime used for the energy integration.
    int64_t lastSampleTime = 0;
    int64_t retryNotBefore = 0;

    static constexpr int64_t STALL_TIMEOUT_US = 4000000;  // no sample for 4 s -> stalled
    static constexpr int64_t BOOT_GRACE_US = 10000000;    // ignore the first 10 s

public:
    UIP<MeanWindow> winPoint{};
    UIP<MeanWindow> winPrint{};


    unsigned long long windowTimestamp = 0;

    float calibFactorU = 1, calibFactorI = 1;

private:
    const uint8_t eePromIndex;

    unsigned long maxDtReported = 0;

    uint32_t lastDiag = 0; // last non-zero diag from samples in this window

public:
    EnergyCounter(PowerSampler *sampler, std::string name_, size_t eePromIndex_) : sampler(sampler),
        name(std::move(name_)),
        eePromIndex(eePromIndex_) {
        if (readCalibrationFactors(eePromIndex_, calibFactorU, calibFactorI)) {
            UART_LOG("%s read calibration factors U/I = (%.8f/%.8f)", name.c_str(), calibFactorU, calibFactorI);
#if !APPLY_U_GAIN_CAL
            // Say so explicitly -- a stored factor that is silently ignored looks exactly
            // like no factor at all, which is the confusion this change exists to end.
            if (calibFactorU != 1)
                UART_LOG("%s: stored U gain factor %.8f is NOT applied (APPLY_U_GAIN_CAL=0), "
                         "voltage is published raw", name.c_str(), calibFactorU);
#endif
        }
    }

    EnergyCounter(const EnergyCounter &) = delete; // no copy
    EnergyCounter(EnergyCounter &&) = default;

    void setCalibrationFactors(float u, float i, bool multiply) {
        if (!std::isnan(u))calibFactorU = (multiply ? calibFactorU : 1) * u;
        if (!std::isnan(i)) calibFactorI = (multiply ? calibFactorI : 1) * i;

        storeCalibrationFactors(eePromIndex, calibFactorU, calibFactorI);
    }

    void update() {
        PowerSampler &ps(*sampler);
        if (ps.hasData()) {
            Sample s = ps.getSample();
            int64_t now64 = esp_timer_get_time();      // 64-bit clock for stall detection
#ifdef TARGET_STM32H5
            unsigned long nowTime = micros();            // true 1us resolution for energy integration
#else
            unsigned long nowTime = (unsigned long) now64; // (micros() is this, truncated)
#endif
            lastSampleTime = now64;
            retryNotBefore = 0;

            // The U gain correction is disabled: it silently multiplied every published
            // voltage, so a stored factor from a previous chip/campaign showed up as an
            // unexplained gain shift in the recorded data (see APPLY_U_GAIN_CAL above).
#if APPLY_U_GAIN_CAL
            s.u *= calibFactorU;
#endif
            s.i *= calibFactorI;

            auto P = s.p();
            if (lastTime != 0) {
                // we use simple trapezoidal rule here
                unsigned long dt_us = nowTime - lastTime;
                Energy += (double) ((LastP + P) * 0.5f * (dt_us * (1e-6f / 3600.f)));
                s.e = (float) Energy;


                TotalCharge += (double) ((LastI + s.i) * 0.5f * (dt_us * (1e-6f / 3600.f)));
                //s.c = (float) TotalCharge;

                if (dt_us > maxDt)
                    maxDt = dt_us;
            } else {
                s.e = 0.0f;
                startTime = nowTime;
            }

            lastTime = nowTime;
            LastP = P;
            LastI = s.i;

            sampleQueue.emplace(s);
            auto qs = sampleQueue.size_approx();
            if (qs > 250 && qs < 355) {
                ESP_LOGW("ec", "Sample queue is growing beyond 250: %u", qs);
            }

            if (numTimeoutsStreak > 0) numTimeoutsStreak = 0;
        } else {
            // Stall detection, on the 64-bit clock. The old test was
            //     nowTime > lt && (nowTime - lt) > 4e6 && nowTime > 10e6
            // on micros(). The unsigned subtraction in the middle is the correct
            // rollover-safe idiom; the `nowTime > lt` in front of it destroyed that.
            // micros() wraps every 71.6 min, so a stall beginning shortly before a wrap
            // leaves nowTime numerically below a stale lt for up to another 71 minutes,
            // and the entire recovery path is disabled for that whole window -- i.e. it
            // switches itself off exactly when a sensor has just died. The `nowTime >
            // 10e6` boot grace re-armed itself on every wrap for the same reason.
            const int64_t now = esp_timer_get_time();
            if (lastSampleTime == 0) lastSampleTime = now; // start the clock on first call
            if (now < BOOT_GRACE_US) return;
            if (now - lastSampleTime <= STALL_TIMEOUT_US) return;
            if (now < retryNotBefore) return;

            ++numTimeouts;
            ++numTimeoutsStreak;
            ESP_LOGW("ec", "%s: no sample for %.1fs (streak %u), re-arming sampler",
                     name.c_str(), (double) (now - lastSampleTime) * 1e-6, numTimeoutsStreak);
            ps.startReading();

            // Back off WITHOUT blocking. This runs in the single realTimeTask that
            // services every counter in turn, so the old delay(min(100,streak)^2) -- up
            // to 10 SECONDS -- starved every healthy sensor on the board because one
            // sensor had died. Now only the dead counter waits.
            const int64_t backoff = std::min<int64_t>(numTimeoutsStreak * 500000ll, 10000000ll);
            retryNotBefore = now + backoff;
        }
    }

    void consumeQueue() {
        // TODO this is a bit inefficient
        // better to create the window summary inside the RT task and publish this
        Sample s{};
        while (sampleQueue.try_dequeue(s)) {
            //ESP_LOGD("ec", "DEQ!");
            auto P = s.p();

            winPoint.I.add(s.i);
            winPoint.U.add(s.u);
            winPoint.P.add(P);
            winPoint.Temp.add(s.temp);

            if (s.diag) lastDiag = s.diag;

            winPrint.I.add(s.i);
            winPrint.U.add(s.u);
            winPrint.P.add(P);
            winPrint.Temp.add(s.temp);

            windowTimestamp = s.t;

            ++NumSamples;
        }
    }

    bool newSamplesSinceLastSummary() const {
        return NumSamples > NSamplesLastSummary;
    }

    Sample printSample{};
    uint8_t wireSampleIdx = 0;

    WireSample summary(unsigned long dt_us, bool print, bool &outNewSamples) {
        // capture
        auto nSamples = NumSamples;
        auto energy = Energy;
        float i_max = winPoint.I.getMax(), u_max = winPoint.U.getMax();
        float i_mean = winPoint.I.pop(), u_mean = winPoint.U.pop(), p_mean = winPoint.P.pop();
        float temp_mean = winPoint.Temp.pop();

        WireSample ws{
            .v = 1,
            .idx = wireSampleIdx++,
            .i_max = i_max,
            .u_max = u_max,
        };
        ws.diag = lastDiag;
        lastDiag = 0;
        name.copy(ws.dev, 16);
        Sample &s(ws.data);
        {
            s.i = i_mean, s.u = u_mean, s.p_ = p_mean, s.e = (float)energy, s.t = windowTimestamp, s.temp = temp_mean;
        }
        ws.crc = WireSample::compute_crc16((uint8_t *) &ws, (uint8_t *) &ws.crc - (uint8_t *) &ws);

        outNewSamples = nSamples != NSamplesLastSummary;

        // client.writePoint(point);

        if (print) {
            auto now = micros();
            // compute
            float sps = (float) (nSamples - NSamplesLastPrint) / ((float) (now - tLastPrint) * 1e-6f);

            printSample.u = winPrint.U.pop();
            printSample.i = winPrint.I.pop();
            printSample.e = (float) energy;
            UART_LOG("%s %s U=%7.4fV I=%7.4fA P=%6.3fW, E=%6.3fWh, T=%2.1f° N=%lu, sps=%.1f, maxDt=%.2fms",
                     timeStr().c_str(), name.c_str(), printSample.u, printSample.i, winPrint.P.pop(), energy,
                     winPrint.Temp.pop(), nSamples, sps, maxDt * 1e-3f);

            // Serial0.print(", T=");
            // Serial0.print((nowTime - startTime) * 1e-6, 1);
            // Serial0.print("s");
            // Serial0.print(", numDropped=");
            // Serial0.print(numDropped);
            // Serial0.println();

            tLastPrint = now;
            NSamplesLastPrint = nSamples;
        }

        NSamplesLastSummary = nSamples;

        return ws;
    }

    void reset() {
        NumSamples = 0;
        NSamplesLastSummary = 0;

        Energy = 0;
        LastP = 0.0f;

        startTime = 0;
        lastTime = 0;
        maxDt = 0;
        numTimeouts = 0;

        winPoint.clear();
        winPrint.clear();
    }
};
