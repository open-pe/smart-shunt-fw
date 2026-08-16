#pragma once

#include <Arduino.h>

#include <utility>

#include "adc/sampling.h"
#include <cmath>
#include <cstring>   // memcpy/strstr in getInfluxDbPoint()
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
        /* `dev` is a FIXED 16 bytes filled by name.copy(), which does NOT append a
         * terminator. A name of exactly 16 characters or more therefore fills the
         * field with no NUL, and every C string function walks straight on into
         * i_max. "BLE_SHUNT_ADC_ZERO" is 18 characters and truncates to exactly
         * "BLE_SHUNT_ADC_ZE" -- precisely that case. Copy into a terminated
         * buffer once and use it for both the tag and the matching below. */
        char devz[sizeof(dev) + 1];
        memcpy(devz, dev, sizeof(dev));
        devz[sizeof(dev)] = '\0';

        Point point("smart_shunt");
        point.addTag("device", devz);
        //if (nSamples != NSamplesLastSummary) // TODO
            {
            /* 12 decimal places, not the original 8. This is the resolution floor
             * of every published series: 8 places quantises a volt to 10 nV, which
             * is the SAME ORDER as the ADS1262 offset drift the shunt-adc channel
             * exists to characterise -- the effect was the quantisation step.
             *
             * 12 is where it stops being worth going further, not an arbitrary
             * pick: Sample::u is a float, and float32 carries ~7 significant
             * digits, so a 7 uV reading is already only good to ~0.4 pV. Past 12
             * places the extra digits are float noise, not measurement.
             *
             * For volt-scale channels the trailing digits are meaningless for the
             * same reason -- harmless, but do not read them as precision. Cost is
             * a few characters per field on the wire. */
            point.addField("I", data.i, 12);
            point.addField("U", data.u, 12);
            point.addField("I_max", i_max, 3);
            point.addField("U_max", u_max, 3);
            point.addField("P", data.p_, 7);
            point.addField("E", data.e, 4);
            point.addField("T", data.temp, 2);
            /* The DIAGNOSTIC, which this path dropped while the BLE collector has
             * always published it -- so the two routes to the same database
             * disagreed about whether a fault had been reported at all.
             *
             * It also rescues the degenerate point. A sample that says "not
             * measured, and here is why" carries NaN in every measurement field,
             * addField drops NaN silently, and the result was a measurement, a
             * tag and a timestamp with NO FIELDS -- which InfluxDB rejects with a
             * 400. The one case where the point matters most was the one case it
             * could not be written. `diag` is an integer and always present, so
             * a fault sample now always has at least one field.
             *
             * Written unconditionally rather than only when non-zero: a field
             * that appears only during faults makes "no diag field" ambiguous
             * between healthy and not-reported. */
            point.addField("diag", (int) diag);
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

    /// Has ANY finite interval gone into Energy? Until one has, E is unmeasured, not zero.
    /// Energy's initial 0 is otherwise indistinguishable from a real total of zero, which is
    /// exactly the "absence of evidence encoded as absence of the quantity" this file is
    /// fixing elsewhere. A current-only INA228 never sets this, and so never publishes E.
    bool energyValid = false;

    /// Intervals left out of the E integral because an endpoint was not finite, and how much
    /// wall time they add up to. Kept so a total with a hole in it can be told from a complete
    /// one -- the alternative to poisoning E with NaN is omitting time, and that has to be
    /// visible rather than merely quieter.
    ///
    /// Both 32-bit deliberately: they are written from the real-time task and cleared by
    /// reset() from the app task, and an aligned 32-bit store cannot tear on these MCUs the
    /// way a 64-bit one can. Milliseconds, so the range is still ~49 days of unmeasured time.
    uint32_t nUnmeasured = 0;
    uint32_t unmeasuredMs = 0;

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
                const float dt_h = dt_us * (1e-6f / 3600.f);

                /* A trapezoid with a non-finite endpoint is not a small error, it is a
                 * PERMANENT one: `Energy += NaN` leaves Energy NaN for the life of the
                 * counter, and only reset() clears it. Two ways that bit us:
                 *
                 *  - A temperature-only sampler has no P at all (p() = u*i = NaN*NaN), so
                 *    Energy went NaN on its second sample and E was missing from every
                 *    frame it ever sent. Combined with a TMP117 whose I2C read had failed
                 *    -- temp NAN too -- the summary carried NO finite field whatsoever,
                 *    and the collector emitted line protocol with an empty field section
                 *    that InfluxDB rejected outright ("invalid field format").
                 *  - On a real power sampler, ONE failed conversion silently destroyed the
                 *    Wh total from then on. Nothing said so; the counter just stopped
                 *    reporting energy.
                 *
                 * Skipping the interval is the honest option. The energy that flowed while
                 * we could not measure is unknown, and carrying the running total forward
                 * says exactly that -- where NaN throws away everything correctly measured
                 * BEFORE the dropout as well. Skipped time is counted, not swallowed, so a
                 * total quietly missing a chunk of its integration window can be seen.
                 *
                 * energyValid is what keeps the skip from becoming the very lie this file
                 * is fixing. Energy starts at 0, so "every interval was skipped" and "the
                 * load drew nothing" would otherwise both publish E=0. That is not
                 * hypothetical: the INA228 boards default to CURRENT-ONLY (channelState
                 * .vbus=false, ina228.h), so u is NAN, p() is NAN, and every interval they
                 * ever integrate is skipped. Publishing E=0 for them would invent a
                 * measurement out of a channel that cannot produce one. E stays NAN until
                 * at least one finite interval has actually gone into it.
                 *
                 * NOTHING IS LOGGED HERE. This runs in the real-time sampling task, and a
                 * channel with no voltage hits this branch on EVERY sample -- at the
                 * INA228's ~120 Hz a rate-limited line still means a UART write every few
                 * seconds, forever, in the hot path. The counters are reported from
                 * summary()'s print instead, which runs on the app task. */
                const float e_step = (LastP + P) * 0.5f * dt_h;
                if (std::isfinite(e_step)) {
                    Energy += (double) e_step;
                    energyValid = true;
                } else {
                    ++nUnmeasured;
                    unmeasuredMs += (uint32_t) (dt_us / 1000u);
                }
                s.e = energyValid ? (float) Energy : NAN;

                const float q_step = (LastI + s.i) * 0.5f * dt_h;
                if (std::isfinite(q_step)) TotalCharge += (double) q_step;
                //s.c = (float) TotalCharge;

                if (dt_us > maxDt)
                    maxDt = dt_us;
            } else {
                // First sample: there is no preceding interval, so no energy has been
                // integrated. NAN, not 0.0f -- the old 0 asserted a measured zero before
                // anything had been measured at all.
                s.e = NAN;
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
            /* Per-sampler, not the bare constant: a diagnostic channel that
             * publishes every 30 s is not stalled at 4 s, and reporting it as
             * such buries real stalls in false warnings. 0 opts out entirely. */
            const int64_t stallTimeout = ps.stallTimeoutUs();
            if (stallTimeout <= 0) return;
            if (now - lastSampleTime <= stallTimeout) return;
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
        // NAN, not the raw 0, until a finite interval has actually been integrated -- see
        // energyValid. A current-only INA228 has no power channel at all and must publish no
        // energy, rather than a zero that reads like a measured one.
        const float energy = energyValid ? (float) Energy : NAN;
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
            /* SI-prefixed, so this one line serves both a 1.2 kW load and the
             * shunt-adc zero channel's 7 uV -- the old "%7.4fV" rendered the
             * latter as a flat 0.0000 and made the console useless for it. */
            UART_LOG("%s %s U=%s I=%s P=%s, E=%s, T=%2.1f° N=%lu, sps=%.1f, maxDt=%.2fms",
                     timeStr().c_str(), name.c_str(),
                     SiFmt(printSample.u, "V").c_str(),
                     SiFmt(printSample.i, "A").c_str(),
                     SiFmt(winPrint.P.pop(), "W").c_str(),
                     SiFmt((float) energy, "Wh").c_str(),
                     winPrint.Temp.pop(), nSamples, sps, maxDt * 1e-3f);

            /* Report the hole in E here, on the APP task, rather than from the sampling
             * loop that produces it: a channel with no voltage skips every interval, so a
             * rate-limited warning down there would be a UART write every few seconds
             * forever, in the real-time path. Printed only when there is something to say,
             * and only when E is actually a number -- an unmeasured E is already NAN on the
             * wire and does not need explaining as a gap as well. */
            if (nUnmeasured && energyValid)
                UART_LOG("%s   E excludes %lu interval(s), %.1f s of unmeasurable power",
                         name.c_str(), (unsigned long) nUnmeasured, unmeasuredMs * 1e-3f);

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
        energyValid = false;
        LastP = 0.0f;
        nUnmeasured = 0;
        unmeasuredMs = 0;

        // Cleared with Energy, not left running. They are the other half of the same
        // integration and a `reset` that zeroed one while the other kept accumulating from
        // before the reset would make the pair silently inconsistent.
        TotalCharge = 0.0f;
        LastI = 0.0f;

        startTime = 0;
        lastTime = 0;
        maxDt = 0;
        numTimeouts = 0;

        winPoint.clear();
        winPrint.clear();
    }
};
