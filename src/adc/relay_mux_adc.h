#pragma once

#include <atomic>
#include <cmath>
#include "ads1262.h"
#include "sampling.h"
#include "../relay_mux.h"

/*
 * Two ADS1262 sampler facades time-sharing ONE input pair through the Coto relay
 * mux (see ../relay_mux.h for the board and its switching contract).
 *
 * This is the same shape as INA228MuxBackend / PowerSampler_MuxChannel: a backend
 * owns the physical resource and the round-robin, and each channel is a thin facade
 * that publishes its own series. Two differences matter, and both come from the
 * relay being upstream of the ADC rather than inside it:
 *
 * 1. THE ADC CANNOT SEE THE SWITCH. Ads1262ShuntAdc::selectPair() restarts the
 *    conversion and discards until settled because it moved the chip's own INPMUX.
 *    A relay changing what is connected to a pair changes nothing the ADS1262 knows
 *    about, so it keeps converting straight through the contact bounce and the
 *    divider's RC tail and hands those conversions over as ordinary readings. The
 *    discard is therefore ours to do, explicitly, and its absence would not look
 *    like a fault -- it would look like a slightly noisy channel.
 *
 * 2. SWITCHING IS SLOW. ~300 ms of mandated dead time and settle per change, versus
 *    the tens of microseconds an analog mux costs. Every conversion taken during
 *    that window belongs to no channel at all.
 *
 * FIELD MAPPING (differs per channel -- see the note in CLAUDE.md):
 *   U = input voltage in volts (ADC volts * dividerRatio), or raw ADC volts when
 *       dividerRatio is unset
 *   I = NAN always -- there is no current channel through a voltage mux
 *   P = 0 always -- pinned, not computed, exactly as the single-pair shunt case
 *       does, so nothing downstream reports a plausible wattage for a pair of
 *       quantities that were never measured
 */
class RelayMuxAdcBackend {
public:
    using Target = RelayMux2::Target;

private:
    RelayMux2 &mux;
    Ads1262ShuntAdc &dev;
    const Ads1262ShuntAdc::Pair pair;
    const uint8_t pgaGainCode;
    const bool pgaBypass;

    /* Completed ADC scans to throw away after the contact settles, ON TOP OF
     * waiting for the scan in progress to finish. The first boundary after settle
     * can close a scan that began while the contact was still moving, so one full
     * clean scan is the minimum that contains no pre-switch conversion; a second is
     * margin. It is NOT filter settling: this device runs the FIR filter, which
     * SBAS661C sec.9.4.2 documents as zero latency, giving the result in a single
     * cycle -- which is why Ads1262ShuntAdc discards ZERO after its own mux change.
     * The discard here exists for the analog step the ADC cannot see. Cheap -- the relay's
     * 300 ms already dominates -- and the failure it prevents is silent.
     *
     * At one registered pair the device is parked at 19.15 SPS, so this costs about
     * 3 * 52 ms.
     *
     * IT IS NOT THE PRIMARY DEFENCE, AND IT CANNOT BE. The guarantee that a
     * published conversion is uncontaminated rests entirely on SETTLE_MS being
     * longer than the real contact-close-plus-settle time, and that time is
     * unqualified -- DESIGN.md derives only the FASTEST delay-on corner (15.95 ms)
     * and lists contact release, operate and bounce as qualification items. The
     * "~16-65 ms to close" figure used to size SETTLE_MS is an ESTIMATE, not a
     * bound: a slow or leaky delay network, late operation or prolonged bounce
     * would be declared settled and publish a plausible wrong ratio, and no amount
     * of scan discarding downstream would catch it. The discards cover the ADC-side
     * corners only. Qualify SETTLE_MS on a scope before trusting the numbers. */
    static constexpr uint32_t DISCARD_SCANS = 2;

    Target serving = Target::CH_A;   ///< whose turn it is
    uint32_t genAtSettle = 0;
    uint32_t muxGenSeen = 0;
    bool armedForCapture = false;
    bool initialised = false;

    /* Bench override. DESIGN.md's open gates require someone to observe all four
     * contacts through supply, temperature and asynchronous-command tests, and the
     * round-robin makes that impossible -- it moves the contact every ~600 ms, so
     * there is nothing steady to put a scope on. Manual mode parks one channel and
     * stops publishing, because a sample taken while a human is driving the mux by
     * hand has no defensible channel identity. */
    bool manual = false;

    /// Written by the console on appTask, applied by the RT task in hasDataFor().
    /// Same reasoning as RelayMux2's command inbox: setManual(false) used to call
    /// mux.select() directly, which is a state-machine mutation from the wrong
    /// task. One aligned bool crossing the boundary, and the transition itself
    /// happens where every other mutation happens.
    std::atomic<bool> manualReq{false};

    /// Retained from init() so the reset-recovery path below can bring the shared
    /// ADC back up without asking a channel facade for them.
    int8_t pSck = -1, pMosi = -1, pMiso = -1, pCs = -1, pStart = -1;
    uint32_t lastReinitMs = 0;
    static constexpr uint32_t REINIT_INTERVAL_MS = 2000;

    float lastVolts = NAN;

    friend class PowerSampler_RelayMuxChannel;

    static Target other(Target t) { return t == Target::CH_A ? Target::CH_B : Target::CH_A; }

public:
    /* FRONT-END GAIN. The device default is G=32 (PGA_GAIN_CODE = 5), whose full
     * scale is +-2.5 V / 32 = +-78 mV, because every other consumer of this ADC is
     * a millivolt-scale shunt. A relay mux fed through the divider board is NOT:
     * an 80 V input through the 51:1 divider arrives as ~1.57 V. At G=32 that
     * over-ranges on every conversion, and the driver's own over-range path would
     * dutifully publish NAN with DIAG_PGA_RANGE forever -- a channel that looks
     * wired-up and never produces a number.
     *
     * So the default here is G=1 with the PGA BYPASSED, the same configuration the
     * DCCT burden channel uses and for the same reason: at G=1 the PGA's own output
     * swing leaves each input pin roughly -2.2..+2.2 V on these +-2.5 V rails,
     * which a ground-referenced divider output can exceed. Bypassed, full scale is
     * the reference itself, +-2.5 V. */
    RelayMuxAdcBackend(RelayMux2 &mux, Ads1262ShuntAdc &dev, Ads1262ShuntAdc::Pair pair,
                       uint8_t pgaGainCode = 0, bool pgaBypass = true)
        : mux(mux), dev(dev), pair(pair), pgaGainCode(pgaGainCode), pgaBypass(pgaBypass) {}

    Ads1262ShuntAdc::Pair adcPair() const { return pair; }

    bool init(int8_t pinSck, int8_t pinMosi, int8_t pinMiso, int8_t pinCs, int8_t pinStart) {
        if (initialised) return true;   // idempotent: whichever channel gets here first
        if (pair >= Ads1262ShuntAdc::PAIR_COUNT) {
            ESP_LOGE("relaymux", "pair %u is not a real ADC pair (0..%u); refusing to start",
                     (unsigned) pair, (unsigned) Ads1262ShuntAdc::PAIR_COUNT - 1);
            return false;
        }
        if (!mux.begin()) return false;
        if (!dev.init(pinSck, pinMosi, pinMiso, pinCs, pinStart)) return false;
        pSck = pinSck; pMosi = pinMosi; pMiso = pinMiso; pCs = pinCs; pStart = pinStart;
        /* Gain BEFORE registerPair(): registerPair() re-selects the pair, which is
         * what programs MODE2, so a setPairGain() after it would not take effect
         * until the next selection. Same ordering PowerSampler_ShuntAdc uses. */
        dev.setPairGain(pair, pgaGainCode, pgaBypass);
        dev.registerPair(pair);
        serving = Target::CH_A;
        mux.select(serving);
        muxGenSeen = mux.generation();
        armedForCapture = false;
        initialised = true;
        return true;
    }

    /// Re-drive the sequence after a stall. Idempotent by way of RelayMux2::select(),
    /// which ignores a re-request for the channel it is already settled on -- so a
    /// repeated re-arm cannot hold the contact permanently in its settling window
    /// and starve the very channel it is trying to recover.
    void reArm() {
        if (!initialised) return;
        /* NEVER override manual parking. Manual mode publishes nothing on purpose,
         * so EnergyCounter sees no samples, declares the channel stalled after
         * stallTimeoutUs() and calls startReading() -- which lands here. Without
         * this guard, `relaymux off` re-energised a relay a few seconds later and a
         * parked A/B target silently jumped back to the round-robin's `serving`.
         * The fixture would move under the scope, mid-measurement, with nothing on
         * the console to say why -- destroying the exact qualification run the mode
         * exists to enable. stallTimeoutUs() also returns 0 while parked so the
         * detector does not fire in the first place; this is the backstop. */
        if (manual) return;
        mux.select(serving);
        armedForCapture = false;
    }

    /// Park the mux under console control and stop publishing, or hand it back.
    /// Callable from any task; the change lands on the next RT pass.
    void setManual(bool m) { manualReq.store(m, std::memory_order_release); }

    float dieTempC() const { return dev.dieTempC(); }

    bool isManual() const { return manual; }
    RelayMux2 &muxRef() const { return mux; }
    Target servingChannel() const { return serving; }

    /*
     * The RT loop calls hasData() on EVERY registered sampler each pass, so the
     * channel that is not being served is asked thousands of times per switch. It
     * must answer immediately and touch nothing: no SPI, no state, no clock. The
     * INA228 mux backend learned this the hard way (the "M1 fix" in the plan that
     * produced it) -- a non-target channel that reads shared state steals it.
     */
    bool hasDataFor(Target ch) {
        if (!initialised) return false;
        if (ch != serving) return false;

        /* Apply a console-requested mode change here, on the RT task, rather than
         * in setManual(). Resuming has to re-drive select(), and select() may only
         * run on the task that calls tick(). */
        const bool wantManual = manualReq.load(std::memory_order_acquire);
        if (wantManual != manual) {
            manual = wantManual;
            armedForCapture = false;
            if (!manual) {
                /* Discard any request still sitting in the mux inbox before
                 * resuming. `relaymux b` posts the mode and the target as two
                 * separate writes; a fast `relaymux auto` behind it could be
                 * applied first, and tick() would then consume the stale manual
                 * target AFTER the round-robin resumed. `serving` and the settled
                 * target would disagree, every capture would be rejected, and both
                 * channels would stall until EnergyCounter's stall path repaired it
                 * seconds later -- with the console having already said "round-robin
                 * resumed". Authority over the mux is changing hands here, so a
                 * request posted under the old owner is void by definition. */
                mux.clearInbox();
                mux.select(serving);   // resume where the round-robin left off
            }
        }

        /* Manual mode still ticks the state machine -- the console command needs
         * the sequence to actually run -- but publishes nothing. */
        if (manual) {
            mux.tick();
            /* Keep pumping. If these channels are the only facades on this ADC,
             * dropping pump() would park the converter for the whole bench
             * session and the first sample after `relaymux auto` would come from
             * a device that had stopped scanning. */
            dev.pump();
            armedForCapture = false;
            return false;
        }

        /* RECOVER FROM AN ADC RESET, exactly as PowerSampler_ShuntAdc does.
         *
         * Without this the failure is silent and permanent, and it is the worst
         * shape available: after a device reset pump() short-circuits, so
         * generation() stops advancing while volts_ keeps its last pre-reset
         * value. The discard test below then never passes, this channel publishes
         * nothing, EnergyCounter calls it stalled -- and startReading() cannot fix
         * it, because re-arming the RELAY does nothing for a dead ADC. The channel
         * stays dark for the rest of the process.
         *
         * registerPair() is idempotent and the scan set is driver state that
         * survives the device reset, so re-registering is both safe and necessary:
         * init() reprograms the hardware to PAIR_CH0 without consulting the set.
         * Rate-limited so a genuinely absent board does not spin on SPI. */
        if (dev.needsReinit()) {
            const uint32_t now = millis();
            if ((uint32_t) (now - lastReinitMs) < REINIT_INTERVAL_MS) return false;
            lastReinitMs = now;
            ESP_LOGW("relaymux", "re-initialising ADS1262 after device reset");
            if (!dev.init(pSck, pMosi, pMiso, pCs, pStart)) return false;
            /* init() reprograms the front end to the device default G=32. Without
             * re-applying the gain the channel would come back from a reset
             * permanently over-ranged -- recovered in name only. */
            dev.setPairGain(pair, pgaGainCode, pgaBypass);
            dev.registerPair(pair);
            /* The relay is untouched by an ADC reset, but the capture baseline
             * was taken against a generation counter that has just restarted.
             * Drop it and re-arm against the new one. */
            armedForCapture = false;
        }

        mux.tick();

        /* Not settled: nothing to do but keep the ADC scan alive. pump() must
         * still run or the device stops producing, and the discard bookkeeping
         * below needs a generation counter that is actually advancing. */
        dev.pump();

        if (!mux.isSettled() || mux.settledTarget() != ch) {
            armedForCapture = false;
            return false;
        }

        /* First pass after this contact closed: latch the scan counter. Keyed on
         * the mux's generation, not on a bool alone, so a switch away and back to
         * the same channel re-arms rather than reusing the previous close's
         * baseline -- which would let the first conversion after the new close
         * pass the discard test it never actually satisfied. */
        if (!armedForCapture || muxGenSeen != mux.generation()) {
            muxGenSeen = mux.generation();
            genAtSettle = dev.generation();
            armedForCapture = true;
            return false;
        }

        if ((uint32_t) (dev.generation() - genAtSettle) <= DISCARD_SCANS) return false;

        lastVolts = dev.volts(pair);
        return true;
    }

    /// Consume the reading and hand the mux to the other channel. Called only by
    /// the serving channel's getSample(), mirroring the INA228 backend's advance.
    float take(Target ch, uint32_t &diagOut) {
        diagOut = dev.diag();
        const float v = lastVolts;
        lastVolts = NAN;
        armedForCapture = false;
        serving = other(ch);
        mux.select(serving);
        return v;
    }

    /// Round-trip time for one channel: both channels switch and settle, and each
    /// waits out its discarded scans. Used to size the stall timeout so the two
    /// cannot drift apart.
    /* Pessimistic time for one completed ADC scan, i.e. one generation() step.
     * This build registers exactly ONE pair, so the device parks its internal mux
     * and free-runs at 19.15 SPS -- ~52 ms per generation, or ~104 ms chopped.
     * 120 ms carries margin over the chopped case without being the ~250 ms a
     * multi-pair scan would cost, which is a configuration this build does not
     * create. Only used to size the stall timeout, so erring high is safe (a lazier
     * stall detector) while erring low is not (false stalls on a healthy channel). */
    static constexpr uint32_t SCAN_MS_PESSIMISTIC = 120;

    static constexpr uint32_t cycleMsEstimate() {
        // both channels switch, settle, and wait out their discarded scans
        return 2 * (RelayMux2::switchLatencyMs() + (DISCARD_SCANS + 1) * SCAN_MS_PESSIMISTIC);
    }
};

class PowerSampler_RelayMuxChannel : public PowerSampler {
    RelayMuxAdcBackend &backend;
    const RelayMux2::Target channel;
    const uint8_t storageId;

    /// ADC volts -> input volts. Unset (0) publishes RAW ADC VOLTS rather than a
    /// guessed scale, the same contract PowerSampler_ShuntAdc uses: the divider
    /// lives on a separate board and its ratio is a bench fact, so a
    /// plausible-looking default here would put confidently wrong voltages on the
    /// wire instead of obviously unscaled ones.
    const float dividerRatio;

    /* Whether THIS channel carries the ADS1262 die temperature. There is ONE
     * sensor, so exactly one channel opts in or telemetry carries N copies of the
     * same number. It matters more in this build than elsewhere: with SHUNT_ADC
     * unregistered, these two facades are the only ones left, and without this the
     * die temperature -- the x-axis for the offset and reference drift a ratio
     * measurement lives or dies on -- would simply stop being recorded.
     *
     * Free to attach: pump() refreshes dieTempC_ on its own schedule
     * (TEMP_EVERY_N_SCANS) and this only reads the cached value. */
    const bool reportDieTemp;

    const int8_t pinSck, pinMosi, pinMiso, pinCs, pinStart;
    Sample lastSample{};

public:
    PowerSampler_RelayMuxChannel(RelayMuxAdcBackend &backend, RelayMux2::Target channel,
                                 uint8_t storageId, float dividerRatio,
                                 int8_t pinSck, int8_t pinMosi, int8_t pinMiso,
                                 int8_t pinCs, int8_t pinStart, bool reportDieTemp = false)
        : backend(backend), channel(channel), storageId(storageId),
          dividerRatio(dividerRatio), reportDieTemp(reportDieTemp), pinSck(pinSck),
          pinMosi(pinMosi), pinMiso(pinMiso), pinCs(pinCs), pinStart(pinStart) {}

    uint8_t getStorageId() const override { return storageId; }

    /// No current channel exists here, so mean power is NaN by construction and a
    /// vote would pin the board permanently awake -- the same reason TMP117 and the
    /// uncalibrated shunt channel opt out.
    bool measuresPower() const override { return false; }

    /// One published sample costs a full alternation, so the 4 s default would
    /// report this channel stalled while it is working normally. Derived from the
    /// backend's own constants rather than restated, and multiplied by 3 so a
    /// single slow cycle is not a stall.
    int64_t stallTimeoutUs() const override {
        /* 0 disables stall detection (energy_counter.h: `if (stallTimeout <= 0)
         * return;`). While the mux is parked under console control this channel
         * publishes nothing BY DESIGN, so a stall verdict would be false -- and its
         * recovery would move the relay. Not-publishing-because-parked and
         * not-publishing-because-dead are genuinely different states, and the
         * detector cannot tell them apart without being told. */
        if (backend.isManual()) return 0;
        return (int64_t) RelayMuxAdcBackend::cycleMsEstimate() * 3000;
    }

    bool init() override {
        if (!(dividerRatio > 0))
            ESP_LOGW("relaymux", "%s: dividerRatio unset, publishing raw ADC volts",
                     RelayMux2::targetName(channel));
        return backend.init(pinSck, pinMosi, pinMiso, pinCs, pinStart);
    }

    void startReading() override { backend.reArm(); }

    bool hasData() override { return backend.hasDataFor(channel); }

    Sample getSample() override {
        uint32_t diag = 0;
        const float v = backend.take(channel, diag);

        lastSample.setTimeNow();
        /* NAN in, NAN out, with the diagnostic attached. An over-ranged or
         * not-yet-converted pair writes NAN into volts_, and publishing that as
         * 0 V would be a measurement this channel never made. */
        lastSample.u = std::isnan(v) ? NAN : ((dividerRatio > 0) ? v * dividerRatio : v);
        lastSample.i = NAN;
        lastSample.p_ = 0.0f;   // pinned: no power measurement exists on this path
        if (reportDieTemp) lastSample.temp = backend.dieTempC();
        lastSample.diag = diag;
        return lastSample;
    }
};
