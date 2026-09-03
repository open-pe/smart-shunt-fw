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

    /* AVERAGE THIS MANY CONSECUTIVE CONVERSIONS INTO ONE PUBLISHED SAMPLE.
     *
     * THIS IS A PALLIATIVE, NOT A FIX, AND THE DISTINCTION IS MEASURED.
     *
     * Symptom on the nest: VMUX_A published a clean bimodal distribution, most
     * samples on a floor and a few ~141 uV above it, with ~2 uV of spread WITHIN
     * each level. Raising N halved the excursion each time and never removed it:
     *
     *     N = 1   141 uV        N = 2    65 uV        N = 4   34.4 uV
     *
     * That is 1/N dilution of ONE bad conversion, not cancellation of a periodic
     * term. An earlier revision of this comment claimed the latter ("averaging N
     * cancels any disturbance whose period divides N") and predicted ~0 uV at
     * N = 4. The bench returned 34.4 uV. The claim was wrong; the numbers above
     * are what actually happened.
     *
     * THE REAL CAUSE is upstream, in ads1262.h. Every TEMP_EVERY_N_SCANS = 64
     * scans the driver reads die temperature, which drops chop, retunes gain and
     * INPMUX, then restores production config and toggles START -- restarting the
     * conversion cycle. ads1262.h:213 says it plainly: "every one of those
     * restarts is a fresh settling transient in the middle of a precision
     * measurement." 64 * 52.22 ms = 3.34 s. Against the ~836 ms round trip in
     * force when this was measured that is one dwell in four, which is where the
     * period-4 pattern came from -- but at the ~1.15 s round trip AVG_CONVERSIONS=4
     * produces it is ~2.9, so "one in four" is a fact about that measurement, not a
     * standing property. It also explains why affected samples arrived ~60-104 ms
     * late.
     *
     * So the correct fix is to DISCARD the conversions that follow a
     * temperature-read restart, the way DISCARD_SCANS already discards the ones
     * that follow a relay switch. Until that lands, averaging buys a 4x
     * reduction (141 -> 34 uV) for three extra conversions of dwell, and buys
     * sqrt(N) on white noise besides.
     *
     * MUST STILL BE EVEN, and the members must be genuinely adjacent -- see the
     * adjacency caveat on accLastGen, which this code CANNOT fully enforce. */
    static constexpr uint32_t AVG_CONVERSIONS = 4;

#ifdef RELAYMUX_SETTLE_SWEEP
    /* SETTLE CHARACTERISATION BUILD. Not for production -- it deliberately drives
     * the settle time below anything trustworthy in order to find where the
     * measurement breaks.
     *
     * SETTLE_MS_DEFAULT = 250 ms is a MARGIN, never a measurement: the driver only
     * ever widens it, and narrowing is left to a human with observed numbers in
     * front of them. Nobody has ever had those numbers on this rig, because the
     * console that reports them is unreachable (USB-CDC is dead on the nest) and
     * the stability observer cannot resolve below ~150 ms anyway -- it needs
     * STABLE_N=3 agreeing conversions at 50 ms each.
     *
     * So measure it from the OUTSIDE instead, where the resolution is not limited
     * by the conversion rate. Step settleMs through the list below, publish the
     * value in use as `P`, and let InfluxDB group by it. A settle that is too short
     * does not merely add noise: the reading is pulled toward the OTHER channel,
     * which on this rig is a 29 mV step, so the residual appears as a BIAS in the
     * mean and is visible far below the noise floor.
     *
     * The list is walked repeatedly rather than once, so that thermal drift over
     * the run averages across every step instead of aliasing onto one of them. */
    static constexpr uint32_t SWEEP_MS[] = {250, 200, 150, 120, 90, 70, 50, 35, 20};
    static constexpr uint8_t SWEEP_N = sizeof(SWEEP_MS) / sizeof(SWEEP_MS[0]);
    /* PUBLICATIONS per step, counted across BOTH channels -- so half this many per
     * channel. 96 gives 48 per channel, enough that a bias of ~1/5 of the ~1 uV sd
     * is resolvable, and ~45 s per step. */
    static constexpr uint32_t SWEEP_PUBS_PER_STEP = 96;
    uint8_t sweepIdx = 0;
    uint32_t sweepPubs = 0;
#endif
    static_assert(AVG_CONVERSIONS >= 2 && (AVG_CONVERSIONS % 2) == 0,
                  "AVG_CONVERSIONS must be even and >= 2");

    Target serving = Target::CH_A;   ///< whose turn it is
    uint32_t genAtSettle = 0;
    uint32_t muxGenSeen = 0;
    bool armedForCapture = false;
    /* Accumulator for the AVG_CONVERSIONS average. accN == 0 means "not started".
     *
     * accLastGen CANNOT fully enforce physical adjacency, and an earlier comment
     * here claimed it could. generation() advances when pump() PUBLISHES a
     * reading, and pump() consumes at most one per call; DRDY is a flag, not a
     * counter (ads1262.h:540), so conversions that elapse while nothing polls are
     * never counted. After a starved interval the next pump() still steps
     * generation() by one, `g - accLastGen == 1` passes, and two physically
     * non-adjacent values are averaged. The check therefore catches only gaps the
     * driver itself observed -- it is a cheap sanity check, NOT a guarantee, and
     * nPairBroken undercounts for the same reason. A temperature-read restart is
     * likewise invisible to it: the generations either side are consecutive. */
    uint32_t accN = 0, accLastGen = 0;
    float accSum = 0.0f;
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

    /* ---- SETTLE OBSERVATION -------------------------------------------------
     *
     * The conversions this backend used to throw away are the only evidence that
     * exists about what the relay actually did. DISCARD_SCANS still discards them
     * from the PUBLISHED value -- they are contaminated by construction -- but
     * they are read first, and what they show decides two things the fixed timer
     * cannot: whether the contact moved at all, and how long it really took.
     *
     * This costs no time. The conversions happen either way.
     *
     * RESOLUTION LIMIT, and it is not small: stability is declared only after
     * STABLE_N agreeing conversions. The first lands 0..52 ms after ARM and the
     * other two follow ~52 ms apart, so nothing below 104..156 ms can be OBSERVED
     * at the production FIR rate (19.15 SPS), whatever the relay actually does. This mechanism is therefore a guard and a
     * coarse upper bound, NOT the contact-timing measurement DESIGN.md asks for.
     * That needs a high-rate characterisation config -- sinc1 at a few kSPS, chop
     * off -- because the FIR filter this build runs caps at 20 SPS. */
    static constexpr uint8_t STABLE_N = 3;

    /* Settled when STABLE_N consecutive conversions span <= this. A NOISE
     * threshold, not an accuracy target: "has the reading stopped moving by more
     * than it moves anyway".
     *
     * 0 (THE DEFAULT) DISABLES THE GATE ENTIRELY, and that is deliberate.
     *
     * An earlier version defaulted this to 10 uV and claimed in this very comment
     * that too small was "the safe direction -- the channel falls back to the
     * fixed timer". That was simply false: there is no fallback. If stability is
     * never declared the code below waits out the grace and then publishes NaN
     * with DIAG_RELAY_UNSETTLED on EVERY sample, forever. A threshold set below
     * the real noise does not degrade gracefully, it kills the channel -- and
     * 10 uV at the ADC is 510 uV referred to the input through the 51:1 divider,
     * which any live converter's ripple exceeds. The shipped default would have
     * bricked the channel on its first real run.
     *
     * So the gate ships OFF: identical behaviour to before this mechanism existed.
     * The span is measured and reported regardless, so the operator sets this from
     * data -- `relaymux` prints the observed span -- and enables it deliberately.
     * That also keeps the auto-widen ratchet behind an explicit opt-in. */
    const float stableEpsV;
    bool stabilityGateEnabled() const { return stableEpsV > 0; }

    /* A reading this far from the other channel's last accepted value counts as
     * "the step happened". Also the threshold for EXPECTING a step at all: when
     * the two channels are closer together than this, the switch is
     * unobservable -- and, by the same arithmetic, a failure to switch would
     * change the result by less than this too.
     *
     * THE 1 mV DEFAULT IS WRONG FOR A RATIO INSTRUMENT, AND KNOWINGLY SO.
     * Raw signals here are ~20 mV, so 1 mV is 5% -- 50,000 ppm against a ~100 ppm
     * ambition. Two consequences, both bad and both currently live:
     *
     *  - When the two inputs differ by LESS than 1 mV the guard switches itself
     *    off (expectMove goes false). That is precisely the interesting case: a
     *    ratio measurement of two nearly equal voltages is where a failed switch
     *    is most damaging and least visible.
     *  - The re-baseline candidate test compares each reading against the FIRST
     *    candidate, so an "agreeing" set can span nearly 2 * moveEpsV.
     *
     * It was sized when the front end had 3.3 mV of bias-current offset and a
     * 141 uV transient, where 1 mV was defensible. With the PGA enabled the noise
     * floor is ~0.9 uV sd, so this can come down by two or three orders -- but the
     * right value is a bench fact about the contacts and the sources, not a guess.
     * Same contract as stableEpsV: measure the step, then set it. */
    const float moveEpsV;

    /* Reason byte in the high octet, matching ads1262.h's encodeDiag() layout so
     * the two cannot be confused downstream. */
    static constexpr uint32_t DIAG_RELAY_NO_STEP   = 0x40u << 24;
    static constexpr uint32_t DIAG_RELAY_UNSETTLED = 0x41u << 24;
    /* Published WITH a value, not instead of one: the reading matches neither
     * channel's last value. See the guard below for why that is a doubt and not
     * a verdict. */
    static constexpr uint32_t DIAG_RELAY_UNMATCHED = 0x42u << 24;

    float lastAccepted[3] = {NAN, NAN, NAN};   ///< indexed by Target
    uint32_t armGenSeen = 0;
    uint32_t obsGen = 0;
    /* A conversion arriving more than this after the previous one breaks the
     * ring. Without it, "STABLE_N consecutive conversions" was a lie: realTimeTask
     * ticks the mux ABOVE the g_samplingHalted check but only calls samplers below
     * it, so an OTA quiesce (or any skipped generation) leaves the ring intact
     * while msSinceArm() keeps climbing. Three entries straddling a 30 s halt would
     * agree, declare stability at tStableMs ~ 30 s, and the auto-widen would set a
     * 45 s settle. Two conversion periods at the chopped rate, with margin. */
    static constexpr uint32_t MAX_OBS_GAP_MS = 300;

    float ring[STABLE_N] = {NAN, NAN, NAN};
    uint8_t ringIdx = 0, ringCount = 0;
    uint32_t tLastObsMs = 0;
    float spanObservedV = NAN;   ///< always measured, gate or no gate
    float stepFrom = NAN;
    bool moved = false, stable = false;
    uint32_t tMoveMs = 0, tStableMs = 0;

    // reported, never acted on except to widen
    uint32_t settleObservedMs = 0, settleObservedMaxMs = 0, closeObservedMs = 0;
    uint32_t nNoStep = 0, nUnsettled = 0, nWidened = 0;
    /* Times an averaging run was restarted because generation() advanced by more
     * than one between polls -- the RT task did not get back in time. Not a fault
     * in itself (the sample is simply delayed by one conversion), but a rising
     * count means the loop is being starved and the averaging is costing dwell
     * time it should not. */
    uint32_t nPairBroken = 0;
    /* State for the "matches neither channel" test, which is a DOUBT and not a
     * verdict -- see the guard below. Counted and reported; never a reason to
     * withhold a reading. */
    uint32_t nUnmatched[3] = {0, 0, 0};
    uint32_t lastDiagOverride = 0;

    void resetObservation(Target ch) {
        ringIdx = ringCount = 0;
        moved = stable = false;
        tMoveMs = tStableMs = 0;
        tLastObsMs = 0;
        stepFrom = lastAccepted[(uint8_t) other(ch)];
    }

    void observe(float v, uint32_t tMs) {
        if (!std::isfinite(v)) return;
        /* Adjacency. A gap means the ring no longer holds consecutive
         * conversions, so it cannot support a statement about settling. */
        if (ringCount && (uint32_t) (tMs - tLastObsMs) > MAX_OBS_GAP_MS) {
            ringCount = ringIdx = 0;
            stable = false;
        }
        tLastObsMs = tMs;
        ring[ringIdx] = v;
        ringIdx = (uint8_t) ((ringIdx + 1) % STABLE_N);
        if (ringCount < STABLE_N) ++ringCount;
        if (ringCount == STABLE_N) {
            float mn = ring[0], mx = ring[0];
            for (uint8_t i = 1; i < STABLE_N; ++i) {
                if (ring[i] < mn) mn = ring[i];
                if (ring[i] > mx) mx = ring[i];
            }
            spanObservedV = mx - mn;   // recorded even when the gate is off
            if (stabilityGateEnabled() && (mx - mn) <= stableEpsV) {
                if (!stable) { stable = true; tStableMs = tMs; }
            } else if (stabilityGateEnabled()) {
                /* Re-open. Contact bounce is exactly a stable-then-moving-again
                 * pattern, and latching the first quiet moment would time the
                 * bounce rather than the settle. */
                stable = false;
            }
        }
        if (!moved && std::isfinite(stepFrom) && fabsf(v - stepFrom) > moveEpsV) {
            moved = true;
            tMoveMs = tMs;
        }
    }

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
     * The default WAS G=1 with the PGA BYPASSED, copied from the DCCT burden
     * channel, on the grounds that at G=1 the PGA's own output swing leaves each
     * input pin roughly -2.2..+2.2 V on these +-2.5 V rails, which a
     * ground-referenced divider output can exceed. Bypassed, full scale is the
     * reference itself, +-2.5 V.
     *
     * THAT WORST CASE IS NOT THIS SOURCE, AND BYPASS COSTS MORE THAN IT SAVES.
     * The divider is rated 0-80 V, 100 V engineering (ads1262-divider/gen_sch.py).
     * Ground-referenced means AINN sits at AGND, so V_cm = V_diff/2 and the PGA
     * outputs land at V_diff and 0 -- not at +-V_diff:
     *
     *      80 V -> 1.569 V out -> pins at 1.569 / 0 V
     *     100 V -> 1.961 V out -> pins at 1.961 / 0 V
     *
     * The +-2.2 V window is NOMINAL. AVDD comes from an ADP7118 specified +-1.8%
     * over line/load/temperature, so worst-case AVDD is 2.455 V and the limit is
     * 2.155 V, not 2.2. The GUARANTEED margins are 0.586 V at 80 V and 0.194 V at
     * 100 V, and the guaranteed clip point is ~109.9 V, not ~112 V. The board's
     * 100 V engineering limit still sits inside that, so the choice holds -- but
     * quote the tolerated numbers, not the nominal ones.
     *
     * THIS MARGIN IS THE DIVIDER'S, NOT THE ADC'S: re-range the divider and this
     * must be re-derived. AND NONE OF IT IS MEASURED -- nothing above 1.2 V in has
     * ever been put through this front end.
     *
     * What bypass costs is input current. Bypassed, the ADC input is the raw
     * switched-cap modulator rather than a buffer, and the divider presents
     * 1 M || 20 k = 19.6 kOhm. Measured on the nest 2026-09-03, both channels
     * carried the SAME additive offset -- A read 22.79 mV where 51:1 predicts
     * 19.61 mV, and B (open) read 3.33 mV where 0 is predicted. That ~3.3 mV over
     * 19.6 kOhm implies ~165 nA of input bias current, and it is 17% of a 1 V
     * reading. It is also why A-B was needed to recover 1 V instead of reading A
     * directly. The same bias-current sensitivity is the lever the die-temperature
     * restart pushes on to make its ~141 uV transient.
     *
     * So: G=1 with the PGA ENABLED. Buffered, bias should fall to picoamps and
     * both the offset and the transient's mechanism go with it. The ~165 nA is an
     * INFERENCE from the offset, not a datasheet figure -- SBAS661C's bypass-mode
     * input current has not been checked -- but the prediction is self-proving: if
     * B's 3.3 mV does not collapse toward zero, this reasoning is wrong. */
    RelayMuxAdcBackend(RelayMux2 &mux, Ads1262ShuntAdc &dev, Ads1262ShuntAdc::Pair pair,
                       uint8_t pgaGainCode = 0, bool pgaBypass = false,
                       float stableEpsV = 0.0f, float moveEpsV = 1e-4f)
        : mux(mux), dev(dev), pair(pair), pgaGainCode(pgaGainCode), pgaBypass(pgaBypass),
          stableEpsV(stableEpsV), moveEpsV(moveEpsV) {}

    Ads1262ShuntAdc::Pair adcPair() const { return pair; }

    bool init(int8_t pinSck, int8_t pinMosi, int8_t pinMiso, int8_t pinCs, int8_t pinStart) {
        if (initialised) return true;   // idempotent: whichever channel gets here first
        if (pair >= Ads1262ShuntAdc::PAIR_COUNT) {
            ESP_LOGE("relaymux", "pair %u is not a real ADC pair (0..%u); refusing to start",
                     (unsigned) pair, (unsigned) Ads1262ShuntAdc::PAIR_COUNT - 1);
            return false;
        }
        /* Idempotent, and normally already done in setup() -- the relay state
         * machine must run even when the ADC does not, or a board carrying only
         * the mux (bring-up) could never move a contact. */
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

    /// Learned timings and fault counts, for the console. All milliseconds since
    /// ARM rose; settleObserved is an UPPER bound (STABLE_N conversions of
    /// granularity), closeObserved is when the step was first seen.
    /// Conversions averaged into one published sample. Reported by the console so
    /// the operator can tell a quiet reading from a heavily averaged one.
    static constexpr uint32_t avgConversions() { return AVG_CONVERSIONS; }

    /// The settle time currently in force. Reported so a characterisation run can
    /// label each sample with the setting that produced it.
    uint32_t settleMsNow() const { return mux.settleMs(); }

    void settleStats(uint32_t &lastMs, uint32_t &maxMs, uint32_t &closeMs,
                     uint32_t &noStep, uint32_t &unsettled, uint32_t &widened,
                     uint32_t &pairBroken, uint32_t &unmatched,
                     float &spanV, float &stableEps) const {
        lastMs = settleObservedMs; maxMs = settleObservedMaxMs; closeMs = closeObservedMs;
        noStep = nNoStep; unsettled = nUnsettled; widened = nWidened;
        pairBroken = nPairBroken;
        unmatched = nUnmatched[1] + nUnmatched[2];
        /* The span is the number the operator needs in order to choose stableEpsV,
         * so it is reported unconditionally -- including on the fault paths, where
         * the old code recorded nothing and the console showed zeros forever while
         * never-settled climbed. A diagnostic must not be suppressed by the fault
         * it exists to diagnose. */
        spanV = spanObservedV; stableEps = stableEpsV;
    }

    /* Reports the REQUESTED mode, not the RT-task-applied one.
     *
     * `manual` is only synced from `manualReq` inside hasDataFor(), which runs off
     * the back of a registered sampler. The bring-up build (RELAY_MUX_ONLY)
     * registers none, so hasDataFor() is never called and `manual` stays false
     * forever -- the console reported "auto" immediately after a `relaymux a` had
     * parked the mux. Harmless there (nothing publishes and reArm() is never
     * reached), but a status line that contradicts the command just given is worse
     * than no status line, and the same staleness would appear on any board where
     * the ADC failed to come up.
     *
     * The requested value is the authoritative answer to "what mode am I in": it is
     * what the operator asked for, and `manual` only lags it by one RT pass in
     * builds that have one. stallTimeoutUs() reading this is also correct, and
     * marginally more responsive. reArm() deliberately keeps using RT-owned
     * `manual`, because it must not act on a mode change the RT task has not yet
     * applied. */
    bool isManual() const { return manualReq.load(std::memory_order_acquire); }
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

        /* NO mux.tick() here. realTimeTask ticks the state machine once per pass,
         * on the same task, so it advances even when no sampler is registered --
         * which is the bring-up case, where the ADC is absent and hasData() is
         * never called at all. Ticking here as well would be harmless but would
         * hide that dependency. */

        /* Not settled: nothing to do but keep the ADC scan alive. pump() must
         * still run or the device stops producing, and the discard bookkeeping
         * below needs a generation counter that is actually advancing. */
        dev.pump();

        /* Observe EVERY conversion from ARM onward, settled or not. A new switch
         * is signalled by armGeneration(), not generation(): the latter only says
         * a switch FINISHED, which is far too late to have watched it happen. */
        if (mux.armGeneration() != armGenSeen) {
            armGenSeen = mux.armGeneration();
            resetObservation(ch);
        }
        if (dev.generation() != obsGen) {
            obsGen = dev.generation();
            observe(dev.volts(pair), mux.msSinceArm());
        }

        if (!mux.isSettled() || mux.settledTarget() != ch) {
            armedForCapture = false;
            accN = 0; accSum = 0.0f;
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
            accN = 0; accSum = 0.0f;
            return false;
        }

        if ((uint32_t) (dev.generation() - genAtSettle) <= DISCARD_SCANS) return false;

        /* Collect AVG_CONVERSIONS CONSECUTIVE conversions, then average them.
         * See the AVG_CONVERSIONS comment for why this exists and why even.
         *
         * Consecutiveness is enforced, not assumed. If generation() advanced by
         * more than one since the last poll the run is restarted rather than
         * completed with a gap in it: non-adjacent conversions can share a phase,
         * and averaging those would preserve the very term this cancels. A restart
         * costs one more conversion of dwell and nothing else.
         *
         * NaN is summed, not skipped. An over-ranged or otherwise invalid
         * conversion must poison the average so the sample publishes NaN with the
         * device's own diagnostic -- silently averaging the finite half of a pair
         * would turn a detected fault into a plausible number. */
        {
            /* FREEZE ONCE FULL. Without this bound the accumulator keeps taking
             * terms while a downstream gate (the stability gate below) returns
             * false, and the divisor stays AVG_CONVERSIONS: three terms over two
             * publishes 3x/2 of a constant input, as a FINITE and therefore
             * plausible number. Found by review, not by the bench -- it is dormant
             * only because stableEpsV ships at 0 and nothing else returns false
             * after this point today. That is a property of the current
             * configuration, not of this code, so it is fixed here. */
            const uint32_t g = dev.generation();
            if (accN >= AVG_CONVERSIONS) {
                // full: hold the mean that vAvg is about to compute
            } else if (accN == 0) {
                accLastGen = g; accSum = dev.volts(pair); accN = 1;
            } else if (g != accLastGen) {
                if ((uint32_t) (g - accLastGen) != 1) {
                    ++nPairBroken;
                    accLastGen = g; accSum = dev.volts(pair); accN = 1;
                } else {
                    accLastGen = g; accSum += dev.volts(pair); ++accN;
                }
            }
            if (accN < AVG_CONVERSIONS) return false;
        }
        const float vAvg = accSum / (float) AVG_CONVERSIONS;

        /* The fixed timer and the scan discard have both elapsed. Everything from
         * here is the EVIDENCE gate, and it can only ever delay or refuse a
         * sample -- never release one earlier than the code above already would.
         * That asymmetry is deliberate: an adaptive mechanism that can shorten a
         * physical settle on the strength of its own measurements is a mechanism
         * that can talk itself into publishing a transient. */
        const uint32_t tMs = mux.msSinceArm();

        if (stabilityGateEnabled() && !stable) {
            /* Still moving after the full settle. Give it up to twice as long
             * again, then FAIL CLOSED: a reading that never stopped changing is
             * not a measurement, and publishing the last value of a bouncing
             * contact is exactly the plausible-wrong-number this driver exists to
             * avoid. */
            if (tMs < 3 * mux.settleMs()) return false;
            ++nUnsettled;
            ESP_LOGW("relaymux", "%s: never settled within %lums (span > %.3g V); publishing NaN",
                     RelayMux2::targetName(ch), (unsigned long) tMs, (double) stableEpsV);
            lastVolts = NAN;
            lastDiagOverride = DIAG_RELAY_UNSETTLED;
            return true;
        }

        /* THE RUNTIME GUARD. If both channels have a known last value and they
         * differ by more than moveEpsV, then switching MUST have moved the
         * reading. If it did not, the mux did not switch -- a stuck contact, an
         * unpowered coil (the supervisor holds COIL_EN low below ~4.6 V), a
         * broken control wire -- and the "measurement" is the OTHER channel
         * wearing this channel's name.
         *
         * Coverage scales the right way. The error from a failed switch is the
         * difference between the channels; so is the detectability. Below
         * moveEpsV we cannot see it, and below moveEpsV it does not matter. */
        const float mine = lastAccepted[(uint8_t) ch];
        /* The AVERAGE, not the instantaneous conversion. Everything downstream --
         * the step guard, the published value, lastAccepted -- must see the same
         * number, or the guard would vet a reading the channel does not publish. */
        const float v = vAvg;
        const bool expectMove = std::isfinite(mine) && std::isfinite(stepFrom) &&
                                fabsf(mine - stepFrom) > moveEpsV;

        /* MOVED IS NECESSARY, NOT SUFFICIENT. An unpowered coil (the supervisor
         * holding COIL_EN low) and a broken control wire are OPEN faults: BOTH
         * reeds release and the ADC reads the divider network's own termination.
         * If the other channel happens to sit somewhere else, `moved` goes true,
         * the guard passes, and the open network's reading is published under
         * this channel's name.
         *
         * The only extra evidence available is whether the reading resembles what
         * THIS channel read last time. It is recorded, and it is NOT acted on,
         * because acting on it was a bug in both directions:
         *
         *  - Withholding on it assumes THE INPUT IS STATIC BETWEEN VISITS, which
         *    is false by construction for an instrument whose job is to track a
         *    changing voltage. Two deliberate source changes on the bench --
         *    1.0 -> 1.2 V and 1.2 -> 1.5 V (2026-09-03) -- each silenced VMUX_A
         *    PERMANENTLY: take() refreshes lastAccepted[] only from finite
         *    samples, so the first NaN froze the baseline the test compares
         *    against and every later sample failed identically. A guard that
         *    cannot return to PASS on healthy input is broken whatever it catches.
         *
         *  - Adopting on it (N agreeing readings, then re-baseline) rests on "a
         *    real source repeats, an open contact floats". Also false: with the
         *    mux open the tap is tied to the Kelvin return through the 20 kOhm low
         *    leg, so an open contact reads a PERFECTLY STABLE value. The agreeing
         *    readings adopt it and it is published forever -- permanent silence
         *    traded for a permanent plausible wrong number, the worse failure.
         *    (~/dev/kb/adc/floating-mux-input-mirrors-previous-channel.md)
         *
         * One channel's readings cannot separate "the input moved" from "the
         * contact opened"; the information is not there, and neither NaN nor
         * adoption can manufacture it. So publish the number -- it exists, and it
         * is far more often a changed input than a failed relay -- and carry the
         * doubt in the DIAGNOSTIC, where a consumer can see it, query it and
         * decide, instead of destroying the measurement on a coin flip.
         *
         * The fault this was meant to catch is not thereby unguarded: an open
         * contact reads the network termination, and whenever the other channel is
         * not sitting on that same termination `expectMove && !moved` below fires
         * on it. On this rig that is measured -- B's input is open and reads
         * -14 uV, exactly where an open A would land. What is genuinely lost is
         * the case where BOTH the fault and the other channel sit near the
         * termination, and that case was never separable here anyway. */
        if (std::isfinite(mine) && std::isfinite(v) && std::isfinite(stepFrom)) {
            const float dMine = fabsf(v - mine), dOther = fabsf(v - stepFrom);
            if (dMine > moveEpsV && dOther > moveEpsV) {
                ++nUnmatched[(uint8_t) ch];
                ESP_LOGW("relaymux", "%s: reads %.6g V, matching neither channel "
                                     "(%.6g / %.6g) -- most likely the INPUT changed; "
                                     "publishing it with DIAG_RELAY_UNMATCHED (%lu so far)",
                         RelayMux2::targetName(ch), (double) v, (double) mine,
                         (double) stepFrom, (unsigned long) nUnmatched[(uint8_t) ch]);
                lastDiagOverride = DIAG_RELAY_UNMATCHED;
            }
        }

        if (expectMove && !moved) {
            ++nNoStep;
            ESP_LOGE("relaymux", "%s: expected a step of %.3g V and saw none -- mux did not "
                                 "switch (coil? supply? contact?); publishing NaN",
                     RelayMux2::targetName(ch), (double) fabsf(mine - stepFrom));
            lastVolts = NAN;
            lastDiagOverride = DIAG_RELAY_NO_STEP;
            return true;
        }

        /* Learned timings, reported only. tStableMs is bounded below by
         * STABLE_N conversions, so it is an UPPER bound on the true settle, never
         * a tight one -- see the resolution note on STABLE_N. */
        settleObservedMs = tStableMs;
        if (tStableMs > settleObservedMaxMs) settleObservedMaxMs = tStableMs;
        if (moved) closeObservedMs = tMoveMs;

        /* AUTO-WIDEN ONLY. If the contact is settling later than the configured
         * time, the configuration is wrong in the dangerous direction and is
         * corrected immediately. Narrowing is never automatic: it is a decision to
         * trust an estimate over a margin, and it belongs to a human with the
         * console's reported numbers in front of them (`relaymux settle <ms>`). */
#ifdef RELAYMUX_SETTLE_SWEEP
        /* AUTO-WIDEN OFF. It fires precisely when settleMs is short -- which is the
         * whole point of the sweep -- and would ratchet every step back up toward
         * the value being tested against, so the run would measure the widener
         * rather than the relay. */
        if (false) {
#else
        if (tStableMs > mux.settleMs()) {
#endif
            const uint32_t was = mux.settleMs();
            /* CAPPED. Widening is evaluated only after settleMs_ elapses, and
             * `stable` re-opens on any span excursion, so tStableMs tracks the
             * evaluation point: later evaluation -> later tStableMs -> larger
             * settleMs_ -> later evaluation. That is a positive feedback loop, and
             * with stableEpsV near the noise floor flicker is normal rather than
             * exceptional. Uncapped it ratchets x1.5 each time until the stall
             * detector fires. The cap turns a runaway into a bounded complaint. */
            const uint32_t want = tStableMs + tStableMs / 2;
            const uint32_t now = mux.setSettleMs(want);
            ++nWidened;
            if (want > RelayMux2::SETTLE_MAX_MS) {
                ESP_LOGE("relaymux", "%s settled at %lums; implied settle %lums exceeds the "
                                     "%lums cap -- clamped. Bad contact, or stableEpsV is "
                                     "below the noise floor.",
                         RelayMux2::targetName(ch), (unsigned long) tStableMs,
                         (unsigned long) want, (unsigned long) RelayMux2::SETTLE_MAX_MS);
            } else {
                ESP_LOGW("relaymux", "%s settled at %lums, later than the configured %lums -- "
                                     "widening to %lums",
                         RelayMux2::targetName(ch), (unsigned long) tStableMs,
                         (unsigned long) was, (unsigned long) now);
            }
        }

        lastVolts = v;
        return true;
    }

    /// Consume the reading and hand the mux to the other channel. Called only by
    /// the serving channel's getSample(), mirroring the INA228 backend's advance.
    float take(Target ch, uint32_t &diagOut) {
        diagOut = lastDiagOverride ? lastDiagOverride : dev.diag();
        lastDiagOverride = 0;
        const float v = lastVolts;
        /* Remember what this channel actually read, so the NEXT switch away from
         * it knows what step to expect. Only finite values: a faulted sample must
         * not become the baseline that decides whether the following switch was
         * observable. */
        if (std::isfinite(v)) lastAccepted[(uint8_t) ch] = v;
        lastVolts = NAN;
        armedForCapture = false;
        accN = 0; accSum = 0.0f;
#ifdef RELAYMUX_SETTLE_SWEEP
        /* Applied HERE, before select(), so the new value governs the whole of the
         * next dwell rather than half of it. */
        if (++sweepPubs >= SWEEP_PUBS_PER_STEP) {
            sweepPubs = 0;
            sweepIdx = (uint8_t) ((sweepIdx + 1) % SWEEP_N);
            const uint32_t applied = mux.setSettleMs(SWEEP_MS[sweepIdx]);
            ESP_LOGW("relaymux", "settle sweep -> %lums (requested %lums)",
                     (unsigned long) applied, (unsigned long) SWEEP_MS[sweepIdx]);
        }
#endif
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
        return 2 * (RelayMux2::switchLatencyMs() +
                    (DISCARD_SCANS + AVG_CONVERSIONS) * SCAN_MS_PESSIMISTIC);
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
#ifdef RELAYMUX_SETTLE_SWEEP
        /* P carries the settle time in ms for this sample. Overriding the pin is
         * safe only because this build exists to be grouped by it; the production
         * build keeps P at 0 so nothing downstream reports a plausible wattage. */
        lastSample.p_ = (float) backend.settleMsNow();
#else
        lastSample.p_ = 0.0f;   // pinned: no power measurement exists on this path
#endif
        if (reportDieTemp) lastSample.temp = backend.dieTempC();
        lastSample.diag = diag;
        return lastSample;
    }
};
