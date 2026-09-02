#pragma once

#include <Arduino.h>
#include <atomic>
#include "util.h"

/*
 * Coto 3502 relay mux, revision A -- two-channel differential input selector.
 *
 * Board: ~/dev/ee/hw/dmm-mux-TMUX862/relay-mux (DESIGN.md dated 2026-08-21) is the
 * authority for everything asserted here. Two Coto 3502-05-511 DPST Form-A reed
 * relays select input A or input B onto one isolated output pair; on this bench the
 * output goes through the ads1262-divider board into one ADS1262 input pair.
 *
 * WHAT THE HARDWARE DOES FOR US, and therefore what this driver must not duplicate:
 *
 *   NREQ_A  = not REQ_A
 *   NREQ_B  = not REQ_B
 *   VALID_A = ARM and REQ_A and NREQ_B
 *   VALID_B = ARM and REQ_B and NREQ_A
 *
 * One-hot is decoded in 74HCT11/74HCT14 logic, so "both requests high" energises
 * NEITHER coil: no firmware bug can COMMAND two coils at once. That is a statement
 * about coil drive and nothing more -- it does not prove the mechanical contacts
 * cannot overlap, which depends on release and operate times DESIGN.md still lists
 * as qualification items. A TPS3840 supervisor and a common-return MOSFET (Q3,
 * gated by ARM) cut coil current independently of either driver. Every J4 logic
 * input carries a 10k series resistor into a 100k pull-down, so all three lines LOW
 * -- which is also what a floating, unprogrammed or reset MCU presents -- means both
 * coils released.
 *
 * So there is no state to restore at boot, and a RESET releases both coils. A HANG
 * does not: a wedged task leaves the output latches driving whatever they last
 * drove, ARM included. DESIGN.md accepts static ARM for revision A and leaves a
 * watchdog-qualified ARM as an open system-level option; this firmware does not
 * provide one.
 *
 * WHAT THE HARDWARE DOES NOT DO, and is therefore this driver's whole job: the
 * documented switching sequence, and the waiting.
 *
 *   1. Deassert ARM.
 *   2. Set REQ_A = REQ_B = 0.
 *   3. Wait for contact release and dead time.
 *   4. Preload exactly one request.
 *   5. Assert ARM.
 *   6. Wait for relay AND measurement settling before accepting data.
 *
 * Steps 3 and 6 are the ones that cost real time, and skipping them does not fail
 * loudly -- it publishes a plausible voltage measured through a contact that was
 * still bouncing, or through the wrong input entirely.
 *
 * 3.3 V DRIVE: ARGUED, NOT GUARANTEED. J4's contract is that it receives at least
 * VOH 3.10 V. The logic inputs are ~110k to ground and draw ~30 uA, so an ESP32 pin
 * loaded this lightly sits at essentially VDD; 3.3 V through the 10k/100k divider
 * puts 3.0 V on the HCT input against a 2.0 V VIH, and 3.0 V on Q3's gate against
 * the PMV20XNEAR's 2.5 V specification. The arithmetic holds, but "VDD at 30 uA" is
 * a nominal-plus-light-load argument, not a datasheet VOH bound over supply,
 * temperature and cable drop -- and DESIGN.md leaves confirmation of all three
 * outputs open. MEASURE ARM/REQ_A/REQ_B at J4 before trusting a marginal harness.
 * The board still needs its own +5V_CTRL -- this drives logic only, never a coil.
 */
class RelayMux2 {
public:
    enum class Target : uint8_t { NONE = 0, CH_A = 1, CH_B = 2 };

    static const char *targetName(Target t) {
        switch (t) {
            case Target::CH_A: return "A";
            case Target::CH_B: return "B";
            default:           return "off";
        }
    }

    /* TIMING -- DELIBERATELY CONSERVATIVE, AND NOT YET QUALIFIED.
     *
     * DESIGN.md derives exactly one number for the delay-on network, 15.95 ms, and
     * it is the FASTEST corner (R = 0.80 M, C = 94.7 nF, VT+ = 1.2 V min). That
     * number exists to prove non-overlap -- it bounds how soon a coil can possibly
     * engage -- and it is the wrong end of the distribution for deciding when the
     * contact is definitely closed. The slow corner is not derived anywhere, and
     * DESIGN.md's open gates still say "Revision-A qualification must measure
     * contact release time" and "Logic timing and coil voltage do not prove contact
     * non-overlap". So these are engineering margins over an unmeasured quantity,
     * not datasheet limits.
     *
     * Nominal turn-on is around 1 M * 100 nF * ln(5/(5-VT+)) ~ 40 ms, and a slow
     * corner (R +20%, VT+ at the HCT14 maximum) lands near 65 ms. 250 ms is roughly
     * 4x that, plus reed operate time (sub-millisecond on the 3500 series), plus
     * bounce, plus the divider's own RC into the ADC.
     *
     * The cost of being generous is only rate: a full A/B alternation is about
     * 2 * (DEAD_TIME + SETTLE) plus the discarded conversions, so ~1 s. This is a
     * precision DC ratio front end, not a telemetry channel -- an averaging window
     * spans many alternations either way. TIGHTEN THESE ONLY AGAINST A SCOPE
     * TRACE of the actual contacts, which is what the board's qualification plan
     * asks for anyway; the console command below exists to make that measurement
     * possible without a rebuild. */
    static constexpr uint32_t DEAD_TIME_MS = 50;   ///< step 3: both coils released
    static constexpr uint32_t SETTLE_MS    = 250;  ///< step 6: contact closed and quiet

private:
    const uint8_t pinArm, pinReqA, pinReqB;

    enum class State : uint8_t {
        OFF,        ///< disarmed, no request, not switching
        DEAD_TIME,  ///< both requests cleared, waiting out contact release
        SETTLING,   ///< one request preloaded and ARM asserted, waiting for close
        SETTLED,    ///< contact believed closed and quiet; data may be trusted
    };

    State state = State::OFF;
    Target current = Target::NONE;   ///< what is (being) selected
    Target pending = Target::NONE;   ///< what select() asked for
    uint32_t tStateMs = 0;
    uint32_t generation_ = 0;        ///< bumped once per entry into SETTLED
    bool begun = false;

    /// Written by any task via requestFromOtherTask(), consumed only in tick().
    std::atomic<uint8_t> inboxTarget{(uint8_t) Target::NONE};
    std::atomic<bool> inboxPending{false};

    void driveAll(bool arm, bool reqA, bool reqB) const {
        digitalWrite(pinArm, arm);
        digitalWrite(pinReqA, reqA);
        digitalWrite(pinReqB, reqB);
    }

public:
    RelayMux2(uint8_t pinArm, uint8_t pinReqA, uint8_t pinReqB)
        : pinArm(pinArm), pinReqA(pinReqA), pinReqB(pinReqB) {}

    /// False if any control pin is unassigned. 255 is settings.h's "not wired on
    /// this board" sentinel and pinMode(255) is an "Invalid IO" HAL error, not a
    /// no-op -- the same mistake that put three of them in every boot log when the
    /// INA228 mux backend was left enabled on a board with no mux.
    bool pinsConfigured() const {
        return pinArm != 255 && pinReqA != 255 && pinReqB != 255;
    }

    bool begin() {
        /* Idempotent. begin() is now called from setup() so the state machine runs
         * whether or not anything else on the board came up, and the ADC backend
         * calls it too; a second call must not reset a mux that is mid-sequence or
         * already settled on a channel. */
        if (begun) return true;
        if (!pinsConfigured()) {
            ESP_LOGE("relaymux", "control pins unset (ARM=%u REQ_A=%u REQ_B=%u); not starting",
                     pinArm, pinReqA, pinReqB);
            return false;
        }
        /* Level BEFORE direction. An output pin adopts its level at the moment it
         * becomes an output, so pinMode-then-write briefly drives whatever was in
         * the output latch -- and a stray high on ARM with a stale request latched
         * is a coil pulse. Writing first makes the first driven level a known LOW.
         * The board's own pull-downs cover the window before this runs. */
        digitalWrite(pinArm, LOW);
        digitalWrite(pinReqA, LOW);
        digitalWrite(pinReqB, LOW);
        pinMode(pinArm, OUTPUT);
        pinMode(pinReqA, OUTPUT);
        pinMode(pinReqB, OUTPUT);
        driveAll(LOW, LOW, LOW);
        state = State::OFF;
        current = Target::NONE;
        pending = Target::NONE;
        tStateMs = millis();
        begun = true;
        ESP_LOGI("relaymux", "ready: ARM=%u REQ_A=%u REQ_B=%u, dead %lums settle %lums",
                 pinArm, pinReqA, pinReqB,
                 (unsigned long) DEAD_TIME_MS, (unsigned long) SETTLE_MS);
        return true;
    }

    /* CROSS-TASK COMMAND INBOX.
     *
     * select() mutates the state machine and MUST run only on the task that calls
     * tick() -- realTimeTask here. The console runs on appTask
     * (handleConsoleInput() is called from Telemetry::update()), so a console
     * command calling select() directly races tick().
     *
     * The race is not theoretical and not benign. select() writes `state` and
     * `tStateMs` as two separate stores; with `state` written first, tick() on the
     * other core can observe state == DEAD_TIME carrying the PREVIOUS tStateMs,
     * find the dead time already long expired, and assert ARM immediately. That
     * skips the entire contact-release wait -- the one thing this driver exists to
     * guarantee -- and re-energises the other coil while the first contact may
     * still be closed, briefly bridging two floating 0-80 V inputs.
     *
     * The board survives it: the 1 M/100 nF delay-on network cannot engage a coil
     * for at least ~16 ms (DESIGN.md's fastest corner), which is exactly why that
     * network exists. But that makes the hardware interlock load bearing in normal
     * operation instead of the backstop it is meant to be, which is precisely what
     * the comment in select() claims this driver does not do.
     *
     * So off-task callers post here and the RT task applies it in tick(), and the
     * state machine stays single-threaded by construction. A second request
     * arriving before the first is consumed replaces it -- last writer wins, which
     * is the right semantic for "what channel do you want" and cannot queue up
     * stale switching.
     *
     * std::atomic with explicit ordering, NOT volatile. volatile orders volatile
     * accesses against each other within one thread and nothing else: it is not a
     * synchronisation primitive and carries no release/acquire edge, so the
     * consumer has no guarantee of seeing the target store that preceded the flag
     * store. (An earlier version of this comment also claimed "one aligned 32-bit
     * store" -- both fields are a single byte, so that was wrong as well as
     * insufficient.) The release/acquire pair below is what actually publishes the
     * target, and the exchange makes claim-and-clear one operation. */
    void requestFromOtherTask(Target t) {
        inboxTarget.store((uint8_t) t, std::memory_order_relaxed);
        inboxPending.store(true, std::memory_order_release);   // publishes the store above
    }

    /// Drop an unconsumed request. Used when authority over the mux changes hands,
    /// where applying a request posted under the old owner would be wrong.
    void clearInbox() { inboxPending.store(false, std::memory_order_relaxed); }

    /// Ask for a channel. Idempotent: re-selecting what is already settled does
    /// NOT re-run the sequence, so a caller may spam this every pass without ever
    /// letting the contact stay closed long enough to measure through.
    ///
    /// NOT SAFE from a task other than the one calling tick() -- use
    /// requestFromOtherTask() for that.
    void select(Target t) {
        if (!begun) return;
        if (t == pending && (state != State::OFF || t == Target::NONE)) return;
        pending = t;
        /* Always enter through DEAD_TIME, including from OFF and including when
         * t == current. Steps 1-3 are unconditional in the documented sequence:
         * ARM drops and both requests clear before any new request is preloaded,
         * so no path exists where a request changes underneath an asserted ARM.
         * The hardware would decode that as "neither", but relying on the
         * interlock to cover a sequence violation means the interlock is load
         * bearing in normal operation instead of being the backstop it is. */
        driveAll(LOW, LOW, LOW);
        current = Target::NONE;
        /* Timestamp BEFORE the state it belongs to, always. Even now that the
         * cross-task path is gone this is the correct order: a state observed with
         * a stale timestamp expires instantly, and every failure of that kind
         * shortens a wait that exists for physical reasons. */
        tStateMs = millis();
        state = State::DEAD_TIME;
    }

    /// Release both coils and stop. The board reaches the same state on its own if
    /// this firmware dies; calling it explicitly just makes an intentional stop
    /// indistinguishable from one in the log.
    void off() { select(Target::NONE); }

    /// Non-blocking. Safe to call at any rate; it only looks at the clock.
    void tick() {
        if (!begun) return;
        /* exchange, not load-then-store: claim-and-clear is one operation, so a
         * request posted concurrently either loses the race and is seen next tick
         * or wins and re-arms the flag. Acquire pairs with the release above. */
        if (inboxPending.exchange(false, std::memory_order_acquire)) {
            select((Target) inboxTarget.load(std::memory_order_relaxed));
        }
        const uint32_t now = millis();
        switch (state) {
            case State::OFF:
            case State::SETTLED:
                break;

            case State::DEAD_TIME:
                if ((uint32_t) (now - tStateMs) < DEAD_TIME_MS) break;
                if (pending == Target::NONE) {
                    tStateMs = now;
                    state = State::OFF;
                    break;
                }
                // Steps 4 and 5: preload exactly one request, then arm.
                driveAll(LOW, pending == Target::CH_A, pending == Target::CH_B);
                driveAll(HIGH, pending == Target::CH_A, pending == Target::CH_B);
                current = pending;
                tStateMs = now;
                state = State::SETTLING;
                break;

            case State::SETTLING:
                if ((uint32_t) (now - tStateMs) < SETTLE_MS) break;
                tStateMs = now;
                state = State::SETTLED;
                ++generation_;
                break;
        }
    }

    bool isSettled() const { return state == State::SETTLED; }

    /// The channel whose contact is believed closed and quiet, or NONE. Callers
    /// must gate every published sample on this rather than on select()'s
    /// argument: the two differ for DEAD_TIME_MS + SETTLE_MS after every switch,
    /// which is exactly the window in which the data is wrong.
    Target settledTarget() const { return state == State::SETTLED ? current : Target::NONE; }

    Target requested() const { return pending; }

    /// Bumped once per arrival at SETTLED, so a consumer can tell "still the same
    /// closed contact" from "closed again since I last looked" without watching
    /// the state machine itself.
    uint32_t generation() const { return generation_; }

    /// Worst-case time from select() to trustworthy data, for callers sizing their
    /// own timeouts. Derived, never a second hardcoded copy.
    static constexpr uint32_t switchLatencyMs() { return DEAD_TIME_MS + SETTLE_MS; }

    const char *stateName() const {
        switch (state) {
            case State::OFF:       return "off";
            case State::DEAD_TIME: return "dead-time";
            case State::SETTLING:  return "settling";
            case State::SETTLED:   return "settled";
        }
        return "?";
    }
};
