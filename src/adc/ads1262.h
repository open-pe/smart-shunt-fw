#pragma once

// ADS1262 sampler for the pwr-metering "shunt-adc" board (revB).
//
// This is NOT the ProtoCentral breakout wiring -- that version is in git
// history. The shunt-adc board is a bipolar, externally clocked, 4-differential-
// pair efficiency front end behind a 5-forward/1-reverse digital isolator, and
// three of its choices change the driver contract. All references are to
// SBAS661C and to the notes on hw/shunt-adc/shunt-adc.kicad_sch in the
// pwr-metering repo.
//
//  N6/N11  There is no dedicated /DRDY line: the isolator's one reverse channel
//          carries DOUT, so the ready signal rides the DOUT/nDRDY pin (U7 pin 13
//          -> J2.5). The host ties its DRDY input to the MISO pin, holds /CS
//          asserted low forever (sec.9.4.5: "CS must be low to enable the
//          DOUT/DRDY pin"), and masks the edge handler for the whole transfer
//          because every data bit is an edge on that same wire.
//  N11     /RESET//PWDN (U7 pin 20) is NOT brought out -- bringing it out would
//          have cost the isolator channel the conversion clock now owns.
//          Recorded there as an accepted limitation. We reset over SPI instead;
//          a host that loses interface sync must power-cycle the board.
//  N10     fCLK is an external 7.3728 MHz oscillator reaching XTAL1 through the
//          isolator. sec.9.4.8: "If no external clock is detected, the ADC
//          automatically selects the internal oscillator." SILENTLY. A cut
//          clock wire, an unfitted Y1 or an unplugged J8 does not stop
//          conversions -- the board keeps emitting well-formed, plausible,
//          UNSYNCHRONISED data. Nothing in the numbers looks wrong. The only
//          detector is EXTCLK, bit 5 of the STATUS byte, and the schematic note
//          requires it be checked on EVERY acquisition, not once at bring-up,
//          because the fallback can happen at any time. checkExtClk() below is
//          that check and it refuses the sample rather than flagging it.
//
//          THE FALLBACK ALSO BREAKS MAINS REJECTION, which is not obvious from
//          "unsynchronised" and is the larger error of the two. Table 9-6 is
//          indexed by the RATIO TOLERANCE of power line to ADC clock, and the
//          internal oscillator is +-0.5% typ but +-2% MAX (Table 7-5), so the
//          fallback moves the production FIR @ 20 SPS off the +-2% column
//          (-95 dB at 50 Hz) and onto +-6% (-66 dB) -- 29 dB, a factor of 28.
//          Expressed as the differential 50 Hz pickup that consumes a 10 ppm
//          budget on a 35 mV signal (0.35 uV), that is 19.7 mV falling to
//          0.70 mV. 0.70 mV of differential line pickup is entirely plausible
//          on real wiring, so a cut clock wire opens a mains-frequency error
//          path big enough to blow the budget on its own -- silently, with
//          every number still looking plausible. Two independent reasons the
//          EXTCLK check refuses the sample instead of merely flagging it.
//
// J2 pin 7 is the revA /DRDY position and MUST be left unconnected
// (guard_host_header_pin7_open): open reads high = "never ready" = a loud
// timeout, whereas grounded or driven low reads as permanent "data ready" and
// the host consumes garbage forever.

#include <SPI.h>
#include <ti_ads126x.h>
#include <esp32-hal-periman.h>  // perimanGetPinBusType() -- who owns each pin

#include "sampling.h"
#include "util.h"

/* Not declared in TI's ads1263.h, but defined (non-static) in ads1263.c. */
extern "C" uint8_t calculateChecksum(const uint8_t dataBytes[], uint8_t numBytes);

/* Defined in ads1262.cpp, NOT here. Two reasons, both load-bearing:
 *  - `static` in a header gives every translation unit its own copy of the
 *    flags, so the ISR would set one TU's and waitForEdge() read another's;
 *  - `inline` fixes that but gives an IRAM_ATTR function vague linkage, and the
 *    xtensa linker then fails with "dangerous relocation: l32r: literal placed
 *    after use". One TU owns them. */
extern volatile bool ads1262_busBusy;
extern volatile bool ads1262_ready;
extern TaskNotification ads1262_notification;

void IRAM_ATTR ads1262_drdy_isr();
void IRAM_ATTR ads1262_bus_busy(bool busy);


/// Owns the single ADS1262 and round-robins its input multiplexer over the four
/// differential pairs. The PowerSampler facades below read from it; they do not
/// touch the chip.
class Ads1262ShuntAdc {
public:
    /// Input pairs, in scan order. Ports per the schematic's J3..J6 notes.
    /// Numbered, not named after an assumed role: J3..J6 are four generic
    /// differential ports (note N11), and which one carries a shunt versus a
    /// divided voltage is a bench decision, not a property of the board.
    enum Pair : uint8_t {
        PAIR_CH0 = 0,   ///< J3, AIN0(+)/AIN1(-)
        PAIR_CH1,       ///< J4, AIN2(+)/AIN3(-)
        PAIR_CH2,       ///< J5, AIN4(+)/AIN5(-)
        PAIR_CH3,       ///< J6, AIN6(+)/AIN7(-)
        PAIR_COUNT,
        /// "No pair on this axis." Only valid as PowerSampler_ShuntAdc's uPair,
        /// where it selects single-pair mode: the mux parks on iPair and never
        /// moves. Deliberately outside [0, PAIR_COUNT) so it can never be used
        /// to index INPMUX_FOR[] by accident.
        PAIR_NONE = 0xFF
    };

    static constexpr float VREF = 2.50f;      ///< internal reference (REFMUX=0x00, REFOUT bypassed to AVSS)
    static constexpr uint8_t PGA_GAIN = 32;   ///< G=32; note N8 sizes the input window on exactly this
    static constexpr uint8_t PGA_GAIN_CODE = 5;  // log2(32)
    static constexpr uint8_t DATA_RATE_CODE = 0x04;  ///< 20 SPS -- the fastest the FIR filter allows

    /// SCLK. Kept well below the 8 MHz maximum: every edge crosses an ISO7761
    /// with up to 5.9 ns of pulse-width distortion (note N10).
    static constexpr uint32_t SPI_HZ = 2000000;

    /// First-conversion latency after a START, td(STDR): 52.22 ms at 20 SPS with
    /// the FIR filter [SBAS661C Table 9-13]. Every timeout below is sized off
    /// this, not guessed -- an earlier version bounded the EXTCLK confirmation by
    /// an iteration count that expired in ~8 ms and so failed on every boot with
    /// perfectly good hardware.
    static constexpr uint32_t FIRST_CONVERSION_MS = 53;

    /// Conversions discarded after an input-mux change, before a reading is
    /// accepted. ZERO is correct here, per SBAS661C sec.9.4.2: "The FIR and
    /// sinc1 filter modes are zero latency providing the conversion result in
    /// single cycle", and the same section's instruction for scanning -- "To
    /// make sure that conversions are settled after changing channels, start a
    /// new conversion for each channel using the START pin or start command" --
    /// is exactly what selectPair() does. The stated precondition, "settled data
    /// are provided, assuming the analog input is settled before the start
    /// condition", holds: the board's differential input filter corners at
    /// 796 Hz (note N11), i.e. ~200 us, against a 52 ms conversion.
    ///
    /// This is ONLY valid while MODE1 selects FIR. A higher-order sinc has
    /// "more than one conversion latency and therefore require[s] more
    /// conversion cycles to provide fully settled data" -- the static_assert
    /// below fails if the filter is changed without revisiting this.
    static constexpr uint8_t SETTLE_DISCARDS = 0;

    /// MODE1 value: FILTER[2:0] = 100 => FIR.
    /// MODE0: continuous run mode, no reference reversal, and CHOP ON.
    ///
    /// Chop measured on this board (internal shorted input, G=32): input-referred
    /// offset 7.1331 uV -> 0.0082 uV, an 867x reduction. 7.13 uV is 204 ppm of a
    /// 35 mV shunt signal, so without chop the offset term alone blows a 10 ppm
    /// budget by 20x.
    ///
    /// Cost is far smaller than the doubled FIRST-conversion latency of
    /// sec.9.4.2 suggests: steady-state throughput is Equation 21,
    /// 1/td(STDR) = 19.15 SPS against 20.00, i.e. 4%. The per-pair restart in
    /// selectPair() does pay the doubled latency, which is what sets scan
    /// period -- see the health line for the measured value.
    ///
    /// NOTE: the temperature sensor and the supply monitors REQUIRE chop off
    /// (sec.9.3.4/9.3.5). configureSelfTest() drops it and applyProductionConfig()
    /// puts it back; do not write MODE0 anywhere else.
    static constexpr uint8_t MODE0_VALUE = MODE0_CHOP_ON;

    /// MODE1: FILTER[2:0] = 100 => FIR. FIR is only offered at 20 SPS and below
    /// (sec.9.3.10.2), so this is the fastest FIR the part has.
    ///
    /// FIR rather than sinc4 EVEN WITH THE MUX PARKED on one pair, which is not
    /// the obvious answer -- sinc4 @ 10 SPS is 2.3x quieter per sample. Chop is
    /// what decides it: Equation 21 makes the chopped rate 1/td(STDR), so sinc4
    /// @ 10 SPS runs at 2.5 SPS (td 400.4 ms), not 10, while FIR's single-cycle
    /// settling keeps it at 19.15 SPS (td 52.22 ms).
    ///
    /// Table 8-1 is quoted for CHOP OFF, so its numbers must be divided by 1.4
    /// before they can be used here: "The ADC noise is reduced by a factor of
    /// 1.4 with chop mode enabled" (sec.8.8). Chopped, per unit time:
    ///     FIR   @ 20 SPS: (30 nV / 1.4) / sqrt(19.15) = 4.9 nV/sqrt(s)
    ///     sinc4 @ 10 SPS: (13 nV / 1.4) / sqrt(2.5)   = 5.9 nV/sqrt(s)
    /// FIR wins by 1.20x. An earlier revision of this comment gave 6.9 against
    /// 8.2, which is the same calculation with the UNCHOPPED Table 8-1 values --
    /// wrong in absolute terms, though the ratio survives because the 1.4 is
    /// common to both. The sinc4 advantage only exists with chop OFF. Same
    /// reasoning, with the measurement it came from, in zeroDriftSample().
    ///
    /// The price is mains rejection: -95 dB at 50 Hz against sinc4 @ 10 SPS's
    /// -136 (Table 9-6, reproduced under ZERO_MODE1_VALUE). Sized rather than
    /// assumed: -95 dB tolerates 19.7 mV RMS of DIFFERENTIAL 50 Hz pickup before
    /// it alone consumes a 10 ppm budget on 35 mV. (A second figure, "~0.4 mV
    /// before it reaches the noise floor", used to sit here; it does not
    /// reconstruct from either the chopped or the unchopped floor -- 1.2 mV and
    /// 1.7 mV respectively -- so it is removed rather than adjusted, its
    /// derivation being unknown.) 19.7 mV differential on a twisted Kelvin
    /// pair is not a realistic number, so FIR is the right trade here -- but it
    /// is a trade, and it rests on EXTCLK being healthy (note N10).
    static constexpr uint8_t MODE1_VALUE = 0x80;
    static_assert(SETTLE_DISCARDS > 0 || MODE1_VALUE == 0x80,
                  "SETTLE_DISCARDS=0 relies on the FIR filter's zero latency "
                  "(SBAS661C sec.9.4.2). A sinc2..sinc5 filter needs discards > 0.");

    /// Conversions averaged per pair before a reading is published. Noise falls
    /// as sqrt(N). Sized for the ~2.5 SPS on-air target: SBAS661C sec.9.4.2 says
    /// only the FIRST conversion after a START costs td(STDR) = 52.22 ms; in
    /// continuous mode the rest arrive at the nominal 20 SPS = 50 ms. So one pair
    /// with N=8 is 52.22 + 7*50 = 402 ms => 2.49 SPS.
    /// 1 = no device-level averaging. EnergyCounter/MeanWindow already averages
    /// every sample in the summary window, so averaging here too would duplicate
    /// existing plumbing AND slow the scan (N=8 makes a 4-pair scan ~1.6 s).
    /// Raise it only if you need per-sample noise reduction the window cannot give.
    static constexpr uint8_t AVG_N = 1;

    /// Scans between die-temperature reads. It is one value for the whole chip,
    /// not per channel, and it drifts slowly.
    ///
    /// 64, NOT 8, and the cost of 8 is throughput rather than bus traffic. Each
    /// read drops chop, switches to gain 1 and INPMUX 0xBB, then
    /// applyProductionConfig() + selectPair() put it back -- and selectPair()
    /// toggles START, which restarts the conversion cycle for td(STDR): 52.22 ms
    /// at FIR/20 SPS, DOUBLED to ~104 ms by chop (Equation 19). At 8 that
    /// restart lands every 8 * 52.22 = 418 ms of sampling, so ~20-25% of wall
    /// time is spent re-settling rather than converting, and every one of those
    /// restarts is a fresh settling transient in the middle of a precision
    /// measurement.
    ///
    /// 64 also removes the not-ready-frame churn, MEASURED on the bench
    /// (2026-08-16): 8 gave ~1 all-zero first-read per temperature read (96
    /// against 93 expected, then 86 against 85); 64 gave 13 against 13. The 8x
    /// fall tracking the 8x fewer restarts is what makes the restart causal
    /// rather than merely correlated -- see the DROP ANY LATCHED DRDY block in
    /// selectPair(), which is the fix for the frames themselves.
    ///
    /// Nothing needs it faster. The die temperature is the x-axis for offset and
    /// reference drift, both of which move over minutes; 64 scans is ~3.3 s.
    static constexpr uint8_t TEMP_EVERY_N_SCANS = 64;

    /// Minimum gap between PGA range-alarm error lines. The alarm is latched
    /// per conversion, so an excursion logs ~19x/s and buries everything else.
    static constexpr uint32_t RANGE_LOG_INTERVAL_MS = 1000;

    /// Conversions per production clock-rate window. 32 at the chopped FIR rate
    /// of 19.15 SPS is ~1.7 s, comfortably inside the 64 conversions between
    /// temperature reads -- so a window never spans the restart a temperature
    /// read causes, and never has to model its ~104 ms cost.
    static constexpr uint16_t RATE_WINDOW_CONVERSIONS = 32;

    /// How long fclkHz() will keep serving a running estimate before calling it
    /// unverified. A window is ~1.7 s at nominal; a clock at a quarter speed
    /// still closes one in ~6.7 s, so this is deliberately far above the
    /// interesting degradations -- it exists for "conversions stopped
    /// altogether", not for "the clock got slower".
    ///
    /// The far tail is the case that matters. As the clock slows the windows get
    /// longer, and at some point conversions stop arriving at all -- at which
    /// point a rate-based guard has NOTHING to measure. Without this the last
    /// good estimate would be served forever, and the guard would go quiet
    /// exactly when the fault became total.
    static constexpr uint32_t RATE_STALE_MS = 20000;


    /// Conversions discarded after ENTERING zero mode, paid once per
    /// configuration change rather than per sample.
    ///
    /// MEASURED, not assumed. Instrumenting the discard loop on the bench
    /// (2026-08-15, temp-read -> zero-mode re-entry, 6 re-entries) printed every
    /// discarded conversion: settle[0] averaged 7.1419 uV and settle[7] averaged
    /// 7.1421 uV, with ±15 nV of scatter across all eight positions -- i.e. the
    /// per-conversion noise and NO transient. The first conversion after re-entry
    /// is already good, because DRDY/STATUS_ADC1 is hardware-gated on settled
    /// data and the part simply does not flag a conversion before it is valid.
    ///
    /// The previous value of 8 was a guess ("more than the 2 it replaced") and
    /// cost 800 ms at ZERO_DATA_RATE_CODE's 10 SPS on every temperature read --
    /// the single largest term in the sample period. One is kept rather than
    /// zero as margin for the analog step this measurement covers only in one
    /// direction (temp sensor at 125 mV/gain 1 -> shorted input at gain 32) and
    /// for chopExperiment()'s chop ON/OFF transition, which was not measured.
    ///
    /// This discard is also effectively FREE. Timestamping conversions against
    /// the zero-mode restart (same bench run) showed the first arrives at
    /// +400 ms and the rest every 100 ms: td(STDR) for sinc4 is FOUR conversion
    /// periods of filter refill, and the discarded conversion is the one that
    /// ends that refill. Dropping to 0 would save 100 ms, not 400. The refill
    /// is the price of ZERO_DATA_RATE_CODE, and 10 SPS is what puts sinc notches
    /// on BOTH 50 and 60 Hz -- raising the rate to shorten it would trade mains
    /// rejection for throughput.
    static constexpr uint8_t ZERO_SETTLE_DISCARDS = 1;

    /// Filter and rate for the ZERO-DRIFT channel only -- deliberately NOT the
    /// production FIR @ 20 SPS, which stays as it is for the mux scan.
    ///
    /// Sinc4 @ 10 SPS wins on both axes that matter to a static measurement:
    ///   noise (Table 8-1, gain 32): 0.013 uVRMS vs 0.030 for FIR @ 20 SPS.
    ///     That is a genuine per-unit-time gain, not just a slower sample --
    ///     two averaged FIR conversions cover the same 100 ms at 0.021 uVRMS.
    ///     NOTE this comparison holds only with CHOP OFF; see zeroDriftSample(),
    ///     where Equation 21 reverses it.
    ///   50 Hz rejection (Table 9-6): -136 dB vs the production FIR @ 20 SPS's
    ///     -95 dB. (An earlier revision of this line attributed the -95 dB to
    ///     "sinc4 @ 20 SPS" -- that row is -72 dB. See the table below.)
    ///
    /// The rate matters as much as the order: sinc notches sit at MULTIPLES OF
    /// THE DATA RATE, so the rate has to divide the line frequency. 10 divides
    /// 50; 20 does not, which is why sinc4 @ 20 SPS collapses to -72 dB at
    /// 50 Hz. Do not "speed this up" to 20 SPS without re-reading Table 9-6.
    ///
    /// Table 9-6 verbatim, the four rows this driver can select. Columns are the
    /// RATIO TOLERANCE of power line to ADC clock -- with Y1 fitted and EXTCLK
    /// confirmed, the +-2% columns apply with wide margin (crystal is ppm-level,
    /// ENTSO-E holds 50 Hz inside +-0.1%). See note N10 for what the internal-
    /// oscillator fallback does to that assumption.
    ///
    ///     rate  filter  50Hz+-2%  60Hz+-2%  50Hz+-6%  60Hz+-6%
    ///       20  FIR        -95       -94       -66       -66     <- production
    ///       20  sinc4      -72      -136       -72       -96
    ///       10  FIR       -111       -94       -73       -68
    ///       10  sinc4     -136      -136      -100      -100     <- zero channel
    ///
    /// Only NMRR (differential) is tabulated. COMMON-mode line pickup is a
    /// separate and much easier path: CMRR is 130 dB typ at 20 SPS (Table 7-4).
    ///
    /// What it costs, and why this channel can pay it: conversion latency
    /// td(STDR) (Table 9-13) is 400.4 ms against FIR's 52.22 ms. The production
    /// scan pays that on EVERY mux change and could not afford it (4 pairs would
    /// go from ~209 ms to 1.6 s). This channel never leaves AINCOM, so it pays
    /// the settle once per configuration change and never again.
    static constexpr uint8_t ZERO_MODE1_VALUE = MODE1_FILTER_SINC4;
    static constexpr uint8_t ZERO_DATA_RATE_CODE = MODE2_DR_10_SPS;

    /// td(STDR) for the zero channel's filter and rate (sinc4 @ 10 SPS),
    /// Table 9-13. With chop this is ALSO the conversion period, not just the
    /// first-conversion latency -- Equation 21 makes the chopped data rate
    /// 1/td(STDR).
    static constexpr uint32_t ZERO_TD_STDR_MS = 401;

    /// A missed conversion presents as a line stuck low with NO further edges
    /// (note N11 b), so an edge-only wait would hang forever. After this long
    /// without an edge we read anyway, which re-arms the pin. Must exceed the
    /// first-conversion latency of the CURRENT configuration, or a healthy start
    /// looks like a stall -- so it is a runtime value (drdyTimeoutMs_), not one
    /// constant. This is the production default, for FIR @ 20 SPS.
    static constexpr uint32_t DRDY_TIMEOUT_MS = 4 * FIRST_CONVERSION_MS;

    /// Budget for confirming EXTCLK at init. Several conversion periods, so a
    /// slow or retried first conversion cannot be mistaken for a missing clock.
    static constexpr uint32_t EXTCLK_CONFIRM_MS = 8 * FIRST_CONVERSION_MS;

    // diag reason codes, continuing the convention in ina228.h (1=out-of-range, 2=jump)
    static constexpr uint8_t DIAG_CHECKSUM = 3;
    static constexpr uint8_t DIAG_DEVICE_RESET = 4;
    static constexpr uint8_t DIAG_HAL_FAULT = 5;
    static constexpr uint8_t DIAG_NO_EXTCLK = 6;
    static constexpr uint8_t DIAG_CLOCK_DEGRADED = 7;
    /// Input drove the PGA out of range during the conversion: PGAD_ALM
    /// (differential over-range) or PGAH/PGAL_ALM (PGA output within 0.2 V of a
    /// rail).
    ///
    /// NOT a current threshold, and emphatically not the 17.5 A this comment
    /// used to claim. 17.5 A is 35 mV, the DESIGN point note N8 sizes the input
    /// window from -- a choice about the board, not a wall in the part. The
    /// hardware limits are ~39.06 A (digital clipping) and ~41.02 A (PGAD_ALM),
    /// with a blind band between them where the output saturates and no alarm
    /// fires; see the full derivation at the over-range check in pump(). And
    /// PGAH/PGAL are COMMON-MODE alarms that can fire at any current, including
    /// none. Sizing a bench test off this constant is how the earlier figure was
    /// going to waste someone's afternoon.
    static constexpr uint8_t DIAG_PGA_RANGE = 8;
    /// The running clock estimate is too old to stand behind. NOT "the clock is
    /// bad" -- it is "nobody can currently say", which is a different report and
    /// must not be collapsed into either a healthy value or a fault code that
    /// implies a measurement was made.
    static constexpr uint8_t DIAG_CLOCK_UNVERIFIED = 9;

    /// Fractional deviation of the running fCLK estimate that raises
    /// DIAG_CLOCK_DEGRADED. Wide, because the estimate is derived from only a few
    /// conversion intervals and millisecond timestamps -- this exists to catch
    /// the GROSS failure seen on 2026-08-15 (the clock fell to a quarter speed
    /// and the channel ran on for 35 minutes before dying), not to police drift.
    static constexpr float FCLK_ALARM_TOLERANCE = 0.20f;

private:
    static constexpr uint8_t INPMUX_FOR[PAIR_COUNT] = {
            0x01,  // AIN0 / AIN1
            0x23,  // AIN2 / AIN3
            0x45,  // AIN4 / AIN5
            0x67,  // AIN6 / AIN7
    };

    /* Not atomic, and deliberately so: SamplerRegistry::initAll() calls every
     * sampler's init() sequentially on one task before any sampling task exists,
     * so the check-then-set below cannot race today. If samplers ever get their
     * own tasks this needs a real guard. */
    bool initialized = false;
    bool everInitialized = false;  ///< distinguishes "not yet up" from "lost it"

    /* Health telemetry. fCLK is NOT re-measured with checkClock(): that costs a
     * 200 ms window plus a full configuration round trip. Every ADS1262 timing
     * scales with the clock, so the conversion rate measureZero() already
     * computes IS a clock measurement -- ratio against a healthy reference gives
     * fCLK for free, every sample, with no mux excursion at all. */
    float fclkInitHz_ = NAN;   ///< measured by checkClock() while init still trusted the part
    float zeroSpsRef_ = NAN;   ///< zero-mode conversion rate when the clock was known good
    /* The configuration zeroSpsRef_ was captured under. The reference is keyed on
     * the CONFIGURATION, not on the register cache: every temperature read
     * poisons the cache (configureSelfTest sets zeroModeChop_=0xFF) and forces a
     * re-entry with IDENTICAL settings, so keying on the cache re-anchored the
     * reference roughly every 13-50 s. A sustained degradation was then absorbed
     * into the reference within one cycle -- the alarm fired once, went quiet,
     * and fclkEstHz_ returned to nominal while the part ran at quarter speed.
     * That is a guard that disappears exactly when the fault is real. */
    uint8_t refChop_ = 0xFF, refMode1_ = 0xFF, refDr_ = 0xFF;
    float fclkEstHz_ = NAN;    ///< running estimate; NAN until a reference exists

    /* PRODUCTION-PATH clock tracking. The zero-mode machinery above measures the
     * conversion rate too, but ONLY inside measureZero() -- and that sampler is
     * not registered in a measurement build. Retiring the zero channel therefore
     * silently disarmed the whole clock guard: fclkEstHz_ stayed NAN, fclkHz()
     * fell back to the boot-time fclkInitHz_, and SHUNT_ADC_HEALTH published
     * that as the CURRENT clock with a clean diagnostic, indefinitely.
     *
     * That is the same failure the comment above describes and worse: on
     * 2026-08-15 the clock fell to a quarter speed and the channel ran on for
     * 35 minutes. A guard written for exactly that event was disabled by a
     * one-line change in main_esp32.cpp, with nothing to say so.
     *
     * Same trick as the zero path -- the rate IS the clock, measured for free
     * from conversions we already take -- but anchored to the production
     * configuration, which never changes at runtime. So the reference is
     * captured ONCE and cannot be re-anchored by a temperature read, which is
     * the drift the zero path had to defend against explicitly. */
    uint32_t rateWinStartMs_ = 0;   ///< millis() at the first conversion of the window
    uint16_t rateWinCount_ = 0;     ///< conversions accumulated in the window
    float prodSpsRef_ = NAN;        ///< production rate when the clock was known good
    uint32_t lastRateOkMs_ = 0;     ///< millis() of the last completed rate window
    uint32_t lastStaleLogMs_ = 0;   ///< rate-limits the stale-estimate error line
    bool rateEverMeasured_ = false; ///< false until the first window closes
    float avddAvss_ = NAN;     ///< refreshed by refreshHealth(), NAN until first success
    float dvdd_ = NAN;
    bool isrAttached = false;      ///< attachInterrupt() is not idempotent; do it once
    uint8_t pair = 0;              ///< pair currently selected in INPMUX
    uint8_t discardsLeft = SETTLE_DISCARDS;
    uint32_t lastEdgeMs = 0;

    float volts_[PAIR_COUNT] = {NAN, NAN, NAN, NAN};
    float dieTempC_ = NAN;   ///< ADS1262 internal temperature sensor, degC
    uint8_t tempSkip_ = TEMP_EVERY_N_SCANS;   ///< force a read on the first scan
    /// DRDY stall timeout for the configuration currently programmed. Tracks the
    /// config because a fixed value cannot serve both: 212 ms is right for the
    /// production FIR @ 20 SPS, and would fire on EVERY conversion of the chopped
    /// sinc4 zero channel, whose conversions are 400 ms apart with the first at
    /// 800 ms. A stall timeout that trips on healthy hardware is worse than none,
    /// because the re-arm read it triggers disturbs the very conversion it was
    /// wrongly told to rescue.
    uint32_t drdyTimeoutMs_ = DRDY_TIMEOUT_MS;

    uint8_t zeroModeChop_ = 0xFF;   ///< chop bits currently programmed; 0xFF = not in zero mode
    uint8_t zeroModeMode1_ = 0xFF;  ///< filter currently programmed in zero mode
    uint8_t zeroModeDr_ = 0xFF;     ///< data rate currently programmed in zero mode
    uint8_t zeroSettleLeft_ = 0;
    float sd_[PAIR_COUNT] = {NAN, NAN, NAN, NAN};   ///< sample stddev of the last average

    /* Running accumulation for the pair currently being averaged. */
    double sum_ = 0, sumsq_ = 0;
    uint8_t nAvg_ = 0;

    /* When set, the mux stays on this pair instead of scanning. Costs no restart
     * between conversions, so all of the time budget goes into averaging. */
    int8_t onlyPair_ = -1;
    bool scanningFacade_ = false;      ///< a facade needs the full mux scan
    bool zeroFacade_ = false;          ///< the zero-drift facade owns the part
    bool measurementFacade_ = false;   ///< a shunt-measurement facade owns the part
    uint32_t sinceRestart_ = 0;   ///< conversions read since the last selectPair()
    uint32_t lastRangeLogMs_ = 0;  ///< rate-limits the PGA range error; it fires per conversion
    bool rangeLogged_ = false;     ///< so the FIRST one is never rate-limited away
    uint32_t notReadyReads_ = 0;  ///< all-zero "nothing yet" frames; ~1 per restart is normal
    uint32_t generation_ = 0;      ///< bumped when all four pairs have a reading

    /* Two diag slots, because TWO facades read this one device. A single
     * take-and-clear slot meant whichever facade polled second always saw 0 --
     * reporting "no problem" for a fault it simply never got to observe.
     * diag_ accumulates within the current scan; diagPublished_ is latched when
     * the scan completes and is READ, not consumed, so both facades see it. */
    uint32_t diag_ = 0;
    uint32_t diagPublished_ = 0;

    /// Raise a fault that ABORTS the conversion, and publish it immediately.
    ///
    /// The two-slot scheme latches diag_ into diagPublished_ when a scan
    /// COMPLETES -- which is exactly wrong for a fault that stops scans from
    /// completing. A stuck-low MISO is the worked example: every read returns
    /// all-zero bytes, the checksum test fails, pump() returns early, and
    /// generation_ never advances. Nothing then ever copies diag_ across, so
    /// PowerSampler_ShuntAdcHealth kept publishing the LAST GOOD diagnostic
    /// every 30 s for as long as the fault lasted. The measurement channel goes
    /// quiet (hasData() gates on generation_), and a quiet channel plus a clean
    /// health channel is indistinguishable from an idle, healthy board. The
    /// louder the hardware fault, the more thoroughly it was hidden from the
    /// only path that leaves the device -- absence of evidence encoding absence
    /// of the problem, on the one channel a remote observer can see.
    ///
    /// Publishing directly is safe against the opposite error (a stale fault
    /// sticking forever) because a completed scan overwrites diagPublished_
    /// from diag_, and diag_ is cleared at that point. So a fault that clears
    /// itself is reported until the next good scan and then goes away on its
    /// own, with no separate ageing rule to get wrong.
    void raiseFault(uint8_t code, int32_t count) {
        diag_ = encodeDiag(code, count);
        diagPublished_ = diag_;
    }

    /// Fold one completed conversion into the production clock-rate window.
    ///
    /// Called for every FRESH conversion (after the STATUS_ADC1 gate), including
    /// ones later discarded for over-range: an over-range conversion still took
    /// exactly one conversion period, which is the only thing being measured
    /// here. Excluding them would make the measured rate depend on the SIGNAL,
    /// which is precisely what a clock guard must not do.
    ///
    /// WHAT THIS ACTUALLY MEASURES is the rate at which conversions are
    /// CONSUMED, which equals the conversion rate only while pump() keeps up.
    /// ads1262_ready is a flag, not a counter, so a starved caller silently
    /// drops edges and the measured rate falls -- and this guard will report
    /// that as a degraded clock. That is a false POSITIVE, i.e. it errs toward
    /// the alarm, which is the correct direction to be wrong in; but it means a
    /// DIAG_CLOCK_DEGRADED must be read as "conversions are not arriving at the
    /// expected rate", not as proof the crystal is at fault. checkClock() is the
    /// one that measures fCLK directly, and it is the right follow-up.
    void noteConversionForRate() {
        const uint32_t now = millis();
        if (rateWinCount_ == 0) {
            rateWinStartMs_ = now;
            rateWinCount_ = 1;
            return;
        }
        if (++rateWinCount_ < RATE_WINDOW_CONVERSIONS) return;

        const uint32_t elapsed = (uint32_t) (now - rateWinStartMs_);
        rateWinCount_ = 0;
        /* No time base -- millis() has not ticked across the whole window. That
         * is un-evaluable, not fast: bank NOTHING, do not touch lastRateOkMs_,
         * and let the window restart. Banking a rate here would divide by zero,
         * and treating it as fresh would hide a stall behind a bogus success. */
        if (elapsed == 0) return;

        const float sps = (float) (RATE_WINDOW_CONVERSIONS - 1) * 1000.0f / (float) elapsed;
        lastRateOkMs_ = now;
        rateEverMeasured_ = true;

        if (!std::isfinite(prodSpsRef_)) {
            /* First window becomes the reference, and is never re-anchored --
             * the production configuration does not change at runtime, so there
             * is no legitimate reason for the expected rate to move. Trustworthy
             * only because checkClock() measured fCLK against FCLK_NOMINAL_HZ
             * during init and raises DIAG_CLOCK_DEGRADED itself if the clock was
             * ALREADY off; without that this would happily adopt a quarter-speed
             * clock as "normal" and then report every later comparison clean. */
            prodSpsRef_ = sps;
            return;
        }
        if (!(prodSpsRef_ > 0)) return;

        fclkEstHz_ = fclkInitHz_ * (sps / prodSpsRef_);
        const float dev = fabsf(sps / prodSpsRef_ - 1.0f);
        if (dev > FCLK_ALARM_TOLERANCE) {
            raiseFault(DIAG_CLOCK_DEGRADED, 0);
            ESP_LOGW("ads1262", "clock degraded: production rate %.2f SPS vs %.2f reference "
                                "(fCLK ~ %.0f Hz vs %.0f at init). Conversion timing and the "
                                "50/60 Hz FIR nulls scale with this.",
                     sps, prodSpsRef_, fclkEstHz_, fclkInitHz_);
        }
    }

    /// Raise a fault when the running clock estimate has gone unverifiable.
    ///
    /// Runs from pump() rather than from fclkHz(), because it must fire when
    /// conversions have STOPPED -- and fclkHz() is only called by the health
    /// sampler, which is not where the absence of conversions shows up.
    ///
    /// This is the far-tail case for a rate-based guard: measuring the clock
    /// FROM the conversions works only while conversions exist. As the clock
    /// degrades the guard gets better (longer windows, larger deviation) right
    /// up to the point where it gets no input at all, and there it must not fall
    /// back to the last good answer. It reports unverified instead.
    void checkRateFreshness() {
        if (!rateEverMeasured_) return;   // no window has closed yet; init value still stands
        const uint32_t now = millis();
        if ((uint32_t) (now - lastRateOkMs_) <= RATE_STALE_MS) return;

        raiseFault(DIAG_CLOCK_UNVERIFIED, 0);
        if ((uint32_t) (now - lastStaleLogMs_) >= RATE_STALE_MS) {
            lastStaleLogMs_ = now;
            ESP_LOGE("ads1262", "no completed conversion window for %u ms -- the clock estimate is "
                                "UNVERIFIED, not healthy. fclkHz() now reports NAN rather than the "
                                "last good value.", (unsigned) (now - lastRateOkMs_));
        }
    }

    int8_t pinMiso_ = -1;
    int8_t g_pinSck = -1, g_pinMosi = -1, g_pinCs = -1;

    /// Encodes a diagnostic the way sampling.h documents and ina228.h implements:
    /// reason<<24 | sign<<20 | adc_code&0xFFFFF. The raw ADS1262 code is 32-bit
    /// signed and cannot fit in 20 bits, so the sign goes in its own field
    /// rather than being truncated away -- masking a negative code to 20 bits
    /// would silently present it as a large positive one.
    static uint32_t encodeDiag(uint8_t reason, int32_t code) {
        const uint32_t sign = (code < 0) ? 1u : 0u;
        const uint32_t mag = (uint32_t) (code < 0 ? -(int64_t) code : (int64_t) code);
        return ((uint32_t) reason << 24) | (sign << 20) | (mag & 0xFFFFF);
    }

    void selectPair(uint8_t p) {
        /* START low -> INPMUX -> START high restarts the conversion cycle, which
         * is what sec.9.4.1 wants after an input change; the board wires START
         * to a host GPIO on J2.8 for exactly this (and for cross-board sync). */
        setSTART(LOW);
        writeSingleRegister(REG_ADDR_INPMUX, INPMUX_FOR[p]);
        setSTART(HIGH);
        /* This is an INPMUX writer, so the zero-mode cache no longer describes
         * the part. Every other writer (applyProductionConfig, configureSelfTest,
         * init's self-test restore) clears it; omitting it here let a subsequent
         * enterZeroMode() with unchanged chop bits skip reconfiguration and
         * average whatever production pair is selected, published as "offset uV". */
        zeroModeChop_ = 0xFF;
        pair = p;
        discardsLeft = SETTLE_DISCARDS;
        sum_ = sumsq_ = 0;
        nAvg_ = 0;

        /* DROP ANY LATCHED DRDY. ads1262_ready is sticky: the ISR sets it and
         * waitForEdge() clears it on consumption, so an edge latched BEFORE the
         * START toggle above survives into the next wait and makes it return
         * true immediately -- against a part whose conversion cycle restarted
         * microseconds ago and which therefore has nothing to give. readData()
         * then clocks out a mid-conversion word, and the checksum catches it.
         *
         * MEASURED, not theorised (2026-08-16, single-pair CH0 on the bench):
         * exactly one "checksum mismatch" per die-temperature read -- 96 against
         * 93 expected temp reads, then 86 against 85. Raising
         * TEMP_EVERY_N_SCANS 8 -> 64 dropped it to 13 against 13, an 8x fall
         * tracking the 8x fewer restarts, which is what makes this causal rather
         * than correlated. refreshHealth() is simply the most frequent restarter:
         * it runs three self-test conversions, each toggling DRDY, then restores
         * the configuration through here.
         *
         * Losing a real conversion this way is not possible: an edge latched
         * before the START toggle describes the OLD configuration and must be
         * discarded, and a genuine new conversion cannot arrive until td(STDR)
         * afterwards, when the ISR sets the flag again.
         *
         * lastEdgeMs moves too, so the no-edge timeout is measured from the
         * restart rather than from an edge belonging to the previous config --
         * otherwise a restart late in the timeout window can trip the re-arm
         * read immediately, which is the same corrupt read by another route. */
        ads1262_ready = false;
        lastEdgeMs = millis();
        sinceRestart_ = 0;
        /* Abandon any part-built rate window. A START toggle costs td(STDR)
         * (~104 ms chopped), so a window spanning a restart measures the restart
         * rather than the clock and would read as a degraded clock on perfectly
         * healthy hardware -- a guard that fires on good input is as useless as
         * one that stays quiet on bad input, and it trains you to ignore it. */
        rateWinCount_ = 0;
    }

public:
    uint32_t generation() const { return generation_; }

    /// Diagnostics latched from the most recently completed scan. Non-consuming:
    /// every facade sharing this device sees the same value.
    uint32_t diag() const { return diagPublished_; }

    /// True when the device WAS up and has since lost its configuration (the
    /// device-reset flag). Distinct from "never initialised", so a caller can
    /// tell "retry me" from "I was never asked to start".
    bool needsReinit() const { return everInitialized && !initialized; }

    /// Differential volts at the ADC input for a pair, NAN until first read.
    float volts(Pair p) const { return volts_[p]; }

    /// Restrict the mux to ONE pair, or -1 to scan all four. Single-pair mode
    /// avoids a mux restart between conversions, so every 50 ms goes into the
    /// average rather than into re-settling.
    ///
    /// Selects the pair IMMEDIATELY as well as recording it. Recording alone left
    /// the marker and the hardware disagreeing: init() always programs and records
    /// PAIR_CH0, so after a device-reset recovery an onlyPair_ of anything else
    /// would have the mux physically on CH0 while pump() wrote its readings into
    /// volts_[CH0] -- and the facade, reading volts_[iPair], would keep
    /// republishing its last pre-reset value with fresh timestamps until the next
    /// temperature boundary re-selected the pair. Stale data with a current
    /// timestamp is the worst of the available failures, so the two are kept in
    /// step at the one place that sets the marker. Harmless for onlyPair_ == CH0,
    /// which is the only configuration shipped today -- but this mode advertises
    /// arbitrary pairs.
    void setOnlyPair(int8_t p) {
        onlyPair_ = p;
        if (p >= 0 && p < (int8_t) PAIR_COUNT && initialized) selectPair((uint8_t) p);
    }

    /// Current single-pair selection, or -1 when scanning. Facades read this at
    /// init() to refuse an incompatible pairing rather than silently disagree.
    int8_t onlyPair() const { return onlyPair_; }

    /// Set by a facade that needs the full mux scan. Only ever set, never
    /// cleared: facades are static objects that live for the whole run, so a
    /// clear would only ever be wrong.
    void noteScanningFacade() { scanningFacade_ = true; }
    bool hasScanningFacade() const { return scanningFacade_; }

    /// Ownership claims, so two facades that cannot coexist refuse LOUDLY at
    /// init() instead of quietly corrupting each other's readings. Only ever
    /// set, never cleared: facades are static objects that live for the whole
    /// run, so a clear could only ever be wrong.
    void noteZeroFacade() { zeroFacade_ = true; }
    bool hasZeroFacade() const { return zeroFacade_; }
    void noteMeasurementFacade() { measurementFacade_ = true; }
    bool hasMeasurementFacade() const { return measurementFacade_; }

    /// Sample standard deviation of the conversions behind volts(p). This is the
    /// measured noise, not an assumed one -- use it to choose AVG_N.
    float voltsStdDev(Pair p) const { return sd_[p]; }

    /// ADS1262 die temperature in degC, NAN until the first scan completes.
    /// SBAS661C sec.9.3.4 notes the die runs ~0.7 degC above the surrounding
    /// PCB from self-heating, so this tracks board temperature with an offset.
    float dieTempC() const { return dieTempC_; }

    /// Best available fCLK: the running estimate once a reference exists, else
    /// the value checkClock() measured at init. NAN if the part never came up --
    /// never a nominal stand-in, which would report a healthy clock for a board
    /// that was never checked.
    ///
    /// Returns NAN once the running estimate goes stale (RATE_STALE_MS with no
    /// completed window). Before the first window closes it still returns the
    /// init measurement, which is then the freshest thing that exists -- the
    /// staleness rule only applies once something better was being produced and
    /// has stopped. Falling back to fclkInitHz_ forever was the actual bug: a
    /// boot-time reading published as the CURRENT clock, for hours, with a clean
    /// diagnostic beside it.
    float fclkHz() const {
        if (!rateEverMeasured_) return fclkInitHz_;
        if ((uint32_t) (millis() - lastRateOkMs_) > RATE_STALE_MS) return NAN;
        return std::isfinite(fclkEstHz_) ? fclkEstHz_ : fclkInitHz_;
    }

    float avddAvss() const { return avddAvss_; }
    float dvdd() const { return dvdd_; }

    bool init(int8_t pinSck, int8_t pinMosi, int8_t pinMiso, int8_t pinCs, int8_t pinStart) {
        if (initialized) return true;

        pinMiso_ = pinMiso;
        g_pinSck = pinSck; g_pinMosi = pinMosi; g_pinCs = pinCs;

        SPI.begin(pinSck, pinMiso, pinMosi, pinCs);

        Ads126xHalConfig cfg;
        cfg.spi = &SPI;
        cfg.spiHz = SPI_HZ;
        cfg.pinCS = pinCs;
        cfg.pinSTART = pinStart;
        cfg.pinResetPwdn = -1;      // not brought out through the isolator (note N11)
        cfg.pinDRDY = pinMiso;      // shares DOUT; recorded, never polled
        cfg.csHoldLow = true;       // sec.9.4.5 -- required to enable DOUT/DRDY
        cfg.drdyOnDout = true;      // make waitForDRDYinterrupt() refuse
        cfg.onBusBusy = ads1262_bus_busy;

        if (!ads126xHalBegin(cfg)) {
            ESP_LOGE("ads1262", "HAL config rejected");
            return false;
        }

        setSTART(LOW);

        /* Bring-up order from note N12: the clock must be running before the
         * ADC is reset, so that reset resolves the clock-source detection.
         * 10 ms is Y1's MAXIMUM startup, not its typical. The clock is powered
         * from the host through J2.1, so by the time this code runs it is
         * already up -- this wait costs nothing and removes the ordering
         * assumption. */
        delay(10);

        /* No /RESET pin exists here, so reset over SPI. Note N12 calls the
         * reset "belt-and-braces" because clock detection is continuous rather
         * than latched, but it also puts every register at a known default,
         * which is what restoreRegisterDefaults() then claims. */
        sendCommand(OPCODE_RESET);
        delay(5);                   // tWAKE
        restoreRegisterDefaults();  // resync the driver's shadow map

        static constexpr uint8_t kIntRef = 0x01;   // POWER.INTREF
        static constexpr uint8_t kVBias = 0x00;

        const struct { uint8_t addr, val; } regs[] = {
                /* POWER bit 4 (RESET) written as 0 clears the device's reset
                 * flag, which pump() then watches to detect a device that reset
                 * out from under us and lost this configuration. With no /RESET
                 * pin that flag is our only notification. */
                {REG_ADDR_POWER,     (uint8_t) (kVBias | kIntRef)},
                /* STATUS byte is MANDATORY on this board, not a nicety: it
                 * carries EXTCLK, and note N10 warns that leaving the bit clear
                 * means the required clock check has nothing to read -- "the
                 * same silent-pass shape one level up". Checksum on as well. */
                {REG_ADDR_INTERFACE, 0x05},
                {REG_ADDR_MODE0,     MODE0_VALUE},  // continuous, CHOP ON, no ref reversal
                {REG_ADDR_MODE1,     MODE1_VALUE},  // FIR (50/60 Hz nulls, note N10)
                /* MODE2: PGA ENABLED (bit 7 = 0, unlike the breakout config) at
                 * G=32. Note N8 derives the +-1.6575 V input window from G=32
                 * and 35 mV full scale; changing the gain invalidates that
                 * window and the bipolar rail choice that follows from it. */
                {REG_ADDR_MODE2,     (uint8_t) ((PGA_GAIN_CODE << 4) | DATA_RATE_CODE)},
                {REG_ADDR_INPMUX,    INPMUX_FOR[PAIR_CH0]},
                {REG_ADDR_OFCAL0,    0x00},
                {REG_ADDR_OFCAL1,    0x00},
                {REG_ADDR_OFCAL2,    0x00},
                {REG_ADDR_FSCAL0,    0x00},
                {REG_ADDR_FSCAL1,    0x00},
                {REG_ADDR_FSCAL2,    0x40},  // full-scale calibration = 1.0
                {REG_ADDR_IDACMUX,   0x00},  // excitation currents off
                {REG_ADDR_IDACMAG,   0x00},
                {REG_ADDR_REFMUX,    0x00},  // internal 2.5 V reference
                {REG_ADDR_TDACP,     0x00},
                {REG_ADDR_TDACN,     0x00},
                {REG_ADDR_GPIOCON,   0x00},  // AIN8/AIN9/AINCOM stay analog inputs, not GPIO
                {REG_ADDR_GPIODIR,   0x00},
                {REG_ADDR_GPIODAT,   0x00},
        };

        for (auto &r: regs)
            writeSingleRegister(r.addr, r.val);

        /* Read the configuration back. TWO jobs, and the second is easy to miss:
         *
         *  1. It is the only proof an ADS1262 is there and listening -- an
         *     unpowered isolated side leaves DOUT idle high, so the readback is
         *     0xFF and cannot match.
         *  2. It is what makes TI's shadow registerMap[] correct. Upstream
         *     writeSingleRegister() has its shadow update COMMENTED OUT
         *     (ads1263.c:240), so writes alone leave the shadow stale, while
         *     readSingleRegister() does update it. readData() sizes its transfer
         *     from that shadow via STATUS_BYTE_ENABLED / CRC_BYTE_ENABLED, so a
         *     stale INTERFACE entry silently desyncs the wire format.
         *
         * DO NOT trim this loop to "verification only, skip in release". */
        bool readbackOk = true;
        uint8_t all_and = 0xFF, all_or = 0x00;
        for (auto &r: regs) {
            uint8_t got = readSingleRegister(r.addr);
            all_and &= got;
            all_or |= got;
            if (got != r.val) {
                /* Cap the per-register spam: when the line is stuck every one of
                 * them mismatches, and 20 identical lines per retry bury the
                 * verdict that actually tells the user what to do. */
                if (readbackOk)
                    ESP_LOGE("ads1262",
                             "register readback mismatch at 0x%02x: wrote 0x%02x, read 0x%02x",
                             r.addr, r.val, got);
                readbackOk = false;
            }
        }
        if (!readbackOk) {
            /* Say WHICH failure this is rather than leaving the user to guess.
             * The two stuck-line cases are distinguishable and mean different
             * things on this board. */
            if (all_or == 0x00)
                ESP_LOGE("ads1262", "every register reads 0x00 -- MISO is never driven. Either "
                                    "the isolator's HOST side is unpowered (J2.1 is a 3V3 INPUT "
                                    "fed by the host; there is no on-board source on that net) "
                                    "or the DOUT wire is not landing on the MISO pin. "
                                    "IF Y1 IS OSCILLATING, +3V3P IS PRESENT and the host side is "
                                    "powered -- Y1 runs off that same rail -- so it is the DOUT "
                                    "path (J2.5 -> MISO, or isolator channel F), not the supply.");
            else if (all_and == 0xFF)
                ESP_LOGE("ads1262", "every register reads 0xFF -- DOUT idle high, which is what "
                                    "an UNPOWERED ISOLATED SIDE looks like. Check J1's 5 V feed.");
            else
                ESP_LOGE("ads1262", "partial readback -- suspect SCLK integrity, SPI mode, or a "
                                    "marginal connection.");

            /* The message above lists candidates; these two narrow them, and cost
             * nothing because init has already failed.
             *
             * They probe the SAME wire in two different ways, which is the point:
             * DOUT and DRDY share one pin on this board (J2.5), so there is no
             * second path to cross-check against.
             *   checkClock()    AC: does the converter produce DRDY EDGES at all?
             *                       Edges with a dead readback would mean the link
             *                       carries transitions but not valid data --
             *                       SPI mode or timing, not a broken wire.
             *   probePinDrive() DC: is the line DRIVEN at all, or floating? This
             *                       is the one that separates "chip is dead" from
             *                       "wire is not connected", which the 0x00 alone
             *                       cannot.
             * Order matters: probePinDrive() calls pinMode() on the MISO pin,
             * which detaches it from the SPI peripheral (esp32-hal-gpio.c
             * perimanClearPinBus), so nothing may use SPI after it. */
            ESP_LOGW("ads1262", "link diagnostics (init has already failed):");
            (void) checkClock();
            probePinDrive("DOUT/DRDY/MISO", pinMiso);
            return false;
        }

        if (ads126xHalFaults()) {
            ESP_LOGE("ads1262", "%u HAL faults during init -- readback was not real",
                     (unsigned) ads126xHalFaults());
            return false;
        }

        /* Identity only. The configuration is reported by logConfig() from
         * registers that have been read back, at every point that programs it --
         * this line used to assert "G=32, FIR @ 20 SPS" as a literal and was
         * wrong within a second of boot whenever a sampler reprogrammed the part. */
        uint8_t id = readSingleRegister(REG_ADDR_ID);
        ESP_LOGI("ads1262", "found %s (ID=0x%02x)",
                 (id & ID_DEV_MASK) == ID_DEV_ADS1262 ? "ADS1262" : "ADS1263", id);

        /* DRDY is the MISO pin. ads126xHalBegin() left it as the SPI
         * peripheral's; attaching an interrupt to it is additive on ESP32 --
         * the peripheral keeps driving the input, we just also watch edges. */
        if (!isrAttached) {
            attachInterrupt(digitalPinToInterrupt((uint8_t) pinMiso), ads1262_drdy_isr, FALLING);
            isrAttached = true;
            ESP_LOGI("ads1262", "DRDY interrupt on MISO pin %d (shares DOUT, note N6)", pinMiso);
        }

        /* Self-tests BEFORE the production configuration, in the order that
         * gives the most specific diagnosis: the clock test terminates in
         * bounded time whatever is wrong, and a bad clock would otherwise make
         * the supply test look like a dead converter. */
        if (!checkClock()) return false;
        if (!checkSupplies()) return false;

        /* Restore the production configuration the self-tests overwrote, and
         * read it back -- upstream's writeSingleRegister() does not update the
         * driver's shadow map, only readSingleRegister() does. */
        setSTART(LOW);
        writeSingleRegister(REG_ADDR_MODE0, MODE0_VALUE);
        writeSingleRegister(REG_ADDR_MODE1, MODE1_VALUE);
        writeSingleRegister(REG_ADDR_MODE2,
                            (uint8_t) ((PGA_GAIN_CODE << 4) | DATA_RATE_CODE));
        writeSingleRegister(REG_ADDR_INPMUX, INPMUX_FOR[PAIR_CH0]);
        (void) readSingleRegister(REG_ADDR_MODE0);
        (void) readSingleRegister(REG_ADDR_MODE1);
        (void) readSingleRegister(REG_ADDR_MODE2);
        (void) readSingleRegister(REG_ADDR_INPMUX);

        pair = PAIR_CH0;
        discardsLeft = SETTLE_DISCARDS;
        lastEdgeMs = millis();
        setSTART(HIGH);             // begin continuous conversions

        /* Note N12's final step: confirm the ADC really is on the external
         * clock. Refusing to initialise is the right failure -- coming up on
         * the internal oscillator means every number this board produces is
         * unsynchronised from its partner board, and nothing downstream can
         * tell. */
        if (!confirmExtClk()) {
            ESP_LOGE("ads1262", "could not confirm EXTCLK=1 -- either the ADC fell back to its "
                                "internal oscillator (check Y1, the clock wire, J8) or no "
                                "conversion arrived at all (check DRDY on the MISO pin)");
            return false;
        }

        initialized = true;
        everInitialized = true;
        return true;
    }

    /// Is anything actually driving `pin`? Pulls it up, then down, and sees
    /// whether it follows. MUST run before SPI.begin() claims the pin.
    ///
    /// This separates the two causes an all-zero readback cannot: a line nobody
    /// drives (isolator host side unpowered, or the wire is not landing) versus
    /// a line held low by something real.
    static void probePinDrive(const char *name, int8_t pin) {
        pinMode((uint8_t) pin, INPUT_PULLUP);
        delay(5);
        const int up = digitalRead((uint8_t) pin);
        pinMode((uint8_t) pin, INPUT_PULLDOWN);
        delay(5);
        const int down = digitalRead((uint8_t) pin);
        pinMode((uint8_t) pin, INPUT);

        const char *verdict;
        if (up == HIGH && down == LOW)
            verdict = "FLOATING -- nothing is driving it (unpowered isolator host "
                      "side, or the wire is not connected)";
        else if (up == LOW && down == LOW)
            verdict = "driven LOW by something";
        else if (up == HIGH && down == HIGH)
            verdict = "driven HIGH by something";
        else
            verdict = "indeterminate";
        Serial.printf("  %s (GPIO%d): pullup->%d pulldown->%d : %s\n", name, pin, up, down,
                      verdict);
    }

    /// Can this pin actually pull its net LOW, and what holds it when released?
    /// MUST run before SPI.begin() claims the pin.
    ///
    /// Open-drain on purpose: it can only pull down, so it cannot fight another
    /// driver and cannot damage anything if the net is externally driven. If the
    /// pin cannot drag the net low, either the pad's driver is dead or the net is
    /// shorted to a supply.
    static void probePinDriveOut(const char *name, int8_t pin) {
        pinMode((uint8_t) pin, OUTPUT_OPEN_DRAIN);
        digitalWrite((uint8_t) pin, LOW);
        delay(2);
        const int lo = digitalRead((uint8_t) pin);
        digitalWrite((uint8_t) pin, HIGH);   // release
        delay(2);
        const int released = digitalRead((uint8_t) pin);
        pinMode((uint8_t) pin, INPUT);

        const char *verdict;
        if (lo != LOW)
            verdict = "CANNOT PULL LOW -- pad driver dead, or the net is shorted to a supply";
        else if (released == HIGH)
            verdict = "pulls low OK, floats/pulls HIGH when released (normal for a pulled-up net)";
        else
            verdict = "pulls low OK, sits LOW when released";
        Serial.printf("  %s (GPIO%d): drive-low->%d released->%d : %s\n", name, pin, lo, released,
                      verdict);
    }

    /// Reads the ID register by BIT-BANGING the bus, with no SPI peripheral
    /// involved. MUST run before SPI.begin() claims the pins.
    ///
    /// This is the arbiter when the hardware bus reads all-zeros but every pin
    /// tests good: if bit-banging works, the wiring and the ADC are fine and the
    /// fault is in the SPI peripheral's configuration; if it also reads zeros,
    /// the fault is off-board.
    ///
    /// SPI mode 1 (CPOL=0, CPHA=1) by hand: clock idles low, MOSI changes on the
    /// rising edge, MISO is sampled on the falling edge. CS is held low
    /// throughout, as this board requires (sec.9.4.5).
    static void bitbangProbe(int8_t sck, int8_t mosi, int8_t miso, int8_t cs) {
        pinMode((uint8_t) sck, OUTPUT);
        pinMode((uint8_t) mosi, OUTPUT);
        pinMode((uint8_t) miso, INPUT);
        pinMode((uint8_t) cs, OUTPUT);

        digitalWrite((uint8_t) sck, LOW);
        digitalWrite((uint8_t) cs, LOW);     // held low for the whole session
        delayMicroseconds(10);

        auto xfer = [&](uint8_t out) -> uint8_t {
            uint8_t in = 0;
            for (int8_t b = 7; b >= 0; --b) {
                digitalWrite((uint8_t) sck, HIGH);                 // leading edge
                digitalWrite((uint8_t) mosi, (out >> b) & 1);      // CPHA=1: change here
                delayMicroseconds(2);
                digitalWrite((uint8_t) sck, LOW);                  // trailing edge
                in = (uint8_t) ((in << 1) | (digitalRead((uint8_t) miso) & 1));
                delayMicroseconds(2);
            }
            return in;
        };

        // RREG: opcode|addr, then (count-1), then one byte clocked out
        auto readReg = [&](uint8_t addr) -> uint8_t {
            xfer((uint8_t) (OPCODE_RREG + (addr & 0x1F)));
            xfer(0x00);
            return xfer(0x00);
        };

        const uint8_t id = readReg(REG_ADDR_ID);
        const uint8_t iface = readReg(REG_ADDR_INTERFACE);
        const uint8_t mode2 = readReg(REG_ADDR_MODE2);

        Serial.printf("  bit-bang read: ID=0x%02x INTERFACE=0x%02x MODE2=0x%02x\n", id, iface,
                      mode2);
        if (id == 0x00 && iface == 0x00 && mode2 == 0x00)
            Serial.println("    VERDICT: bit-bang also reads all zeros -- the fault is OFF-BOARD "
                           "(wiring, isolator, or the ADC), not the SPI peripheral.");
        else if (id == 0xFF && iface == 0xFF)
            Serial.println("    VERDICT: bit-bang reads all ones -- MISO idle high, nothing "
                           "driving it.");
        else
            Serial.println("    VERDICT: BIT-BANG WORKS. Wiring and ADC are fine; the hardware "
                           "SPI peripheral is the problem.");

        digitalWrite((uint8_t) cs, HIGH);
        pinMode((uint8_t) sck, INPUT);
        pinMode((uint8_t) mosi, INPUT);
        pinMode((uint8_t) cs, INPUT);
    }

    /// Clocks the bus continuously so it can be traced with a plain multimeter.
    ///
    /// Normal operation clocks in ~300 us bursts inside a 2 s retry window --
    /// about 0.01% duty, invisible without a triggered capture. Here SCLK and DIN
    /// run at ~50% duty, so every node on a live path reads about half of 3.3 V
    /// on a DMM, and a node that is stuck reads a hard 0 V or 3.3 V. Walk the
    /// path and the first stuck node is the break:
    ///     GPIO4 -> J2 SCLK -> U8 host-side in -> U8 isolated-side out -> U7 pin 11
    ///     GPIO5 -> J2 DIN  -> ...                                    -> U7 pin 12
    ///
    /// Bit-banged rather than hardware SPI so the pattern is a clean square wave
    /// with no idle gaps, and CS is held low as this board requires (sec.9.4.5).
    /// Never returns.
    [[noreturn]] static void busExerciser(int8_t sck, int8_t mosi, int8_t miso, int8_t cs,
                                          int8_t start) {
        pinMode((uint8_t) sck, OUTPUT);
        pinMode((uint8_t) mosi, OUTPUT);
        pinMode((uint8_t) miso, INPUT);
        pinMode((uint8_t) cs, OUTPUT);
        if (start >= 0) pinMode((uint8_t) start, OUTPUT);

        digitalWrite((uint8_t) cs, LOW);       // asserted, as in normal operation
        if (start >= 0) digitalWrite((uint8_t) start, HIGH);

        Serial.println();
        Serial.println("=== BUS EXERCISER -- continuous clocking, trace with a multimeter ===");
        Serial.printf("  SCLK GPIO%d and DIN GPIO%d: ~50%% duty, expect ~1.65 V on a live node\n",
                      sck, mosi);
        Serial.printf("  nCS GPIO%d held LOW (0 V), START GPIO%d held HIGH (3.3 V)\n", cs, start);
        Serial.println("  A node reading a hard 0 V or 3.3 V instead of ~1.65 V is the break.");
        Serial.printf("  DOUT GPIO%d is an input here -- if it reads ~1.65 V it is following DIN,\n"
                      "  which would mean U7 pins 12 and 13 are bridged.\n", miso);
        Serial.println("  Reset the board to leave this mode.");
        Serial.println();

        uint32_t lastReport = millis();
        uint32_t misoHigh = 0, samples = 0;
        bool dinLevel = false;
        for (;;) {
            /* SCLK and DIN both toggle, but at different rates, so they can be
             * told apart on a meter: SCLK ~50%, DIN ~50% at half the frequency. */
            for (int i = 0; i < 2; i++) {
                digitalWrite((uint8_t) sck, HIGH);
                delayMicroseconds(2);
                digitalWrite((uint8_t) sck, LOW);
                delayMicroseconds(2);
                misoHigh += (digitalRead((uint8_t) miso) == HIGH);
                ++samples;
            }
            /* Track DIN's level locally -- reading back an output pin is not
             * reliable on ESP32 once the input buffer is disabled. */
            dinLevel = !dinLevel;
            digitalWrite((uint8_t) mosi, dinLevel);

            if ((uint32_t) (millis() - lastReport) >= 3000) {
                lastReport = millis();
                /* Report what DOUT is doing. Following DIN shows up as roughly
                 * half the samples high; a stuck line shows as 0% or 100%. */
                Serial.printf("  [exerciser] DOUT high in %lu%% of %lu samples\n",
                              (unsigned long) (samples ? misoHigh * 100 / samples : 0),
                              (unsigned long) samples);
                misoHigh = samples = 0;
            }
        }
    }

    /// Bench diagnosis for a failed init(). Requires ads126xHalBegin() to have
    /// run (init() does that before it can fail on readback).
    ///
    /// Distinguishes the three things an all-zero readback can mean, which the
    /// mismatch message alone cannot:
    ///   - MISO stuck at a constant  -> nothing is driving DOUT. On this board
    ///     an unpowered isolated side idles DOUT HIGH (0xFF), while a floating
    ///     or unpowered HOST side of the isolator gives an undriven line that
    ///     typically reads 0x00 (note N7: VCC1 comes from the host on J2.1).
    ///   - scratch pattern survives   -> SPI is fine; the fault is elsewhere.
    ///   - scratch pattern lost       -> no working command path to the ADC.
    void diagnose() {
        Serial.println("--- ADS1262 probe ---");
        /* The config sweep below rewrites MODE1/MODE2/INPMUX/REFMUX and leaves
         * the part in the LAST row's configuration, restoring nothing. Invalidate
         * the zero-mode cache up front so a later enterZeroMode() reprograms
         * instead of averaging the sweep's leftover mux as "internal short". */
        zeroModeChop_ = 0xFF;

        /* Who owns each pin? A MISO pin that is not registered as
         * SPI_MASTER_MISO is not being sampled by the controller at all, and the
         * bus reads a constant 0x00 no matter what the wire is doing. */
        Serial.println("  pin ownership (want SCK/MOSI/MISO on the SPI bus):");
        const struct { const char *n; int8_t p; } pins[] = {
                {"SCLK", (int8_t) g_pinSck}, {"DIN ", (int8_t) g_pinMosi},
                {"DOUT", (int8_t) pinMiso_}, {"nCS ", (int8_t) g_pinCs},
        };
        for (auto &pp: pins) {
            if (pp.p < 0) continue;
            const peripheral_bus_type_t t = perimanGetPinBusType((uint8_t) pp.p);
            const char *ts = "OTHER";
            switch (t) {
                case ESP32_BUS_TYPE_INIT: ts = "unset"; break;
                case ESP32_BUS_TYPE_GPIO: ts = "GPIO (NOT on the SPI bus!)"; break;
                case ESP32_BUS_TYPE_SPI_MASTER_SCK: ts = "SPI SCK"; break;
                case ESP32_BUS_TYPE_SPI_MASTER_MISO: ts = "SPI MISO"; break;
                case ESP32_BUS_TYPE_SPI_MASTER_MOSI: ts = "SPI MOSI"; break;
                case ESP32_BUS_TYPE_SPI_MASTER_SS: ts = "SPI SS"; break;
                default: break;
            }
            Serial.printf("    %s GPIO%-2d : %s\n", pp.n, pp.p, ts);
        }

        /* Positive/negative comms test. OFCAL0 is a plain scratch register with
         * no side effects at these values (offset calibration, restored to 0). */
        static constexpr uint8_t SCRATCH = REG_ADDR_OFCAL0;
        const uint8_t pats[] = {0xA5, 0x5A};
        uint8_t survived = 0;
        for (uint8_t p: pats) {
            writeSingleRegister(SCRATCH, p);
            const uint8_t got = readSingleRegister(SCRATCH);
            Serial.printf("  scratch OFCAL0: wrote 0x%02x read 0x%02x %s\n", p, got,
                          got == p ? "OK" : "MISMATCH");
            if (got == p) ++survived;
        }
        writeSingleRegister(SCRATCH, 0x00);

        Serial.println("  register dump 0x00..0x14:");
        uint8_t all_and = 0xFF, all_or = 0x00;
        for (uint8_t a = 0; a <= REG_ADDR_GPIODAT; a++) {
            const uint8_t v = readSingleRegister(a);
            all_and &= v;
            all_or |= v;
            Serial.printf("    0x%02x = 0x%02x\n", a, v);
        }

        Serial.printf("  halFaults=%u\n", (unsigned) ads126xHalFaults());
        if (all_or == 0x00)
            Serial.println("  VERDICT: every register reads 0x00 -- MISO never driven. "
                           "Most likely the isolator's HOST side has no power: J2.1 is a 3V3 "
                           "INPUT fed by the host (note N7), and there is no on-board source "
                           "on that net. Check J2.1 (3V3) and the J2 ground, then J1's 5V brick.");
        else if (all_and == 0xFF)
            Serial.println("  VERDICT: every register reads 0xFF -- DOUT idle high, which is "
                           "what an UNPOWERED ISOLATED SIDE looks like (note N6). Check J1.");
        else if (survived == 2)
            Serial.println("  VERDICT: SPI command path WORKS (scratch pattern survived). "
                           "The init mismatch is a register/config problem, not wiring.");
        else
            Serial.println("  VERDICT: mixed readback -- partial comms. Suspect SCLK integrity, "
                           "SPI mode, or a marginal connection.");
        /* Conversion-level probe. Register access lives in the DVDD/digital
         * domain; conversions need AVDD/AVSS. If registers read back correctly
         * but ADC1 never reports new data, the analog rails are the suspect,
         * not the interface. */
        Serial.println("  conversion probe (START high, ~1 s of RDATA1):");
        setSTART(HIGH);
        uint8_t sawAdc1 = 0, sawExtClk = 0, n = 0;
        for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < 1000; n++) {
            uint8_t status = 0, checksum = 0, data[4] = {0};
            const int32_t code = readData(&status, data, &checksum);
            if (n < 6)
                Serial.printf("    STATUS=0x%02x [ADC1=%d EXTCLK=%d RESET=%d REFALM=%d "
                              "PGAL=%d PGAH=%d PGAD=%d] code=%ld chk=0x%02x(want 0x%02x)\n",
                              status, !!(status & STATUS_ADC1), !!(status & STATUS_EXTCLK),
                              !!(status & STATUS_RESET), !!(status & STATUS_REF_ALM),
                              !!(status & STATUS_PGAL_ALM), !!(status & STATUS_PGAH_ALM),
                              !!(status & STATUS_PGAD_ALM), (long) code, checksum,
                              calculateChecksum(data, 4));
            if (status & STATUS_ADC1) ++sawAdc1;
            if (status & STATUS_EXTCLK) ++sawExtClk;
            delay(20);
        }
        Serial.printf("    of %u reads: ADC1-new=%u EXTCLK=%u\n", n, sawAdc1, sawExtClk);

        /* If the START PIN produced nothing, retry with the START1 OPCODE. The
         * two reach the converter by different routes -- the pin crosses a
         * forward isolator channel from J2.8, the opcode rides the SPI path we
         * have already proven works. Splitting them separates "START never
         * arrives" from "the analog core cannot convert". */
        uint8_t cmdAdc1 = 0;
        if (!sawAdc1) {
            Serial.println("    retrying with the START1 OPCODE instead of the pin:");
            setSTART(LOW);
            sendCommand(OPCODE_STOP1);
            sendCommand(OPCODE_START1);
            for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < 1000;) {
                uint8_t status = 0, checksum = 0, data[4] = {0};
                readData(&status, data, &checksum);
                if (status & STATUS_ADC1) ++cmdAdc1;
                delay(20);
            }
            Serial.printf("    opcode-started: ADC1-new=%u\n", cmdAdc1);
        }

        if (!sawAdc1 && cmdAdc1)
            Serial.println("    VERDICT: converts on the START1 OPCODE but NOT on the START "
                           "PIN -- the pin path is broken. Check GPIO16 -> J2.8 and its "
                           "forward isolator channel.");
        else if (!sawAdc1)
            Serial.println("    VERDICT: registers OK and EXTCLK=1, but ADC1 NEVER reports new "
                           "data by pin OR command. EXTCLK only proves a clock EXISTS -- read "
                           "the clock estimate below for its RATE before blaming anything else. "
                           "If the rate is sane, suspect the analog rails (AVDD-AVSS must be "
                           "4.75..5.25 V).");
        else if (!sawExtClk)
            Serial.println("    VERDICT: converting, but on the INTERNAL oscillator. "
                           "Check Y1, J8 and the clock path (schematic note N10).");
        else
            Serial.println("    VERDICT: converting on the external clock -- healthy.");

        /* Config sweep. Each row reaches the converter by a different route, so
         * whichever rows convert localises the fault:
         *   - the internal AVDD/DVDD monitors need NO external input wiring, so
         *     they separate "the analog core is dead" from "our inputs are bad";
         *   - sinc1 at 38400 SPS makes a conversion ~26 us nominal instead of
         *     52 ms, so it still completes quickly even if fCLK is far slower
         *     than the 7.3728 MHz we assume (EXTCLK=1 proves a clock EXISTS, it
         *     says nothing about its FREQUENCY);
         *   - PGA bypassed removes the input-window constraint entirely. */
        Serial.println("  config sweep (3 s each, looking for ANY conversion):");
        struct Try { const char *name; uint8_t mode1, mode2, inpmux, refmux; uint8_t gain; };
        /* REFMUX 0x00 = internal 2.5 V reference; 0x24 = AVDD/AVSS as the
         * reference, which bypasses the internal reference entirely. If a row
         * converts ONLY with 0x24, the internal reference is dead -- REFOUT
         * (pin 8) sits next to AVSS (pin 7) and note N3 wants 1 uF between
         * them, so a short or a damaged cap there is the obvious cause. */
        static constexpr uint8_t REF_INT = 0x00;
        static constexpr uint8_t REF_AVDD =
                (uint8_t) (REFMUX_RMUXP_INT_AVDD | REFMUX_RMUXN_INT_AVSS);
        static const Try tries[] = {
                {"sinc1 38400SPS PGAbyp AVDDmon REFint ", MODE1_FILTER_SINC1,
                 (uint8_t) (0x80 | MODE2_DR_38400_SPS),
                 (uint8_t) (INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N), REF_INT, 1},
                {"sinc1 38400SPS PGAbyp AVDDmon REFavdd", MODE1_FILTER_SINC1,
                 (uint8_t) (0x80 | MODE2_DR_38400_SPS),
                 (uint8_t) (INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N), REF_AVDD, 1},
                {"sinc1 20SPS   PGAbyp AIN0/1  REFavdd", MODE1_FILTER_SINC1,
                 (uint8_t) (0x80 | MODE2_DR_20_SPS), 0x01, REF_AVDD, 1},
                {"sinc1 20SPS   PGAbyp TEMPmon REFavdd", MODE1_FILTER_SINC1,
                 (uint8_t) (0x80 | MODE2_DR_20_SPS),
                 (uint8_t) (INPMUX_MUXP_TEMP_P | INPMUX_MUXN_TEMP_N), REF_AVDD, 1},
        };

        for (auto &t: tries) {
            setSTART(LOW);
            writeSingleRegister(REG_ADDR_MODE1, t.mode1);
            writeSingleRegister(REG_ADDR_MODE2, t.mode2);
            writeSingleRegister(REG_ADDR_INPMUX, t.inpmux);
            writeSingleRegister(REG_ADDR_REFMUX, t.refmux);
            sendCommand(OPCODE_STOP1);
            setSTART(HIGH);
            sendCommand(OPCODE_START1);

            uint32_t hits = 0;
            int32_t lastCode = 0;
            uint8_t lastStatus = 0;
            for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < 3000;) {
                uint8_t status = 0, checksum = 0, data[4] = {0};
                const int32_t code = readData(&status, data, &checksum);
                lastStatus = status;
                if (status & STATUS_ADC1) { ++hits; lastCode = code; }
                delay(5);
            }
            const float v = (float) lastCode * (VREF / (float) t.gain) / (float) (2u << 30);
            Serial.printf("    %s : ADC1=%lu STATUS=0x%02x code=%ld v=%+.4f V",
                          t.name, (unsigned long) hits, lastStatus, (long) lastCode, v);
            /* The analog supply monitor reads (AVDD-AVSS)/4 [SBAS661C sec.9.3.4]. */
            if (t.inpmux == (INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N) && hits)
                Serial.printf("  => AVDD-AVSS = %.3f V", v * 4.0f);
            Serial.println();
        }
        Serial.println("  (if EVERY row shows ADC1=0 the converter itself is not running; "
                       "if only the fast row converts, fCLK is far below 7.3728 MHz)");

        /* Time real conversions to infer fCLK. Every ADS1262 timing scales with
         * the clock, so measured_rate / nominal_rate is exactly fCLK / 7.3728 MHz.
         * EXTCLK only proves a clock EXISTS -- this is what proves its RATE. */
        Serial.println("  clock estimate (sinc1 @ nominal 38400 SPS, 6 s):");
        setSTART(LOW);
        writeSingleRegister(REG_ADDR_MODE1, MODE1_FILTER_SINC1);
        writeSingleRegister(REG_ADDR_MODE2, (uint8_t) (0x80 | MODE2_DR_38400_SPS));
        writeSingleRegister(REG_ADDR_INPMUX,
                            (uint8_t) (INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N));
        sendCommand(OPCODE_STOP1);
        setSTART(HIGH);
        sendCommand(OPCODE_START1);

        uint32_t first = 0, last = 0, count = 0;
        for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < 6000;) {
            uint8_t status = 0, checksum = 0, data[4] = {0};
            readData(&status, data, &checksum);
            if (status & STATUS_ADC1) {
                const uint32_t now = millis();
                if (!count) first = now;
                last = now;
                ++count;
            }
        }
        if (count >= 2) {
            const float sps = (float) (count - 1) * 1000.0f / (float) (last - first);
            const float fclk = 7.3728e6f * (sps / 38400.0f);
            Serial.printf("    %lu conversions, measured %.3f SPS vs 38400 nominal\n",
                          (unsigned long) count, sps);
            Serial.printf("    => fCLK ~ %.0f Hz (expected 7372800 Hz, ratio 1/%.0f)\n", fclk,
                          38400.0f / sps);
            if (fclk < 1.0e6f)
                Serial.println("    VERDICT: the clock reaching XTAL1 is FAR below the 1 MHz "
                               "minimum (SBAS661C sec.7.3 allows 1..8 MHz). Check Y1 is "
                               "oscillating, its +3V3P supply, J8, and isolator channel E.");
        } else {
            Serial.printf("    only %lu conversions in 6 s -- too few to time\n",
                          (unsigned long) count);
        }

        Serial.println("--- end probe ---");
    }

    /// Decodes the STATUS byte to the console.
    ///
    /// Worth having precisely when nothing else works: the status byte rides every
    /// RDATA1 response, so it is readable WITHOUT a successful conversion. When
    /// the converter is producing nothing, this is the only thing the part will
    /// still tell us about its own state.
    ///
    /// EXTCLK (bit 5) answers "which clock is it actually using", which is not the
    /// same question as "is a clock present at XTAL1" -- sec.9.4.9 switches to the
    /// internal oscillator automatically when no external clock is detected, so
    /// EXTCLK = 0 means the external clock is gone AND the part is coping. The PGA
    /// and reference alarms are the useful ones for a dead converter: they are
    /// driven by the ANALOG rails and are set without any conversion having to
    /// succeed.
    void reportStatusByte(const char *who) {
        uint8_t status = 0, checksum = 0, data[4] = {0};
        (void) readData(&status, data, &checksum);
        ESP_LOGE("ads1262", "%s: STATUS=0x%02x -- clock is %s%s%s%s%s%s%s", who, status,
                 /* The PAIRING with "no conversions" is what diagnoses, not the bit
                  * alone -- and the two cases point in OPPOSITE directions:
                  *
                  *   EXTCLK=0 + no conversions: cannot happen. sec.9.4.9 selects the
                  *     internal 7.3728 MHz automatically when no external clock is
                  *     detected, so the part would be converting. Suspect the part.
                  *   EXTCLK=1 + no conversions: a clock IS present and the part has
                  *     latched onto it, and it is NOT USABLE. Frequency or levels at
                  *     XTAL1 -- not the supplies.
                  *
                  * Measured twice on this board (2026-08-15): STATUS=0x20 with zero
                  * conversions, supplies healthy at AVDD-AVSS = 5.013 V, and the fault
                  * was the crystal. EXTCLK=1 is the STRONGER clock indictment of the
                  * two, which is the opposite of how it reads. */
                 (status & STATUS_EXTCLK)
                         ? "EXTERNAL -- if there are NO CONVERSIONS this indicts the CLOCK, not "
                           "the supplies: the part latched onto something at XTAL1 that is not a "
                           "usable clock. Check frequency and levels there"
                         : "INTERNAL (no external clock detected; sec.9.4.9 fell back "
                           "automatically -- conversions should still run, so a missing clock "
                           "is NOT why they stopped)",
                 (status & STATUS_REF_ALM) ? ", REF_ALM: VREF <= 0.4 V" : "",
                 (status & STATUS_PGAL_ALM) ? ", PGAL_ALM: PGA output below AVSS+0.2 V" : "",
                 (status & STATUS_PGAH_ALM) ? ", PGAH_ALM: PGA output above AVDD-0.2 V" : "",
                 (status & STATUS_PGAD_ALM) ? ", PGAD_ALM: differential over-range" : "",
                 (status & STATUS_RESET) ? ", RESET: device has reset since this bit was cleared"
                                         : "",
                 (status & (STATUS_REF_ALM | STATUS_PGAL_ALM | STATUS_PGAH_ALM | STATUS_PGAD_ALM))
                         ? " <- an ANALOG rail or the reference is the suspect"
                         /* NOT "no alarms, rails are fine". sec.9.2: the reference and
                          * PGA alarms are "latched during the conversion phase and
                          * appended to the conversion data" -- with no conversions they
                          * CANNOT be set, so clear bits here are unevaluated, not good
                          * news. Reporting them as reassurance would hide a dead rail
                          * exactly when the converter has stopped. */
                         : ", alarm bits UNEVALUATED (they latch during a conversion; "
                           "with none they cannot set -- this is not a clean bill of health)");
    }

    /// Logs the filter, rate, chop state and gain ACTUALLY IN FORCE, decoded from
    /// register values -- never from a literal, and never from what we intended
    /// to write.
    ///
    /// The init banner used to print the fixed string "FIR @ 20 SPS". That was a
    /// statement about the production default, not about the hardware:
    /// PowerSampler_ShuntAdcZero reprograms the part to sinc4 @ 10 SPS chopped
    /// within a second of boot, so the single line the log offered about the
    /// converter's configuration named the one configuration it was NOT running.
    /// Pass registers that have been READ BACK, so this cannot repeat.
    static void logConfig(const char *who, uint8_t mode0, uint8_t mode1, uint8_t mode2) {
        static const char *const kFilter[8] = {"sinc1", "sinc2", "sinc3", "sinc4",
                                               "FIR", "?5", "?6", "?7"};
        static const float kSps[16] = {2.5f, 5, 10, 16.6f, 20, 50, 60, 100,
                                       400, 1200, 2400, 4800, 7200, 14400, 19200, 38400};
        const uint8_t filt = (uint8_t) ((mode1 >> 5) & 0x07);
        const uint8_t dr = (uint8_t) (mode2 & 0x0F);
        const uint8_t gainCode = (uint8_t) ((mode2 >> 4) & 0x07);
        const bool chop = (mode0 & MODE0_CHOP_ON) != 0;

        /* CHANGE DETECTOR, not a heartbeat. applyProductionConfig() runs after
         * every die-temperature read (TEMP_EVERY_N_SCANS), so logging every call
         * put ~4 lines/s on the console -- measured on the bench, and enough to
         * interleave with and visibly corrupt other output mid-line.
         *
         * Suppressing REPEATS costs no diagnostic value: an unchanged
         * configuration is the thing that was already reported. A configuration
         * that DIFFERS is the event worth seeing, and it still prints -- including
         * the first one after boot, and including a wrong one, which prints once
         * and loudly rather than never. Deliberately keyed on the READ-BACK
         * registers, so a part that silently drifts off its configuration still
         * announces itself. */
        static uint8_t lastM0 = 0, lastM1 = 0, lastM2 = 0;
        static const char *lastWho = nullptr;
        static bool haveLast = false;
        if (haveLast && mode0 == lastM0 && mode1 == lastM1 && mode2 == lastM2 &&
            who == lastWho)
            return;
        lastM0 = mode0; lastM1 = mode1; lastM2 = mode2; lastWho = who; haveLast = true;

        /* With chop the NOMINAL rate is not the throughput: Equation 21 makes the
         * chopped data rate 1/td(STDR), which for sinc4 @ 10 SPS is 2.5, not 10.
         * Say so rather than print a rate the part will not deliver -- computing
         * it would need the whole of Table 9-13 in flash. */
        ESP_LOGI("ads1262", "%s: %s @ %.1f SPS%s, chop %s, G=%u%s", who, kFilter[filt], kSps[dr],
                 chop ? " nominal (chopped throughput = 1/td(STDR), Table 9-13)" : "",
                 chop ? "ON" : "off", (unsigned) (1u << gainCode),
                 (mode2 & 0x80) ? " -- PGA BYPASSED" : "");
    }

    /// Restores the measurement configuration the self-tests and the temperature
    /// read overwrite. Reads every register back: upstream's writeSingleRegister()
    /// does NOT update the driver's shadow map, only readSingleRegister() does.
    void applyProductionConfig() {
        zeroModeChop_ = 0xFF;   // configuration changed; zero mode must be re-entered
        drdyTimeoutMs_ = DRDY_TIMEOUT_MS;   // back to the FIR @ 20 SPS figure
        setSTART(LOW);
        writeSingleRegister(REG_ADDR_MODE0, MODE0_VALUE);
        writeSingleRegister(REG_ADDR_MODE1, MODE1_VALUE);
        writeSingleRegister(REG_ADDR_MODE2, (uint8_t) ((PGA_GAIN_CODE << 4) | DATA_RATE_CODE));
        writeSingleRegister(REG_ADDR_REFMUX, 0x00);
        writeSingleRegister(REG_ADDR_INPMUX, INPMUX_FOR[PAIR_CH0]);
        const uint8_t m0 = readSingleRegister(REG_ADDR_MODE0);
        const uint8_t m1 = readSingleRegister(REG_ADDR_MODE1);
        const uint8_t m2 = readSingleRegister(REG_ADDR_MODE2);
        (void) readSingleRegister(REG_ADDR_REFMUX);
        (void) readSingleRegister(REG_ADDR_INPMUX);
        logConfig("production config", m0, m1, m2);

        /* Leave the ADC CONVERTING. This function drops START to write the
         * registers, and every caller wants the measurement configuration back
         * in its running state -- a restore that leaves the converter stopped is
         * not a restore. Omitting this stalled the sampler after the chop
         * comparison: START stayed low, conversions stopped, and pump() consumed
         * one stale DRDY and then silently saw STATUS_ADC1 clear forever
         * (scans=0, accepted=1). */
        setSTART(HIGH);
    }

    /// Reads the internal temperature sensor. SBAS661C Equation 9:
    ///   T(degC) = [(reading_uV - 122400) / 420] + 25
    /// sec.9.3.4 requires the PGA enabled at gain 1, chop disabled and the
    /// internal reference powered -- configureSelfTest() does the first two and
    /// POWER.INTREF stays set from the measurement configuration.
    ///
    /// Leaves dieTempC_ untouched on failure rather than writing a fabricated
    /// value; the caller keeps publishing the last good reading, and NAN until
    /// the first success.
    bool readDieTemperature() { return refreshHealth(); }

    /// Reads die temperature AND both internal supply monitors in ONE excursion.
    ///
    /// The expensive part of reading an internal monitor is the round trip out of
    /// and back into the measurement configuration -- on the zero channel that is
    /// sinc4's ~400 ms filter refill (800 ms chopped), measured on the bench. The
    /// conversions themselves are ~0.83 ms each at SELFTEST_DR. So once we are
    /// out here, two more monitors cost ~2 ms and NO additional refill; reading
    /// them separately would have cost a second full round trip.
    ///
    /// Each field is left untouched on its own failure rather than being written
    /// with a fabricated value: a stale reading is visibly stale in the series,
    /// while a plausible invented one is not. Returns false if ANY read failed.
    bool refreshHealth() {
        constexpr float lsb1 = VREF / (float) (2u << 30);   // gain = 1 in self-test config
        int32_t code = 0;
        bool ok = true;

        /* Captured BEFORE the first configureSelfTest(), which invalidates the
         * zero-mode cache -- by the end of this function these members no longer
         * say what the caller was running. */
        const uint8_t wasChop = zeroModeChop_;
        const uint8_t wasMode1 = zeroModeMode1_;
        const uint8_t wasDr = zeroModeDr_;

        configureSelfTest((uint8_t) (INPMUX_MUXP_TEMP_P | INPMUX_MUXN_TEMP_N));
        if (oneConversion(code)) {
            const float uV = (float) code * lsb1 * 1e6f;
            dieTempC_ = (uV - 122400.0f) / 420.0f + 25.0f;   // SBAS661C Equation 9
        } else ok = false;

        /* V_ANLMON = (AVDD - AVSS)/4 and V_DIGMON = (DVDD - DGND)/4, hence x4.
         * This is the monitoring the board cannot do any other way -- there is no
         * independent sense path, so when the converter dies these go with it.
         * Recording them continuously gives the APPROACH to a failure even though
         * the failure itself is unmeasurable. */
        /* Stop on the first failure. oneConversion() returns false on a device
         * reset (having cleared `initialized`), and there is no point programming
         * the mux of a part that has just told us its configuration is gone --
         * the re-init will rewrite everything anyway. */
        if (ok) {
            configureSelfTest((uint8_t) (INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N));
            if (oneConversion(code)) avddAvss_ = (float) code * lsb1 * 4.0f; else ok = false;
        }
        if (ok) {
            configureSelfTest((uint8_t) (INPMUX_MUXP_DVDD_P | INPMUX_MUXN_DVDD_N));
            if (oneConversion(code)) dvdd_ = (float) code * lsb1 * 4.0f; else ok = false;
        }

        /* Restore unconditionally -- even after a partial failure, or the next
         * measurement runs in the self-test configuration (gain 1, sinc1, temp
         * mux) and reads nonsense -- but restore the configuration the CALLER was
         * in, not always the production one.
         *
         * A zero-channel caller used to get applyProductionConfig(): FIR @ 20 SPS
         * programmed and started, in a configuration nothing reads, only for the
         * next sample to reprogram zero mode. Two full register rounds per sample
         * where one will do, and visible in the log as a
         * "production config" / "zero mode" pair after every reading.
         *
         * The settling discards are NOT saved and must not be: the self-test
         * excursion really did change gain, mux and filter, so sinc4's filter has
         * to refill either way. What is saved is programming and running a third
         * configuration in between. */
        if (wasChop != 0xFF) {
            /* Force a reprogram: configureSelfTest() cleared the cache, so passing
             * the saved triple would otherwise be a no-op against a part that is
             * still in the self-test configuration. */
            zeroModeChop_ = 0xFF;
            enterZeroMode(wasChop, wasMode1, wasDr);
        } else {
            applyProductionConfig();
        }
        return ok;
    }

    /// Bench experiment: the ADC's own zero, chop OFF vs chop ON.
    ///
    /// Measured on the INTERNAL shorted input (AINCOM/AINCOM) at the production
    /// gain, so it is input-referred exactly like the real channels and needs no
    /// external wiring -- with J3..J6 open, a real channel would be measuring
    /// floating-input drift rather than offset.
    ///
    /// Reports what the 10 ppm budget actually needs: mean (the offset that does
    /// NOT cancel in the efficiency ratio), sigma (noise), and the achieved data
    /// rate, since chop pays for its offset rejection in time -- SBAS661C
    /// sec.9.4.2 doubles the first-conversion latency, and this scan restarts
    /// conversions on every mux change.
    void chopExperiment(uint16_t samples = 64) {
        Serial.println();
        Serial.println("=== chop experiment: internal shorted input, G=32 ===");
        Serial.printf("  %u samples per mode; 10 ppm of a 35 mV signal is 0.35 uV\n",
                      (unsigned) samples);

        float offUv = NAN, offSd = NAN, offSps = NAN;
        float onUv = NAN, onSd = NAN, onSps = NAN;
        const bool okOff = measureZero(MODE0_CHOP_OFF, samples, offUv, offSd, offSps);
        const bool okOn = measureZero(MODE0_CHOP_ON, samples, onUv, onSd, onSps);

        Serial.println("  mode      mean(uV)   sigma(uV)   rate(SPS)");
        if (okOff) Serial.printf("  chop off  %+9.4f  %9.4f  %9.2f\n", offUv, offSd, offSps);
        else Serial.println("  chop off  MEASUREMENT FAILED");
        if (okOn) Serial.printf("  chop on   %+9.4f  %9.4f  %9.2f\n", onUv, onSd, onSps);
        else Serial.println("  chop on   MEASUREMENT FAILED");

        if (okOff && okOn) {
            Serial.printf("  offset |mean| %.4f -> %.4f uV  (%.1fx)\n", fabsf(offUv),
                          fabsf(onUv), fabsf(onUv) > 0 ? fabsf(offUv) / fabsf(onUv) : INFINITY);
            Serial.printf("  noise  sigma  %.4f -> %.4f uV  (%.2fx, datasheet says 1.4x)\n",
                          offSd, onSd, onSd > 0 ? offSd / onSd : INFINITY);
            Serial.printf("  rate          %.2f -> %.2f SPS  (%.2fx)\n", offSps, onSps,
                          onSps > 0 ? offSps / onSps : INFINITY);
            /* The scan restarts conversions per pair, so each pair pays the
             * first-conversion latency -- that is what sets multiplex skew. */
            Serial.printf("  => 4-pair scan ~%.0f ms -> ~%.0f ms, so multiplex skew %.1fx\n",
                          4000.0f / offSps, 4000.0f / onSps,
                          onSps > 0 ? offSps / onSps : INFINITY);
        }
        Serial.println("=== end chop experiment ===");
        Serial.println();

        applyProductionConfig();
    }

    /// One averaged reading of the INTERNAL short (AINCOM/AINCOM) with chop
    /// explicitly ON, at the production GAIN but sinc4 @ 10 SPS rather than the
    /// production FIR @ 20 SPS -- see ZERO_MODE1_VALUE for why.
    ///
    /// Measures the ADC ONLY: an internal short excludes the external path, so it
    /// says nothing about the system offset (delta-epsilon) that needs the
    /// terminals shorted at the shunt.
    ///
    /// CHOP WAS OFF until 2026-08-15. That run answered its question: the offset
    /// took ~2 h to settle onto 7.2525 uV, rising 104 nV, and the die temperature
    /// could not explain it -- the same 0.6 C change produced 104 nV during the
    /// transient and 2.4 nV after it. 104 nV is ~3 ppm of a 35 mV full scale,
    /// nearly a third of the 10 ppm budget, spent before the first reading.
    /// Chop is the term that removes it (VOS +-0.1/G uV typ, i.e. +-3.1 nV at
    /// G=32, and drift 1 nV/C against 30/G+10 = 10.9). Chop ON now measures what
    /// is LEFT after that correction.
    ///
    /// THROUGHPUT COST, and it is not small here: SBAS661C Equation 21 makes the
    /// chopped data rate 1/td(STDR), so it is settling time -- not the nominal
    /// rate -- that sets throughput. sinc4 @ 10 SPS has td(STDR) = 400.4 ms, so
    /// this runs at 2.5 SPS, not 10. FIR @ 20 SPS (td 52.22 ms -> 19.15 SPS)
    /// would give BETTER noise per unit time under chop, 6.9 nV/sqrt(s) against
    /// 8.2 -- the sinc4 choice only wins with chop OFF. Kept on sinc4 so this run
    /// differs from the previous one in exactly one variable.
    ///
    /// Returns false rather than partial statistics if the samples cannot be
    /// gathered.
    bool zeroDriftSample(uint16_t n, float &meanUv, float &sdUv, float &sps) {
        return measureZero(MODE0_CHOP_ON, n, meanUv, sdUv, sps,
                           ZERO_MODE1_VALUE, ZERO_DATA_RATE_CODE);
    }

    /// Reads one conversion at most, advancing the mux when it is accepted.
    /// Returns true if a reading was stored.
    bool pump() {
        if (!initialized) return false;

        /* BEFORE waitForEdge(), deliberately. If conversions have stopped, every
         * path below returns early, so a freshness check placed after the wait
         * would itself stop running exactly when it is needed. */
        checkRateFreshness();

        if (!waitForEdge()) return false;

        uint8_t status = 0, checksum = 0, data[4] = {0};
        const uint32_t faultsBefore = ads126xHalFaults();

        /* sec.9.4.6: no serial activity may occur between DRDY going low and the
         * readback, or the data are invalid. So this is the FIRST bus access
         * after the edge -- no register polling in between. */
        int32_t count = readData(&status, data, &checksum);
        ++sinceRestart_;

        if (ads126xHalFaults() != faultsBefore) {
            raiseFault(DIAG_HAL_FAULT, 0);
            ESP_LOGW("ads1262", "HAL fault during read");
            return false;
        }

        /* NOT-READY, WHICH IS NOT CORRUPTION. Restarting the conversion cycle
         * (selectPair()'s START toggle) puts an edge on the shared DOUT/nDRDY
         * pin, and with DRDY tied to MISO (note N6) the ISR cannot tell that
         * edge from a data-ready one. So the first read after every restart asks
         * a part that has nothing yet, and it answers with an ALL-ZERO frame:
         * STATUS with ADC1 clear, four zero data bytes, and a zero checksum
         * byte. Zero is not the checksum of four zero bytes -- that is 0x9B --
         * so this used to be reported as a wire fault.
         *
         * It was ~2 messages/second and 100% false: 95 of 95 mismatches captured
         * on the bench (2026-08-16) were byte-identical to this, every one of
         * them the FIRST read after a restart. It was never seen before because
         * the previously registered channel published at 2.5 SPS instead of 19,
         * so the same event fired ~30x less often.
         *
         * Recognised NARROWLY -- all four data bytes zero AND checksum zero AND
         * ADC1 clear -- so that a real corrupt frame still reaches the loud path
         * below. Silence here is safe because it cannot persist: if the part
         * genuinely stopped converting, waitForEdge()'s drdyTimeoutMs_ re-arm
         * and the DRDY timeout are the independent detectors, and neither is
         * touched by this. */
        /* STATUS_EXTCLK is REQUIRED for the quiet path, and it is the whole
         * safety of it. A stuck-low MISO reads every byte as zero -- status
         * included -- which matches "all-zero payload" perfectly. Because DRDY
         * rides that same pin (note N6), a stuck-low line ALSO reads as
         * permanent data-ready, so the ISR would fire forever, pump() would
         * swallow zeros forever, and the channel would go silent with no
         * warning: absence of evidence encoding absence of the problem. A
         * healthy not-ready frame proves the part is alive by still carrying
         * EXTCLK=1 (measured: STATUS=0x20 on all 95). A frame with EXTCLK clear
         * falls through to the loud paths below instead, which is also what
         * keeps the note N10 internal-oscillator check reachable.
         *
         * The status byte must equal STATUS_EXTCLK EXACTLY, not merely contain
         * it. STATUS_RESET is 0x01, so a reset-flagged not-ready frame is 0x21 --
         * and a `& STATUS_EXTCLK` test would swallow it. That matters more than
         * it looks: the device-reset check below is what clears `initialized`
         * and triggers re-initialisation, and every path out of this branch
         * returns before reaching it. Quietly eating 0x21 would leave a part
         * that had lost its configuration running forever with nobody
         * re-programming it. Requiring the exact byte also excludes the PGA
         * alarm bits for the same reason.
         *
         * sinceRestart_ == 1 confines this to the FIRST read after a restart,
         * which is the only place it was ever observed (95 of 95). Because
         * sinceRestart_ increments on every read and resets only in
         * selectPair(), at most one frame per restart can take the quiet path --
         * a stuck DRDY or a stalled converter produces a second, third, fourth
         * frame that no longer matches and goes loud. That is a stronger bound
         * than a consecutive-run counter, and it needs no extra state. */
        if (status == STATUS_EXTCLK && checksum == 0 && sinceRestart_ == 1 &&
            !(data[0] | data[1] | data[2] | data[3])) {
            ++notReadyReads_;
            ESP_LOGD("ads1262", "not-ready frame after restart (#%u)", (unsigned) notReadyReads_);
            return false;
        }

        if (checksum != calculateChecksum(data, 4)) {
            raiseFault(DIAG_CHECKSUM, count);
            /* The BYTES, not just the verdict. "checksum mismatch" alone cannot
             * distinguish a too-early read (STATUS with ADC1 clear, data still
             * zero) from a bit error on the wire (one byte off) from a framing
             * slip (everything shifted). Those have completely different fixes,
             * and the bare message sent one round of debugging down the wrong
             * one. sinceRestart counts conversions since the last selectPair(). */
            ESP_LOGW("ads1262", "checksum mismatch: STATUS=0x%02x ADC1=%d data=%02x %02x %02x %02x "
                                "chk got=0x%02x want=0x%02x sinceRestart=%u",
                     status, !!(status & STATUS_ADC1), data[0], data[1], data[2], data[3],
                     checksum, calculateChecksum(data, 4), (unsigned) sinceRestart_);
            return false;
        }

        if (status & STATUS_RESET) {
            raiseFault(DIAG_DEVICE_RESET, count);
            ESP_LOGE("ads1262", "device reset flag set -- configuration lost, re-init needed");
            initialized = false;    // force a full re-init rather than trusting the part
            return false;
        }

        /* THE mandatory per-acquisition check (note N10). Absence of the
         * external clock must never read as "clock fine". */
        if (!(status & STATUS_EXTCLK)) {
            raiseFault(DIAG_NO_EXTCLK, count);
            ESP_LOGE("ads1262", "EXTCLK=0: ADC on internal oscillator, data UNSYNCHRONISED "
                                "-- refusing sample");
            return false;
        }

        if (!(status & STATUS_ADC1))
            return false;           // not a fresh conversion; re-reading would bias averages

        /* A genuine conversion arrived: it is one tick of the clock guard. */
        noteConversionForRate();

        /* OVER-RANGE. These alarms are "latched during the conversion phase and
         * appended to the conversion data" (sec.9.2), so they qualify THIS
         * conversion -- which is why the test sits after the STATUS_ADC1 gate,
         * where the byte describes a real conversion, and not before it.
         *
         * Until now they were decoded only in the no-conversions fault dump, so
         * in normal operation a clipped reading was accepted as an ordinary one.
         * That is the dangerous shape: a transient past the input range would
         * have published a pinned value that still looks like a perfectly
         * plausible current. Nothing else in pump() would have noticed.
         *
         * The value is DISCARDED (volts_ goes NAN, so nothing clipped is ever
         * published as a current) but the conversion still COUNTS as completing
         * this pair. An earlier version returned here instead, and that was
         * wrong in two ways, both seen on the bench (2026-08-16, PGAL_ALM from a
         * common-mode excursion):
         *   - the mux never advanced, so in multi-pair mode ONE bad pair
         *     silenced the other three;
         *   - generation_ never moved, so diagPublished_ never updated and
         *     DIAG_PGA_RANGE could not reach telemetry at all. The channel just
         *     went quiet, which is the reading a consumer cannot distinguish
         *     from "nothing to report".
         * diagPublished_ is now set DIRECTLY as well, so dev.diag() surfaces the
         * fault immediately -- PowerSampler_ShuntAdcHealth publishes every 30 s
         * regardless of the measurement channel, and that is the path by which a
         * sustained over-range is visible remotely rather than only in the log.
         *
         * WHICH ALARM MATTERS: PGAD_ALM is differential over-range -- genuinely
         * too much current. But it is NOT the clipping point, and the gap is a
         * blind band this check cannot close:
         *
         *   digital clipping   +-VREF/G      = +-78.125 mV = +-39.06 A at 2 mOhm
         *   PGAD_ALM threshold +-105% FSR    = +-82.03 mV  = +-41.02 A
         *                                      (sec.7.5, sec.9.3.7.1, Fig. 9-8)
         *
         * So between 39.06 A and 41.02 A the 32-bit output is SATURATED and no
         * alarm fires. A 40 A input publishes 39.06 A: plausible, pinned, and
         * unflagged. The alarm accuracy is +-1% typ / +-3% max FSR on top, which
         * widens the band to ~38 A..42 A in the worst direction. THIS CHECK
         * THEREFORE DOES NOT BOUND CLIPPING -- treat it as a fault detector, not
         * as a range guard, and clamp in software if the range guard is needed.
         *
         * What the monitors ARE good for (sec.9.3.7): they are fast analog
         * comparators, so they catch short overrange events "not necessarily
         * evident in the output as clipped codes because of averaging of the
         * digital filter" -- transients the FIR would otherwise hide.
         *
         * Neither number is 17.5 A, which an earlier revision of this comment
         * and of the log message below both claimed. 35 mV / 17.5 A is not a
         * hardware threshold at all: it is the DESIGN POINT from which note N8's
         * common-mode window is derived. Equation 12 reads
         *     AVSS + 0.3 + |VIN|(G-1)/2 < VINP,VINN < AVDD - 0.3 - |VIN|(G-1)/2
         * so at |VIN| = 35 mV the window is 2.5 - 0.3 - 0.5425 = +-1.6575 V,
         * which is the figure the board was sized on. The window shrinks as the
         * differential grows but does not vanish -- at full scale it is still
         * 2.5 - 0.3 - 1.2109 = +-1.289 V. Quoting 17.5 A as the clipping point
         * sent a reader looking at the shunt for what is really a common-mode
         * fault, which is why the two are now named apart explicitly.
         *
         * PGAH/PGAL_ALM are ABSOLUTE: the PGA output ran into AVDD-0.2 V or
         * AVSS+0.2 V. At a small differential that means the COMMON MODE left the
         * window above, not that the current was too large. The fixes are
         * unrelated -- one is a shunt/gain problem, the other is a wiring and
         * reference-potential problem (DESIGN.md F1: J7 must be tied to the shunt
         * cold end, or the secondary floats on CM-cap leakage).
         *
         * The absolute alarms have their own blind band, in the opposite
         * direction from the differential one: they trip at AVDD-0.2/AVSS+0.2
         * (sec.7.5), while Equation 12 reserves 0.3 V. So an input can violate
         * the datasheet's valid-input range -- and stop being specified -- with
         * both alarms still clear. 0.1 V of PGA output at G=32 is 3.1 mV of
         * input, so the band is not narrow. */
        if (status & (STATUS_PGAD_ALM | STATUS_PGAH_ALM | STATUS_PGAL_ALM)) {
            /* raiseFault(), not a bare diag_ assignment: the latch-on-scan-complete
             * scheme cannot carry a fault that the very next good conversion
             * erases. finishPairAndAdvance() copies diag_ (now 0) over
             * diagPublished_ ~50 ms later, so a ONE-CONVERSION excursion was gone
             * long before the 30 s health channel next looked. */
            raiseFault(DIAG_PGA_RANGE, count);
            volts_[pair] = NAN;
            sd_[pair] = NAN;
            sum_ = sumsq_ = 0;
            nAvg_ = 0;
            /* Rate-limited: this fires per conversion, ~19/s at FIR 20 SPS, and a
             * sustained excursion drowned every other line on the console. */
            /* rangeLogged_ guards the FIRST report: lastRangeLogMs_ starts at 0, so an
             * excursion inside the first RANGE_LOG_INTERVAL_MS after boot compares
             * millis() against 0 and is silently rate-limited away -- exactly the window
             * where a miswired bench is most likely to be over-range. */
            if (!rangeLogged_ ||
                (uint32_t) (millis() - lastRangeLogMs_) >= RANGE_LOG_INTERVAL_MS) {
                lastRangeLogMs_ = millis();
                rangeLogged_ = true;
                const bool diff = (status & STATUS_PGAD_ALM) != 0;
                ESP_LOGE("ads1262", "PGA %s (STATUS=0x%02x:%s%s%s) -- %s; discarding",
                         diff ? "DIFFERENTIAL over-range" : "ABSOLUTE range violation", status,
                         diff ? " PGAD" : "",
                         (status & STATUS_PGAH_ALM) ? " PGAH(out>AVDD-0.2V)" : "",
                         (status & STATUS_PGAL_ALM) ? " PGAL(out<AVSS+0.2V)" : "",
                         diff ? "PGA differential output past +-105% FSR (+-82.0 mV in, ~+-41 A at "
                                "2 mOhm, G=32) -- note the output already CLIPS at +-78.1 mV/39.1 A, "
                                "so readings just under this alarm may be saturated"
                              : "PGA output hit a rail: at a small differential this is the COMMON "
                                "MODE outside the Equation-12 window (+-1.66 V at 35 mV, note N8), "
                                "NOT excess current -- check J7 to the shunt cold end");
            }
            return finishPairAndAdvance();
        }

        if (discardsLeft) {
            --discardsLeft;         // still settling after the mux change
            return false;
        }

        // bipolar 32-bit: LSB = VREF / (gain * 2^31)
        constexpr float lsb = VREF / (float) PGA_GAIN / (float) (2u << 30);
        const double v = (double) lsb * (double) count;
        sum_ += v;
        sumsq_ += v * v;
        if (++nAvg_ < AVG_N) return true;   // keep dwelling on this pair

        const double mean = sum_ / nAvg_;
        /* Sample variance; clamped at 0 because catastrophic cancellation in the
         * sum-of-squares form can make it very slightly negative. */
        const double var = nAvg_ > 1 ? (sumsq_ - sum_ * mean) / (nAvg_ - 1) : 0.0;
        volts_[pair] = (float) mean;
        sd_[pair] = (float) sqrt(var > 0 ? var : 0.0);
        sum_ = sumsq_ = 0;
        nAvg_ = 0;

        return finishPairAndAdvance();
    }

    /// This pair is done -- publish the generation and move the mux on.
    ///
    /// Shared by the normal completion path and the over-range discard, which is
    /// the point: an over-range MUST still advance the scan and latch the
    /// diagnostic, or one bad pair stalls the mux and the fault never reaches
    /// telemetry. Callers set volts_[pair]/sd_[pair] (real value, or NAN when the
    /// conversion was discarded) and clear the accumulators before calling.
    bool finishPairAndAdvance() {
        if (onlyPair_ >= 0) {
            /* Single-pair mode: do NOT touch INPMUX, so conversions keep arriving
             * at the nominal rate with no restart latency.
             *
             * The reconfiguration below is therefore CONDITIONAL on the die-temp
             * read, and that is load-bearing rather than tidiness. selectPair()
             * toggles START (sec.9.4.1), which restarts the conversion cycle and
             * costs td(STDR) -- 52.22 ms at FIR/20 SPS, doubled to ~104 ms by
             * chop (Equation 19). Running it unconditionally, as this block did
             * until now, paid that restart after EVERY completed average: at
             * AVG_N = 1 the achieved rate was ~9.6 SPS instead of 19.15, which
             * is half the throughput and sqrt(2) worse noise per unit time --
             * while the comment above claimed the opposite. Only the temperature
             * read disturbs the configuration (it needs INPMUX = 0xBB and
             * gain 1), so only the temperature read pays to restore it. */
            /* Die temperature moves far slower than the scan rate, and each read
             * costs a mux change, a gain switch and two START restarts. Reading
             * it every scan was both redundant and a measurable part of why the
             * achieved rate fell short of the budget. */
            if (++tempSkip_ >= TEMP_EVERY_N_SCANS) {
                tempSkip_ = 0;
                readDieTemperature();
                applyProductionConfig();
                selectPair((uint8_t) onlyPair_);
            }
            diagPublished_ = diag_;
            diag_ = 0;
            ++generation_;
            return true;
        }

        if (pair + 1 >= PAIR_COUNT) {
            /* One die-temperature reading per scan. It needs a different gain
             * and input, so it runs at the scan boundary rather than inside it,
             * and restores the measurement configuration afterwards. At sinc1
             * 1200 SPS it costs about a millisecond against a ~209 ms scan. */
            /* Die temperature moves far slower than the scan rate, and each read
             * costs a mux change, a gain switch and two START restarts. Reading
             * it every scan was both redundant and a measurable part of why the
             * achieved rate fell short of the budget. */
            if (++tempSkip_ >= TEMP_EVERY_N_SCANS) {
                tempSkip_ = 0;
                readDieTemperature();
            }

            /* A full scan of all four pairs is complete. Latch whatever
             * diagnostics accumulated during it so every facade reading this
             * generation sees the same value, then start the next scan clean. */
            diagPublished_ = diag_;
            diag_ = 0;
            ++generation_;
            selectPair(PAIR_CH0);
        } else {
            selectPair(pair + 1);
        }
        return true;
    }

private:
    // --- init-time self-tests -------------------------------------------------
    //
    // These exist because EXTCLK=1 is NOT proof of a usable clock. It says an
    // external clock was DETECTED; it says nothing about its FREQUENCY. On the
    // bench a floating XTAL1 picking up ~30 kHz of noise read EXTCLK=1 and
    // produced well-formed conversions at 1/240 the expected rate -- the exact
    // "absence of evidence encodes absence of the problem" shape schematic note
    // N10 warns about, in a form the note did not anticipate. Nothing in the
    // STATUS byte catches it, so it has to be MEASURED.
    //
    // All three tests fail closed: any path that cannot complete a measurement
    // returns false, never "fine".

    /// Nominal data rate used by the self-tests. Fast enough that a healthy
    /// clock yields plenty of conversions inside CLOCK_WINDOW_MS, slow enough
    /// that the polling loop is not the bottleneck.
    static constexpr uint8_t SELFTEST_DR = MODE2_DR_1200_SPS;
    static constexpr float SELFTEST_NOMINAL_SPS = 1200.0f;
    static constexpr uint32_t CLOCK_WINDOW_MS = 200;   ///< ~240 conversions when healthy
    static constexpr uint32_t CLOCK_RETRY_MS = 2000;   ///< second chance for a very slow clock
    /// Below this many conversions the rate estimate is too quantised to report.
    static constexpr uint32_t CLOCK_MIN_SAMPLES = 20;
    static constexpr uint32_t SELFTEST_CONV_TIMEOUT_MS = 500;

    static constexpr float FCLK_NOMINAL_HZ = 7372800.0f;
    static constexpr float FCLK_MIN_HZ = 1.0e6f;   ///< SBAS661C sec.7.3: 1..8 MHz external
    static constexpr float FCLK_MAX_HZ = 8.0e6f;
    static constexpr float FCLK_WARN_TOLERANCE = 0.05f;   ///< beyond this, log loudly

    static constexpr float AVDD_AVSS_MIN = 4.75f, AVDD_AVSS_MAX = 5.25f;  ///< Recommended Op Cond
    static constexpr float DVDD_MIN = 2.7f, DVDD_MAX = 5.25f;

    /// Puts the ADC into the self-test configuration and restarts conversions.
    /// sec.9.3.5 for the supply monitors: "enable the PGA, set gain = 1, and
    /// disable chop mode".
    void configureSelfTest(uint8_t inpmux) {
        zeroModeChop_ = 0xFF;   // configuration changed; zero mode must be re-entered
        setSTART(LOW);
        writeSingleRegister(REG_ADDR_MODE0, 0x00);                    // chop off
        writeSingleRegister(REG_ADDR_MODE1, MODE1_FILTER_SINC1);      // zero latency
        writeSingleRegister(REG_ADDR_MODE2, SELFTEST_DR);             // PGA on, gain 1
        writeSingleRegister(REG_ADDR_INPMUX, inpmux);
        sendCommand(OPCODE_STOP1);
        setSTART(HIGH);
        sendCommand(OPCODE_START1);
    }

    /// Waits for one genuine conversion. False on timeout -- never a fabricated code.
    bool oneConversion(int32_t &code) {
        for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < SELFTEST_CONV_TIMEOUT_MS;) {
            uint8_t status = 0, checksum = 0, data[4] = {0};
            const uint32_t faultsBefore = ads126xHalFaults();
            const int32_t c = readData(&status, data, &checksum);
            if (ads126xHalFaults() != faultsBefore) return false;
            if (!(status & STATUS_ADC1)) continue;
            if (checksum != calculateChecksum(data, 4)) continue;
            /* A reset since configureSelfTest() means the mux/gain we just
             * programmed are gone, so this conversion is of some other input.
             * Refuse rather than hand back a code the caller will scale into a
             * plausible temperature or supply voltage. The flag is sticky until
             * POWER is rewritten (cleared in init()'s register table), so this
             * cannot false-fire on a healthy part after init. */
            if (status & STATUS_RESET) {
                raiseFault(DIAG_DEVICE_RESET, 0);
                ESP_LOGE("ads1262", "device reset during self-test conversion -- refusing value");
                initialized = false;
                return false;
            }
            code = c;
            return true;
        }
        return false;
    }

    uint32_t countConversions(uint32_t windowMs) {
        uint32_t n = 0;
        for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < windowMs;) {
            uint8_t status = 0, checksum = 0, data[4] = {0};
            readData(&status, data, &checksum);
            if (status & STATUS_ADC1) ++n;
        }
        return n;
    }

    /// Measures fCLK by timing real conversions. Every ADS1262 timing scales with
    /// the clock, so measured_rate / nominal_rate is exactly fCLK / 7.3728 MHz.
    bool checkClock() {
        configureSelfTest((uint8_t) (INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N));

        uint32_t window = CLOCK_WINDOW_MS;
        uint32_t n = countConversions(window);
        /* Too few conversions to divide by. Covers both a dead converter (n=0)
         * and a very slow clock, where 1-2 hits in the short window quantise the
         * estimate so badly it reported 61440 Hz and 30720 Hz on consecutive
         * runs of the same board. A healthy clock lands ~240 here and skips this
         * entirely, so the retry costs nothing in the normal case. */
        if (n < CLOCK_MIN_SAMPLES) {
            window = CLOCK_RETRY_MS;
            n = countConversions(window);
        }
        if (n == 0) {
            ESP_LOGE("ads1262", "SELF-TEST FAIL: no conversions in %u ms at a nominal %.0f SPS. "
                                "The converter is not running at all.",
                     (unsigned) window, SELFTEST_NOMINAL_SPS);
            /* Do NOT tell the user to check the clock here.  SBAS661C sec.9.4.9:
             * "If no external clock is detected, the ADC automatically selects the
             * internal oscillator."  The fallback is automatic and needs no
             * command, so a dead, absent or out-of-spec external clock CANNOT stop
             * conversions -- the part would run on its own 7.3728 MHz instead.
             * Zero conversions therefore points at the ANALOG SUPPLIES or the part,
             * not at Y1, R88 or isolator channel E.  The message used to say
             * "check AVDD/AVSS and that a clock reaches XTAL1" and sent us chasing
             * the clock path for an hour. */
            reportStatusByte("no conversions");
            return false;
        }

        const float sps = (float) n * 1000.0f / (float) window;
        const float fclk = FCLK_NOMINAL_HZ * (sps / SELFTEST_NOMINAL_SPS);

        /* Anchor the running estimate. Recorded BEFORE the range check so a
         * failing board still reports what it actually measured rather than
         * nothing; the check below aborts init anyway, so nothing consumes it. */
        fclkInitHz_ = fclk;
        zeroSpsRef_ = NAN;   // reference belongs to this clock; re-establish it

        if (fclk < FCLK_MIN_HZ || fclk > FCLK_MAX_HZ) {
            ESP_LOGE("ads1262",
                     "SELF-TEST FAIL: fCLK ~ %.0f Hz, outside the %.0f..%.0f Hz the part accepts "
                     "(expected %.0f). EXTCLK reads 1 because SOMETHING is on XTAL1, but it is "
                     "not a valid clock -- check Y1 is oscillating, its supply and output enable, "
                     "J8, and the isolator channel carrying the clock.",
                     fclk, FCLK_MIN_HZ, FCLK_MAX_HZ, FCLK_NOMINAL_HZ);
            return false;
        }
        if (fabsf(fclk - FCLK_NOMINAL_HZ) / FCLK_NOMINAL_HZ > FCLK_WARN_TOLERANCE) {
            /* Raise the alarm here too, not just in the log. The 1..8 MHz range
             * check above passes a clock running at a QUARTER speed (1.84 MHz),
             * which is exactly the 2026-08-15 failure -- so without this, a
             * re-init onto an already-degraded clock would adopt it as the new
             * reference and every downstream comparison would read "healthy". */
            raiseFault(DIAG_CLOCK_DEGRADED, 0);
            ESP_LOGW("ads1262", "fCLK ~ %.0f Hz is more than %.0f%% off the expected %.0f Hz. "
                                "Conversion timing and the 50/60 Hz filter nulls scale with it.",
                     fclk, FCLK_WARN_TOLERANCE * 100.0f, FCLK_NOMINAL_HZ);
        }
        else
            ESP_LOGI("ads1262", "fCLK ~ %.0f Hz (%lu conversions in %u ms)", fclk,
                     (unsigned long) n, (unsigned) window);
        return true;
    }

    /// Reads the ADC's own supply monitors. sec.9.3.5:
    ///   V_ANLMON = (AVDD - AVSS) / 4,  V_DIGMON = (DVDD - DGND) / 4
    /// Independent of any external wiring, so this is a true self-check.
    bool checkSupplies() {
        // gain = 1 in the self-test config, so volts = code * VREF / 2^31
        constexpr float lsb1 = VREF / (float) (2u << 30);
        int32_t code = 0;

        configureSelfTest((uint8_t) (INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N));
        if (!oneConversion(code)) {
            ESP_LOGE("ads1262", "SELF-TEST FAIL: no conversion for the analog supply monitor");
            return false;
        }
        const float avddAvss = code * lsb1 * 4.0f;

        configureSelfTest((uint8_t) (INPMUX_MUXP_DVDD_P | INPMUX_MUXN_DVDD_N));
        if (!oneConversion(code)) {
            ESP_LOGE("ads1262", "SELF-TEST FAIL: no conversion for the digital supply monitor");
            return false;
        }
        const float dvdd = code * lsb1 * 4.0f;

        ESP_LOGI("ads1262", "supplies: AVDD-AVSS = %.3f V, DVDD = %.3f V", avddAvss, dvdd);

        bool ok = true;
        if (avddAvss < AVDD_AVSS_MIN || avddAvss > AVDD_AVSS_MAX) {
            ESP_LOGE("ads1262", "SELF-TEST FAIL: AVDD-AVSS = %.3f V, outside %.2f..%.2f V. "
                                "Check U3 (+2.5 V) and U4 (-2.5 V) and their +/-5 V inputs.",
                     avddAvss, AVDD_AVSS_MIN, AVDD_AVSS_MAX);
            ok = false;
        }
        if (dvdd < DVDD_MIN || dvdd > DVDD_MAX) {
            ESP_LOGE("ads1262", "SELF-TEST FAIL: DVDD = %.3f V, outside %.2f..%.2f V. "
                                "Check U5 (+3.3 V) -- a 2.5 V part fitted there lands at 2.5 V, "
                                "below the 2.7 V minimum.",
                     dvdd, DVDD_MIN, DVDD_MAX);
            ok = false;
        }
        return ok;
    }

    /// Collects `n` conversions of the internal shorted input at the production
    /// gain and returns mean, standard deviation (both input-referred µV) and the
    /// achieved rate. False if it could not gather the samples -- never partial
    /// statistics dressed up as a result.
    /// Puts the ADC into the zero-measurement configuration and leaves it there:
    /// inputs internally shorted to AINCOM, production filter and gain, chop as
    /// asked. Idempotent -- re-entering an identical configuration is a no-op, so
    /// consecutive samples do NOT restart conversions or re-settle.
    ///
    /// Holding one configuration is the whole point. Reconfiguring per sample
    /// walked the part through chop OFF->ON->OFF and gain 32->1->32 between
    /// consecutive readings, and a chop transition needs settling that two
    /// discarded conversions do not buy. That churn lands in the data as steps
    /// which look exactly like offset drift but are not.
    void enterZeroMode(uint8_t chopBits, uint8_t mode1, uint8_t drCode) {
        /* Every field is compared, not just chop: the filter and rate are now
         * caller-chosen, and keying the cache on a subset of what it programs
         * would serve a stale configuration as if it were the requested one. */
        if (zeroModeChop_ == chopBits && zeroModeMode1_ == mode1 && zeroModeDr_ == drCode)
            return;   // already in this configuration

        setSTART(LOW);
        writeSingleRegister(REG_ADDR_MODE0, chopBits);
        writeSingleRegister(REG_ADDR_MODE1, mode1);
        writeSingleRegister(REG_ADDR_MODE2,
                            (uint8_t) ((PGA_GAIN_CODE << 4) | drCode));
        writeSingleRegister(REG_ADDR_INPMUX,
                            (uint8_t) (INPMUX_MUXP_AINCOM | INPMUX_MUXN_AINCOM));
        /* Upstream writeSingleRegister() does not update the shadow map; only
         * readSingleRegister() does, and readData() sizes its transfer from it. */
        const uint8_t m0 = readSingleRegister(REG_ADDR_MODE0);
        const uint8_t m1 = readSingleRegister(REG_ADDR_MODE1);
        const uint8_t m2 = readSingleRegister(REG_ADDR_MODE2);
        (void) readSingleRegister(REG_ADDR_INPMUX);
        logConfig("zero mode", m0, m1, m2);

        sendCommand(OPCODE_STOP1);
        setSTART(HIGH);
        sendCommand(OPCODE_START1);
        zeroModeChop_ = chopBits;
        zeroModeMode1_ = mode1;
        zeroModeDr_ = drCode;
        zeroSettleLeft_ = ZERO_SETTLE_DISCARDS;
        /* Deliberately does NOT clear zeroSpsRef_. Re-entering zero mode is not
         * evidence about the clock -- and it happens after every temperature
         * read with unchanged settings. measureZero() re-anchors only when the
         * CONFIGURATION differs from the one the reference was taken under
         * (refChop_/refMode1_/refDr_), which is the only case where the expected
         * rate actually changed. */

        /* Chop doubles the first-conversion latency (Equation 19) AND makes the
         * steady-state period td(STDR) rather than 1/DR (Equation 21), so both
         * cases are covered by scaling the same figure. 3x leaves the timeout
         * catching a genuinely dead DRDY without firing on a slow healthy one. */
        drdyTimeoutMs_ = 3u * (chopBits ? 2u : 1u) * ZERO_TD_STDR_MS;
    }

    /// Accumulates n conversions in the configuration already established by
    /// enterZeroMode(). Touches no registers, so the part is undisturbed between
    /// samples and only the settling right after a configuration CHANGE is
    /// discarded -- not after every sample.
    bool measureZero(uint8_t chopBits, uint16_t n, float &meanUv, float &sdUv, float &sps,
                     uint8_t mode1 = MODE1_VALUE, uint8_t drCode = DATA_RATE_CODE) {
        enterZeroMode(chopBits, mode1, drCode);

        constexpr float lsbUv = VREF / (float) PGA_GAIN / (float) (2u << 30) * 1e6f;

        /* Budget scaled to the SLOWEST configuration this is called in, since a
         * budget sized for the fast case times out on a part that is working
         * correctly -- a timeout that fires on healthy hardware is worse than no
         * timeout, because it reports a fault that is not there.
         *
         * Worst case is sinc4 @ 10 SPS WITH CHOP. The chopped rate is NOT the
         * nominal rate: Equation 21 makes it 1/td(STDR), so with td(STDR) =
         * 400.4 ms (Table 9-13) conversions arrive every 400 ms, not 100, and
         * Equation 19 doubles the first one to 800 ms. That is
         * 800 + (ZERO_SETTLE_DISCARDS + n - 1) x 400 = 5.2 s at n = 4.
         *
         * The previous 2000 + count x 400 gave 6.8 s against that -- 1.3x, not
         * the 4x its comment claimed, because the comment assumed 100 ms
         * conversions and chop had been off when it was written. 1200 ms per
         * conversion restores a genuine ~3x. */
        const uint32_t budgetMs =
                3000 + (uint32_t) (ZERO_SETTLE_DISCARDS + n) * 1200;
        double sum = 0, sumSq = 0;
        uint32_t got = 0, first = 0, last = 0;

        for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < budgetMs && got < n;) {
            uint8_t status = 0, checksum = 0, data[4] = {0};
            const uint32_t faultsBefore = ads126xHalFaults();
            const int32_t code = readData(&status, data, &checksum);
            if (ads126xHalFaults() != faultsBefore) return false;
            if (!(status & STATUS_ADC1)) continue;
            if (checksum != calculateChecksum(data, 4)) continue;
            /* The part reset under us: its registers are back at POR defaults
             * (chop off, gain 1, AIN0/AIN1) while zeroModeChop_ still claims the
             * zero configuration is programmed. Without this, enterZeroMode()
             * no-ops on the next call and we publish raw AIN0/AIN1 at gain 1 as
             * "internal short, gain 32" -- plausible numbers, no diagnostic, for
             * as long as it takes the periodic temp read to rewrite the
             * registers by accident. pump() has always checked this; this loop
             * did not. Invalidate the cache so the next call reprograms. */
            if (status & STATUS_RESET) {
                diag_ = encodeDiag(DIAG_DEVICE_RESET, 0);
                ESP_LOGE("ads1262", "device reset during zero measurement -- configuration lost");
                zeroModeChop_ = 0xFF;
                initialized = false;
                return false;
            }
            if (zeroSettleLeft_) { --zeroSettleLeft_; continue; }

            const double uv = (double) code * (double) lsbUv;
            const uint32_t now = millis();
            if (!got) first = now;
            last = now;
            sum += uv;
            sumSq += uv * uv;
            ++got;
        }
        if (got < n || last == first) return false;

        meanUv = (float) (sum / got);
        const double var = sumSq / got - (sum / got) * (sum / got);
        sdUv = (float) sqrt(var > 0 ? var : 0);
        sps = (float) (got - 1) * 1000.0f / (float) (last - first);

        /* Track fCLK from this rate. The nominal zero-mode rate is not hardcoded
         * -- it depends on the data rate code AND on chop, which halves it -- so
         * the first healthy measurement after a configuration change becomes the
         * reference and everything after is a ratio against it. That makes this
         * correct for any filter/rate/chop combination without a table to keep in
         * sync, and it is why enterZeroMode() clears the reference. */
        const bool newConfig = (refChop_ != chopBits || refMode1_ != mode1 || refDr_ != drCode);
        if (newConfig || !std::isfinite(zeroSpsRef_)) {
            /* First rate seen for THIS configuration. Trustworthy as a reference
             * only because checkClock() validated fCLK during init and raises
             * DIAG_CLOCK_DEGRADED itself if the clock was already off nominal. */
            refChop_ = chopBits;
            refMode1_ = mode1;
            refDr_ = drCode;
            zeroSpsRef_ = sps;
        } else if (zeroSpsRef_ > 0) {
            fclkEstHz_ = fclkInitHz_ * (sps / zeroSpsRef_);
            const float dev = fabsf(sps / zeroSpsRef_ - 1.0f);
            if (dev > FCLK_ALARM_TOLERANCE) {
                diag_ = encodeDiag(DIAG_CLOCK_DEGRADED, 0);
                ESP_LOGW("ads1262", "clock degraded: conversion rate %.2f SPS vs %.2f expected "
                                    "(fCLK ~ %.0f Hz vs %.0f at init)",
                         sps, zeroSpsRef_, fclkEstHz_, fclkInitHz_);
            }
        }

        /* Publish whatever this measurement accumulated. diag() returns
         * diagPublished_, which until now was latched ONLY inside pump() -- the
         * multi-pair scan, which this build never calls (shuntAdcIn is not
         * registered, and esp32s3_n16r8 compiles with SHUNT_ADC_ONLY). Every reason
         * raised on the zero-mode path, including DIAG_CLOCK_DEGRADED and
         * DIAG_DEVICE_RESET, therefore never reached a published Sample. */
        diagPublished_ = diag_;
        diag_ = 0;
        return true;
    }

    /// True once EXTCLK reads 1 in a genuine conversion.
    ///
    /// Bounded by WALL CLOCK, not by an attempt count. waitForEdge() polls with a
    /// 1 ms notification timeout and returns false when nothing has arrived yet,
    /// so "8 attempts" was ~8 ms -- and the first conversion after START cannot
    /// arrive before td(STDR) = 52.22 ms at 20 SPS/FIR [Table 9-13]. The old
    /// loop therefore expired before any real data existed and reported a
    /// missing clock on every boot of perfectly good hardware.
    bool confirmExtClk() {
        const uint32_t t0 = millis();
        while ((uint32_t) (millis() - t0) < EXTCLK_CONFIRM_MS) {
            if (!waitForEdge()) continue;

            uint8_t status = 0, checksum = 0, data[4] = {0};
            const uint32_t faultsBefore = ads126xHalFaults();
            readData(&status, data, &checksum);

            /* A read that never reached the wire tells us nothing about the
             * clock -- keep waiting rather than judging on invented bytes. */
            if (ads126xHalFaults() != faultsBefore) continue;

            /* Only a real conversion carries a meaningful STATUS byte. */
            if (!(status & STATUS_ADC1)) continue;

            /* EXTCLK is read-only status, so the first genuine conversion is
             * decisive in both directions. */
            return (status & STATUS_EXTCLK) != 0;
        }
        /* Timed out without ever seeing a conversion. That is "unverified", and
         * unverified must not be reported as synchronised. */
        ESP_LOGE("ads1262", "no conversion within %u ms -- cannot confirm EXTCLK "
                            "(DRDY edges arriving at all?)", (unsigned) EXTCLK_CONFIRM_MS);
        return false;
    }

    bool waitForEdge();

    friend void IRAM_ATTR ads1262_drdy_isr();
};


// --- DRDY plumbing -----------------------------------------------------------
//
// The ready line IS the data line, so the handler must be blind while we clock.
// `ads1262_bus_busy` is called by the HAL immediately around every transfer.

inline bool Ads1262ShuntAdc::waitForEdge() {
    if (!ads1262_ready) {
        ads1262_notification.subscribe();
        ads1262_notification.wait(1);
    }

    if (ads1262_ready) {
        ads1262_ready = false;
        lastEdgeMs = millis();
        return true;
    }

    /* No edge. A conversion we failed to read leaves the pin low with no
     * further edges at all, which looks identical to "never ready" (note
     * N11 b) -- so after the timeout, read anyway to re-arm it. */
    if ((uint32_t) (millis() - lastEdgeMs) >= drdyTimeoutMs_) {
        ESP_LOGW("ads1262", "no DRDY edge for %u ms -- reading to re-arm",
                 (unsigned) drdyTimeoutMs_);
        lastEdgeMs = millis();
        return true;
    }
    return false;
}


/// Streams the ADC's INTERNAL short with chop ON, through the normal sampler
/// path, so its offset drift can be tracked against die temperature alongside
/// everything else instead of via a bespoke bench loop.
///
/// MUTUALLY EXCLUSIVE with PowerSampler_ShuntAdc on the same device: this mode
/// reconfigures the ADC (chop off, INPMUX to AINCOM/AINCOM) and leaves it there,
/// so a production scan interleaved with it would read the wrong thing. Register
/// one or the other, never both.
///
/// Measures the ADC ONLY. An internal short excludes the external path, so this
/// is NOT the system offset (delta-epsilon) the shunt sizing depends on -- that
/// needs the terminals shorted at the shunt, with chop ON.
///
/// FIELD MAPPING -- this channel overloads Sample/InfluxDB fields, because the
/// quantities it produces are not U/I/P. Units are VOLTS throughout, like every
/// other channel -- there is no per-channel scaling in the publish path.
///   U ("U")     offset of the shorted input, volts at the ADC input
///   I ("I")     within-burst standard deviation of the avgN conversions, volts
///               -- the white-noise floor, NOT a current
///   I_max       largest such deviation in the window; 3 decimals, so it stores
///               as 0.000 for this channel -- use I, not I_max
///   P ("P")     always 0; there is no power on a shorted input
///   T ("T")     ADC die temperature, the x-axis for the drift
class PowerSampler_ShuntAdcZero : public PowerSampler {
    Ads1262ShuntAdc &dev;
    const uint16_t avgN;

    /// Data samples between die-temperature reads. One sensor, slow-moving, and
    /// each read costs the zero path a configuration change plus a re-settle.
    /// A temperature read costs a full zero-mode re-entry: sinc4's td(STDR) is
    /// 400 ms of filter refill (Table 9-13, and measured to the millisecond on
    /// the bench), which lands on the sample that follows it. That is the whole
    /// reason the interval is this long.
    ///
    /// 32 is oversampling by a wide margin, not a compromise. The die's thermal
    /// time constant is MINUTES -- the warm-up after a reflash took ~7 min to
    /// settle -- so at ~2.4 SPS this still reads the temperature every ~13 s,
    /// two orders of magnitude faster than the signal can physically move.
    /// Sampling it every 8 samples bought no information and cost 400 ms four
    /// times as often.
    static constexpr uint8_t TEMP_EVERY_N_SAMPLES = 32;
    uint8_t tempSkip = TEMP_EVERY_N_SAMPLES;   ///< force a read on the first sample

    uint32_t lastReinitMs = 0;
    static constexpr uint32_t REINIT_INTERVAL_MS = 2000;

    const uint8_t storageId;
    const int8_t pinSck, pinMosi, pinMiso, pinCs, pinStart;
    Sample lastSample{};

public:
    PowerSampler_ShuntAdcZero(Ads1262ShuntAdc &dev, uint16_t avgN, uint8_t storageId,
                              int8_t pinSck, int8_t pinMosi, int8_t pinMiso, int8_t pinCs,
                              int8_t pinStart)
            : dev(dev), avgN(avgN), storageId(storageId), pinSck(pinSck), pinMosi(pinMosi),
              pinMiso(pinMiso), pinCs(pinCs), pinStart(pinStart) {}

    uint8_t getStorageId() const override { return storageId; }

    /// Diagnostic channel, not a power channel. Opting out matters: sampling.h
    /// notes that a sampler whose mean power is NaN by construction would
    /// otherwise pin the board permanently awake.
    bool measuresPower() const override { return false; }

    bool init() override {
        /* MUTUALLY EXCLUSIVE with any measurement facade, and now enforced rather
         * than only documented. This facade parks the ADC on its internal short
         * and leaves it there; a production channel sharing the device would
         * publish those shorted conversions as plausible shunt current. Both
         * samplers.add() lines used to be enablable at once with no complaint. */
        if (dev.hasMeasurementFacade()) {
            ESP_LOGE("ads1262", "a measurement facade owns this device -- the zero-drift facade "
                                "parks the ADC on its internal short and cannot share it, "
                                "refusing to start");
            return false;
        }
        if (!dev.init(pinSck, pinMosi, pinMiso, pinCs, pinStart)) return false;
        dev.noteZeroFacade();
        return true;
    }

    void startReading() override {}

    bool hasData() override {
        /* Recover from a device reset. measureZero() reports one by clearing
         * `initialized`, and ONLY init() rewrites the POWER register -- which is
         * what clears the ADC's sticky RESET flag. Without this the detector
         * wedged the channel permanently: every later conversion still carried
         * STATUS_RESET, so the check re-tripped forever, and EnergyCounter's
         * "re-arming sampler" was inert because startReading() does nothing here.
         * Observed on the bench 2026-08-15: 82 consecutive re-arms over 22
         * minutes, board otherwise healthy. A detector whose failure verdict has
         * no recovery path turns a transient fault into a permanent outage.
         *
         * Rate-limited, so a board that is genuinely gone does not spin. Mirrors
         * PowerSampler_ShuntAdc::hasData(), which had this from the start. */
        if (dev.needsReinit()) {
            const uint32_t now = millis();
            if ((uint32_t) (now - lastReinitMs) < REINIT_INTERVAL_MS) return false;
            lastReinitMs = now;
            ESP_LOGW("ads1262", "zero channel: re-initialising after device reset");
            if (!dev.init(pinSck, pinMosi, pinMiso, pinCs, pinStart)) return false;
        }

        float meanUv = NAN, sdUv = NAN, sps = NAN;
        if (!dev.zeroDriftSample(avgN, meanUv, sdUv, sps)) return false;
        /* Every 8th sample only. A temperature read needs INPMUX=0xBB and gain 1,
         * so it leaves zero mode and the next sample pays the re-settle. Doing it
         * per sample meant every reading carried that disturbance; at 1-in-8 the
         * other seven stay in one undisturbed configuration, and the perturbed
         * one is predictable rather than everywhere. */
        if (++tempSkip >= TEMP_EVERY_N_SAMPLES) {
            tempSkip = 0;
            dev.readDieTemperature();
        }

        lastSample.setTimeNow();

        /* VOLTS, like every other channel -- no per-channel unit special case
         * anywhere in the publish path.
         *
         * This relies on getInfluxDbPoint() publishing U and I at 12 decimal
         * places. At the original 8 the quantum was 10 nV, the same order as the
         * offset drift being characterised, so the effect WAS the quantisation
         * step. If that precision is ever reduced this channel goes blind
         * silently -- the series keeps arriving, it just stops moving.
         *
         * Known cost: EnergyCounter::summary()'s shared "%7.4f" console format
         * renders 7 uV as 0.0000. The console is unusable for this channel; the
         * stored series is the readout. */
        lastSample.u = meanUv * 1e-6f;

        /* NOT a current: the WITHIN-BURST standard deviation of the avgN
         * conversions, in volts at the ADC input, riding the `i` field so it
         * reaches InfluxDB (as "I", with "I_max" as its window max) through the
         * existing averaging path -- Sample is packed and shared with the BLE
         * WireSample, so a dedicated member would cost RAM on every channel and
         * bump the wire format for one diagnostic series.
         *
         * Why it is worth carrying: `u` scatter is noise PLUS drift and cannot
         * separate them. `sd` is the white-noise floor alone (SBAS661C Table 8-1
         * gives 0.030 uVRMS at 20 SPS / FIR / gain 32). If `u` wanders while
         * `sd` stays flat, that is drift, measured rather than inferred. */
        lastSample.i = sdUv * 1e-6f;

        /* Explicit 0, NOT NaN. Sample::p() falls back to u*i when p_ is NaN, so
         * leaving it NaN would now publish u*sd as a power; and EnergyCounter
         * integrates p() into Energy, where a single NaN is sticky and would
         * make E NaN for the rest of the run. There is no power on a shorted
         * input, so 0 is both true and finite. */
        lastSample.p_ = 0.0f;
        lastSample.temp = dev.dieTempC();   // single dedicated sampler: no duplication
        lastSample.diag = dev.diag();
        return true;
    }

    Sample getSample() override { return lastSample; }
};


/// One half of the efficiency measurement: a voltage pair and a current pair.
/// Two of these share the single Ads1262ShuntAdc.
class PowerSampler_ShuntAdc : public PowerSampler {
    Ads1262ShuntAdc &dev;
    const Ads1262ShuntAdc::Pair uPair, iPair;

    /// Scaling from ADC volts to engineering units. No defaults on purpose:
    /// J3..J6 are pre-scaled low-voltage ports with NO divider on the board
    /// (note N11), so both factors live entirely in the bench setup and a
    /// wrong-but-plausible default would silently produce wrong numbers.
    const float shuntOhm;      ///< current  = V_adc / shuntOhm
    const float dividerRatio;  ///< voltage  = V_adc * dividerRatio

    /// Whether THIS channel carries the die temperature. There is ONE sensor on
    /// the chip, so reporting it from every registered channel would put N copies
    /// of the same number in telemetry. Default false: opt exactly one channel in.
    const bool reportDieTemp;

    const uint8_t storageId;
    const int8_t pinSck, pinMosi, pinMiso, pinCs, pinStart;

    uint32_t lastGen = 0;
    uint32_t lastReinitMs = 0;
    static constexpr uint32_t REINIT_INTERVAL_MS = 2000;
    Sample lastSample{};

public:
    PowerSampler_ShuntAdc(Ads1262ShuntAdc &dev, Ads1262ShuntAdc::Pair uPair,
                          Ads1262ShuntAdc::Pair iPair, float shuntOhm, float dividerRatio,
                          uint8_t storageId, int8_t pinSck, int8_t pinMosi, int8_t pinMiso,
                          int8_t pinCs, int8_t pinStart, bool reportDieTemp = false)
            : dev(dev), uPair(uPair), iPair(iPair), shuntOhm(shuntOhm),
              dividerRatio(dividerRatio), reportDieTemp(reportDieTemp), storageId(storageId),
              pinSck(pinSck), pinMosi(pinMosi), pinMiso(pinMiso), pinCs(pinCs),
              pinStart(pinStart) {}

    uint8_t getStorageId() const override { return storageId; }

    /// Opt OUT of the idle-sleep vote while the scale is unset. With shuntOhm
    /// unset this sampler reports i = NAN forever, so its mean power is NaN
    /// forever -- and looksActive() reads a non-finite mean power as "cannot
    /// judge, stay awake". Left at the default true, one uncalibrated channel
    /// would silently pin the board awake for good. Same reason TMP117 opts out.
    bool measuresPower() const override { return shuntOhm > 0 && dividerRatio > 0; }

    bool init() override {
        /* An unconfigured scale factor is allowed and reported as NAN rather than
         * refusing to start: NAN means "not measured" and cannot be mistaken for
         * a reading, whereas a placeholder ohm value would put plausible wrong
         * currents on the wire. Set them and the same channel becomes amps/volts
         * with no other change. */
        if (!(shuntOhm > 0))
            ESP_LOGW("ads1262", "shuntOhm unset: reporting raw ADC volts in u, i=NAN");
        if (!(dividerRatio > 0) && uPair != Ads1262ShuntAdc::PAIR_NONE)
            ESP_LOGW("ads1262", "dividerRatio unset: u is raw ADC volts, unscaled");
        // idempotent: whichever facade initialises first brings up the shared ADC
        if (!dev.init(pinSck, pinMosi, pinMiso, pinCs, pinStart)) return false;

        /* iPair is the ONLY axis that indexes hardware, so it is validated here
         * rather than trusted. PAIR_NONE is 0xFF and both constructor arguments
         * have the same type, so swapping them -- the obvious mistake now that a
         * named sentinel exists -- would reach INPMUX_FOR[255] and volts_[255].
         * Out-of-bounds on a 4-element array, from a plain argument order slip. */
        if (iPair >= Ads1262ShuntAdc::PAIR_COUNT) {
            ESP_LOGE("ads1262", "iPair=%u is not a real pair (0..%u); refusing to start. "
                                "PAIR_NONE is only valid as uPair.",
                     (unsigned) iPair, (unsigned) Ads1262ShuntAdc::PAIR_COUNT - 1);
            return false;
        }

        /* uPair INDEXES MEMORY TOO, which the sentence above used to deny. It
         * does not reach INPMUX_FOR[], so it is not a hardware axis -- but
         * hasData() calls dev.volts(uPair) on every published sample whenever
         * uPair is not PAIR_NONE, and volts_ is a 4-element array. Anything in
         * 4..254 therefore reads off the end of it and publishes whatever
         * happened to be in the adjacent member (sd_[], dieTempC_, ...) as a
         * voltage: finite, plausible, wrong, and with no diagnostic anywhere.
         *
         * PAIR_COUNT itself is the likely value to arrive here, because it is
         * the natural thing to write for "one past the end" and it sits directly
         * beside PAIR_NONE in the same enum. Only PAIR_NONE and a real pair are
         * meaningful, so everything else is refused. */
        if (uPair >= Ads1262ShuntAdc::PAIR_COUNT && uPair != Ads1262ShuntAdc::PAIR_NONE) {
            ESP_LOGE("ads1262", "uPair=%u is neither a real pair (0..%u) nor PAIR_NONE; "
                                "refusing to start rather than index volts_[] out of bounds",
                     (unsigned) uPair, (unsigned) Ads1262ShuntAdc::PAIR_COUNT - 1);
            return false;
        }

        /* Single-pair mode is a property of the DEVICE, not of this facade, so
         * two facades sharing one ADS1262 cannot disagree about it. They would
         * not fail loudly if they did: the scanning facade would keep receiving
         * single-pair generations and publish its other pairs' stale values
         * forever, which reads as working. Refuse at init() instead -- the class
         * contract allows shared facades, so the incompatible COMBINATION is
         * what has to be caught. */
        if (dev.onlyPair() >= 0 && uPair != Ads1262ShuntAdc::PAIR_NONE) {
            ESP_LOGE("ads1262", "device is already in single-pair mode (pair %d); a scanning "
                                "facade cannot share it -- refusing to start", (int) dev.onlyPair());
            return false;
        }
        if (dev.onlyPair() < 0 && uPair == Ads1262ShuntAdc::PAIR_NONE && dev.hasScanningFacade()) {
            ESP_LOGE("ads1262", "a scanning facade already owns this device -- a single-pair "
                                "facade cannot share it, refusing to start");
            return false;
        }
        /* Two single-pair facades wanting DIFFERENT pairs slipped through both
         * checks above: the second one is itself PAIR_NONE (so the first check
         * does not apply) and onlyPair_ is already set (so the second does not
         * either). It then called setOnlyPair() with its own iPair and silently
         * took the mux, leaving the first facade reading a pair the hardware no
         * longer visits -- stale values with fresh timestamps. Two facades on the
         * SAME pair are fine and deliberately still allowed. */
        if (uPair == Ads1262ShuntAdc::PAIR_NONE && dev.onlyPair() >= 0 &&
            dev.onlyPair() != (int8_t) iPair) {
            ESP_LOGE("ads1262", "device is already parked on pair %d; this facade wants pair %u. "
                                "One mux cannot serve both -- refusing to start",
                     (int) dev.onlyPair(), (unsigned) iPair);
            return false;
        }
        /* The ZERO facade reconfigures the part to its internal short and leaves
         * it there. It is mutually exclusive with any measurement facade, and
         * until now nothing enforced that: both samplers.add() lines could be
         * enabled and init() would succeed for both. The production channel would
         * then publish internal-short conversions as plausible shunt current --
         * and the conditional restore added for throughput makes that worse, since
         * production configuration is now only reasserted at temperature
         * boundaries rather than after every sample. */
        if (dev.hasZeroFacade()) {
            ESP_LOGE("ads1262", "the zero-drift facade owns this device (it parks the ADC on its "
                                "internal short) -- a measurement facade cannot share it, "
                                "refusing to start");
            return false;
        }
        dev.noteMeasurementFacade();

        /* Single-pair mode. Set AFTER init(), which runs the self-test and leaves
         * the part on PAIR_CH0; setting it before would be overwritten. */
        if (uPair == Ads1262ShuntAdc::PAIR_NONE) {
            dev.setOnlyPair((int8_t) iPair);
            ESP_LOGI("ads1262", "single-pair mode: mux parked on pair %u, no voltage "
                                "channel (u = raw shunt volts, p = 0)", (unsigned) iPair);
        } else {
            dev.noteScanningFacade();
            ESP_LOGI("ads1262", "scanning mode: u from pair %u, i from pair %u",
                     (unsigned) uPair, (unsigned) iPair);
        }
        return true;
    }

    void startReading() override {
        // nothing to do: the ADC free-runs and pump() advances the scan
    }

    bool hasData() override {
        /* The device sets needsReinit() when it reports its own reset flag: its
         * registers are back at defaults, so it must be reconfigured before
         * anything it produces means anything. Without this the detection was
         * inert -- pump() short-circuited forever and the sampler stayed silently
         * dead for the rest of the process. Rate-limited so a board that is
         * genuinely gone does not spin on re-init. */
        if (dev.needsReinit()) {
            const uint32_t now = millis();
            if ((uint32_t) (now - lastReinitMs) < REINIT_INTERVAL_MS) return false;
            lastReinitMs = now;
            ESP_LOGW("ads1262", "re-initialising after device reset");
            if (!dev.init(pinSck, pinMosi, pinMiso, pinCs, pinStart)) return false;
            /* RE-PARK. init() always programs and records PAIR_CH0, but it does
             * NOT consult onlyPair_ -- so after a reset the hardware sits on CH0
             * while onlyPair_ still names whatever this facade asked for. pump()
             * then writes its readings into volts_[CH0] and this facade goes on
             * reading volts_[iPair], republishing its last PRE-RESET value with
             * fresh timestamps until the next temperature boundary happens to
             * reselect the pair. Stale data wearing a current timestamp is the
             * worst of the available failures, and it is silent.
             *
             * Invisible for iPair == CH0, which is all this build ships, so the
             * defect was masked by the configuration rather than absent. */
            if (uPair == Ads1262ShuntAdc::PAIR_NONE) dev.setOnlyPair((int8_t) iPair);
        }

        dev.pump();

        /* Only publish on a COMPLETED scan, so u and i belong to the same pass
         * over the multiplexer. They are still not simultaneous -- one ADC
         * cannot be -- but they are at least adjacent and consistently ordered. */
        if (dev.generation() == lastGen) return false;
        lastGen = dev.generation();

        const bool singlePair = (uPair == Ads1262ShuntAdc::PAIR_NONE);
        const float vI = dev.volts(iPair);
        const float vU = singlePair ? vI : dev.volts(uPair);

        /* NaN WITH a diagnostic is a REPORTABLE fault, not "nothing to say".
         *
         * The over-range path deliberately writes NAN into volts_ so a clipped
         * reading can never be published as a current -- but returning false here
         * then swallowed the reason as well. The scan HAD advanced, so lastGen
         * was consumed, and the next good conversion cleared diagPublished_ about
         * 50 ms later. A transient PGA excursion therefore left no trace anywhere
         * a remote observer could see: the series simply skipped a sample, which
         * is indistinguishable from an idle board. The health channel could not
         * cover it either -- it publishes every 30 s, some 600 conversions later.
         *
         * So publish the fault as a sample that carries NAN and says why. NaN is
         * this driver's word for "not measured", which is exactly true of a
         * clipped conversion, and diag names the cause. Silence, by contrast,
         * asserts nothing and cannot be distinguished from health.
         *
         * NaN with NO diagnostic really is nothing to say -- a pair that has not
         * produced its first reading yet -- and still returns false. */
        if (std::isnan(vU) || std::isnan(vI)) {
            const uint32_t d = dev.diag();
            if (!d) return false;
            lastSample.setTimeNow();
            lastSample.u = NAN;
            lastSample.i = NAN;
            /* NAN, not 0. Sample::p() falls back to u*i when p_ is NAN, and both
             * are NAN here, so p reports "not measured" rather than a confident
             * zero watts for a conversion that never happened. */
            lastSample.p_ = NAN;
            if (reportDieTemp) lastSample.temp = dev.dieTempC();
            lastSample.diag = d;
            return true;
        }

        lastSample.setTimeNow();
        lastSample.u = (dividerRatio > 0) ? vU * dividerRatio : vU;
        lastSample.i = (shuntOhm > 0) ? vI / shuntOhm : NAN;

        /* Single-pair mode has no voltage channel, so u carries the RAW SHUNT
         * VOLTS (the same "unscaled u" convention used when dividerRatio is
         * unset). Sample::p() would then return u*i = the shunt's own
         * dissipation, which is a real quantity but would be logged as "P" and
         * read as load power. Pin it to 0 instead, exactly as the zero channel
         * does: there is no power measurement here, and 0 says so where a
         * plausible small number would not.
         *
         * Assigned on BOTH branches, never conditionally. lastSample is a member
         * that survives between calls, so a p_ written once persists into every
         * later sample -- a single faulted reading (which sets NAN) would
         * otherwise leave a scanning channel reporting NAN power forever, and a
         * single-pair 0 would pin a scanning channel to zero watts. */
        lastSample.p_ = singlePair ? 0.0f : NAN;
        if (reportDieTemp) lastSample.temp = dev.dieTempC();
        /* Non-consuming: the other facade sharing this device must see the same
         * diagnostic rather than a zero because we got here first. */
        lastSample.diag = dev.diag();
        return true;
    }

    Sample getSample() override { return lastSample; }
};


/// Health/telemetry channel for the shunt-adc front end: fCLK and the ADC's own
/// supply monitors, recorded as a time series.
///
/// Why this exists: on 2026-08-15 the external clock degraded to a quarter speed
/// at 07:20 and the board died at 07:58. Nothing recorded it. The diagnosis came
/// from counting InfluxDB rows per 5 minutes AFTER the fact and noticing the rate
/// had dropped by exactly 4x -- fCLK was inferrable only because every ADS1262
/// timing scales with it. Recording it directly turns that archaeology into a
/// trend, and DIAG_CLOCK_DEGRADED raises it live.
///
/// This sampler PUBLISHES ONLY. It never triggers an ADC excursion of its own --
/// the values are refreshed by whichever measurement channel is registered, on
/// its existing schedule, so adding this costs no conversions and no settling.
/// The consequence is that it is only as fresh as that channel: with nothing else
/// registered, the supplies stay at their init values and only fCLK updates.
///
/// KNOWN LIMITATION: both quantities are measured THROUGH the converter, so when
/// the ADC stops converting they stop with it. This records the approach to a
/// failure, not the failure. That was enough on 2026-08-15 -- the degradation ran
/// for 35 minutes -- but it is not a substitute for an independent supply sense.
class PowerSampler_ShuntAdcHealth : public PowerSampler {
    Ads1262ShuntAdc &dev;
    const uint32_t intervalMs;
    uint32_t lastPublishMs = 0;
    bool published = false;

    const uint8_t storageId;
    const int8_t pinSck, pinMosi, pinMiso, pinCs, pinStart;
    Sample lastSample{};

public:
    PowerSampler_ShuntAdcHealth(Ads1262ShuntAdc &dev, uint32_t intervalMs, uint8_t storageId,
                                int8_t pinSck, int8_t pinMosi, int8_t pinMiso, int8_t pinCs,
                                int8_t pinStart)
            : dev(dev), intervalMs(intervalMs), storageId(storageId), pinSck(pinSck),
              pinMosi(pinMosi), pinMiso(pinMiso), pinCs(pinCs), pinStart(pinStart) {}

    uint8_t getStorageId() const override { return storageId; }

    /// Diagnostics, not power. Opting out keeps it out of the idle-sleep vote,
    /// which reads a non-finite mean power as "cannot judge -> stay awake".
    bool measuresPower() const override { return false; }

    /// Publishing every intervalMs is the design, not a stall. Give the detector
    /// generous headroom over the interval so a genuine hang is still caught.
    int64_t stallTimeoutUs() const override { return (int64_t) intervalMs * 3000; }

    bool init() override {
        /* The health facade drives no conversions of its own and reconfigures
         * nothing, so it is free to sit alongside whichever measurement channel
         * is active. It claims no ownership on purpose. */
        return dev.init(pinSck, pinMosi, pinMiso, pinCs, pinStart);
    }

    void startReading() override {}

    bool hasData() override {
        const uint32_t now = millis();
        if (published && (uint32_t) (now - lastPublishMs) < intervalMs) return false;

        const float fclk = dev.fclkHz();
        const float avdd = dev.avddAvss();
        /* Nothing measured yet. Publishing zeros here would put a hard clock
         * fault into the series for what is only a cold start. */
        if (!std::isfinite(fclk) && !std::isfinite(avdd)) return false;

        lastPublishMs = now;
        published = true;

        lastSample.setTimeNow();
        /* Field reuse, as on the zero channel: u = fCLK in Hz, i = AVDD-AVSS in
         * volts. Sample is exactly 32 bytes and WireSample 64, both pinned by
         * static_assert in main_esp32.cpp and parsed by the bridge on rpi.local,
         * so a dedicated field would be a cross-repo wire-format change for a
         * diagnostic series. NAN stays NAN -- "not measured" must not become 0. */
        lastSample.u = fclk;
        lastSample.i = avdd;
        /* Explicitly 0, not NAN: Sample::p() falls back to u*i when p_ is NAN,
         * which would multiply hertz by volts and integrate the result into
         * EnergyCounter as energy. */
        lastSample.p_ = 0.0f;
        lastSample.temp = dev.dieTempC();
        lastSample.diag = dev.diag();
        return true;
    }

    Sample getSample() override { return lastSample; }
};
