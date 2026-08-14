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
    enum Pair : uint8_t {
        PAIR_IIN = 0,   ///< J3 J_IIN,  AIN0(+)/AIN1(-)
        PAIR_IOUT,      ///< J4 J_IOUT, AIN2(+)/AIN3(-)
        PAIR_VIN,       ///< J5 J_VIN,  AIN4(+)/AIN5(-)
        PAIR_VOUT,      ///< J6 J_VOUT, AIN6(+)/AIN7(-)
        PAIR_COUNT
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

    static constexpr uint8_t MODE1_VALUE = 0x80;
    static_assert(SETTLE_DISCARDS > 0 || MODE1_VALUE == 0x80,
                  "SETTLE_DISCARDS=0 relies on the FIR filter's zero latency "
                  "(SBAS661C sec.9.4.2). A sinc2..sinc5 filter needs discards > 0.");

    /// Conversions averaged per pair before a reading is published. Noise falls
    /// as sqrt(N). Sized for the ~2.5 SPS on-air target: SBAS661C sec.9.4.2 says
    /// only the FIRST conversion after a START costs td(STDR) = 52.22 ms; in
    /// continuous mode the rest arrive at the nominal 20 SPS = 50 ms. So one pair
    /// with N=8 is 52.22 + 7*50 = 402 ms => 2.49 SPS.
    static constexpr uint8_t AVG_N = 8;

    /// A missed conversion presents as a line stuck low with NO further edges
    /// (note N11 b), so an edge-only wait would hang forever. After this long
    /// without an edge we read anyway, which re-arms the pin. Must exceed
    /// FIRST_CONVERSION_MS or a healthy start looks like a stall.
    static constexpr uint32_t DRDY_TIMEOUT_MS = 4 * FIRST_CONVERSION_MS;

    /// Budget for confirming EXTCLK at init. Several conversion periods, so a
    /// slow or retried first conversion cannot be mistaken for a missing clock.
    static constexpr uint32_t EXTCLK_CONFIRM_MS = 8 * FIRST_CONVERSION_MS;

    // diag reason codes, continuing the convention in ina228.h (1=out-of-range, 2=jump)
    static constexpr uint8_t DIAG_CHECKSUM = 3;
    static constexpr uint8_t DIAG_DEVICE_RESET = 4;
    static constexpr uint8_t DIAG_HAL_FAULT = 5;
    static constexpr uint8_t DIAG_NO_EXTCLK = 6;

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
    bool isrAttached = false;      ///< attachInterrupt() is not idempotent; do it once
    uint8_t pair = 0;              ///< pair currently selected in INPMUX
    uint8_t discardsLeft = SETTLE_DISCARDS;
    uint32_t lastEdgeMs = 0;

    float volts_[PAIR_COUNT] = {NAN, NAN, NAN, NAN};
    float dieTempC_ = NAN;   ///< ADS1262 internal temperature sensor, degC
    float sd_[PAIR_COUNT] = {NAN, NAN, NAN, NAN};   ///< sample stddev of the last average

    /* Running accumulation for the pair currently being averaged. */
    double sum_ = 0, sumsq_ = 0;
    uint8_t nAvg_ = 0;

    /* When set, the mux stays on this pair instead of scanning. Costs no restart
     * between conversions, so all of the time budget goes into averaging. */
    int8_t onlyPair_ = -1;
    uint32_t generation_ = 0;      ///< bumped when all four pairs have a reading

    /* Two diag slots, because TWO facades read this one device. A single
     * take-and-clear slot meant whichever facade polled second always saw 0 --
     * reporting "no problem" for a fault it simply never got to observe.
     * diag_ accumulates within the current scan; diagPublished_ is latched when
     * the scan completes and is READ, not consumed, so both facades see it. */
    uint32_t diag_ = 0;
    uint32_t diagPublished_ = 0;

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
        pair = p;
        discardsLeft = SETTLE_DISCARDS;
        sum_ = sumsq_ = 0;
        nAvg_ = 0;
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
    void setOnlyPair(int8_t p) { onlyPair_ = p; }

    /// Sample standard deviation of the conversions behind volts(p). This is the
    /// measured noise, not an assumed one -- use it to choose AVG_N.
    float voltsStdDev(Pair p) const { return sd_[p]; }

    /// ADS1262 die temperature in degC, NAN until the first scan completes.
    /// SBAS661C sec.9.3.4 notes the die runs ~0.7 degC above the surrounding
    /// PCB from self-heating, so this tracks board temperature with an offset.
    float dieTempC() const { return dieTempC_; }

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
                {REG_ADDR_INPMUX,    INPMUX_FOR[PAIR_IIN]},
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
                                    "or the DOUT wire is not landing on the MISO pin.");
            else if (all_and == 0xFF)
                ESP_LOGE("ads1262", "every register reads 0xFF -- DOUT idle high, which is what "
                                    "an UNPOWERED ISOLATED SIDE looks like. Check J1's 5 V feed.");
            else
                ESP_LOGE("ads1262", "partial readback -- suspect SCLK integrity, SPI mode, or a "
                                    "marginal connection.");
            return false;
        }

        if (ads126xHalFaults()) {
            ESP_LOGE("ads1262", "%u HAL faults during init -- readback was not real",
                     (unsigned) ads126xHalFaults());
            return false;
        }

        uint8_t id = readSingleRegister(REG_ADDR_ID);
        ESP_LOGI("ads1262", "found %s (ID=0x%02x), G=%u, %s",
                 (id & ID_DEV_MASK) == ID_DEV_ADS1262 ? "ADS1262" : "ADS1263", id,
                 PGA_GAIN, "FIR @ 20 SPS");

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
        writeSingleRegister(REG_ADDR_INPMUX, INPMUX_FOR[PAIR_IIN]);
        (void) readSingleRegister(REG_ADDR_MODE0);
        (void) readSingleRegister(REG_ADDR_MODE1);
        (void) readSingleRegister(REG_ADDR_MODE2);
        (void) readSingleRegister(REG_ADDR_INPMUX);

        pair = PAIR_IIN;
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

    /// Restores the measurement configuration the self-tests and the temperature
    /// read overwrite. Reads every register back: upstream's writeSingleRegister()
    /// does NOT update the driver's shadow map, only readSingleRegister() does.
    void applyProductionConfig() {
        setSTART(LOW);
        writeSingleRegister(REG_ADDR_MODE0, MODE0_VALUE);
        writeSingleRegister(REG_ADDR_MODE1, MODE1_VALUE);
        writeSingleRegister(REG_ADDR_MODE2, (uint8_t) ((PGA_GAIN_CODE << 4) | DATA_RATE_CODE));
        writeSingleRegister(REG_ADDR_REFMUX, 0x00);
        writeSingleRegister(REG_ADDR_INPMUX, INPMUX_FOR[PAIR_IIN]);
        (void) readSingleRegister(REG_ADDR_MODE0);
        (void) readSingleRegister(REG_ADDR_MODE1);
        (void) readSingleRegister(REG_ADDR_MODE2);
        (void) readSingleRegister(REG_ADDR_REFMUX);
        (void) readSingleRegister(REG_ADDR_INPMUX);
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
    bool readDieTemperature() {
        configureSelfTest((uint8_t) (INPMUX_MUXP_TEMP_P | INPMUX_MUXN_TEMP_N));
        int32_t code = 0;
        const bool ok = oneConversion(code);
        if (ok) {
            constexpr float lsb1 = VREF / (float) (2u << 30);   // gain = 1 here
            const float uV = (float) code * lsb1 * 1e6f;
            dieTempC_ = (uV - 122400.0f) / 420.0f + 25.0f;
        }
        applyProductionConfig();
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

    /// Reads one conversion at most, advancing the mux when it is accepted.
    /// Returns true if a reading was stored.
    bool pump() {
        if (!initialized) return false;

        if (!waitForEdge()) return false;

        uint8_t status = 0, checksum = 0, data[4] = {0};
        const uint32_t faultsBefore = ads126xHalFaults();

        /* sec.9.4.6: no serial activity may occur between DRDY going low and the
         * readback, or the data are invalid. So this is the FIRST bus access
         * after the edge -- no register polling in between. */
        int32_t count = readData(&status, data, &checksum);

        if (ads126xHalFaults() != faultsBefore) {
            diag_ = encodeDiag(DIAG_HAL_FAULT, 0);
            ESP_LOGW("ads1262", "HAL fault during read");
            return false;
        }

        if (checksum != calculateChecksum(data, 4)) {
            diag_ = encodeDiag(DIAG_CHECKSUM, count);
            ESP_LOGW("ads1262", "checksum mismatch");
            return false;
        }

        if (status & STATUS_RESET) {
            diag_ = encodeDiag(DIAG_DEVICE_RESET, count);
            ESP_LOGE("ads1262", "device reset flag set -- configuration lost, re-init needed");
            initialized = false;    // force a full re-init rather than trusting the part
            return false;
        }

        /* THE mandatory per-acquisition check (note N10). Absence of the
         * external clock must never read as "clock fine". */
        if (!(status & STATUS_EXTCLK)) {
            diag_ = encodeDiag(DIAG_NO_EXTCLK, count);
            ESP_LOGE("ads1262", "EXTCLK=0: ADC on internal oscillator, data UNSYNCHRONISED "
                                "-- refusing sample");
            return false;
        }

        if (!(status & STATUS_ADC1))
            return false;           // not a fresh conversion; re-reading would bias averages

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

        if (onlyPair_ >= 0) {
            /* Single-pair mode: do NOT touch INPMUX, so conversions keep arriving
             * at the nominal rate with no restart latency. */
            readDieTemperature();
            applyProductionConfig();
            selectPair((uint8_t) onlyPair_);
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
            readDieTemperature();

            /* A full scan of all four pairs is complete. Latch whatever
             * diagnostics accumulated during it so every facade reading this
             * generation sees the same value, then start the next scan clean. */
            diagPublished_ = diag_;
            diag_ = 0;
            ++generation_;
            selectPair(PAIR_IIN);
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
                                "The converter is not running at all -- check AVDD/AVSS and that "
                                "a clock reaches XTAL1.",
                     (unsigned) window, SELFTEST_NOMINAL_SPS);
            return false;
        }

        const float sps = (float) n * 1000.0f / (float) window;
        const float fclk = FCLK_NOMINAL_HZ * (sps / SELFTEST_NOMINAL_SPS);

        if (fclk < FCLK_MIN_HZ || fclk > FCLK_MAX_HZ) {
            ESP_LOGE("ads1262",
                     "SELF-TEST FAIL: fCLK ~ %.0f Hz, outside the %.0f..%.0f Hz the part accepts "
                     "(expected %.0f). EXTCLK reads 1 because SOMETHING is on XTAL1, but it is "
                     "not a valid clock -- check Y1 is oscillating, its supply and output enable, "
                     "J8, and the isolator channel carrying the clock.",
                     fclk, FCLK_MIN_HZ, FCLK_MAX_HZ, FCLK_NOMINAL_HZ);
            return false;
        }
        if (fabsf(fclk - FCLK_NOMINAL_HZ) / FCLK_NOMINAL_HZ > FCLK_WARN_TOLERANCE)
            ESP_LOGW("ads1262", "fCLK ~ %.0f Hz is more than %.0f%% off the expected %.0f Hz. "
                                "Conversion timing and the 50/60 Hz filter nulls scale with it.",
                     fclk, FCLK_WARN_TOLERANCE * 100.0f, FCLK_NOMINAL_HZ);
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
    bool measureZero(uint8_t chopBits, uint16_t n, float &meanUv, float &sdUv, float &sps) {
        setSTART(LOW);
        writeSingleRegister(REG_ADDR_MODE0, chopBits);
        writeSingleRegister(REG_ADDR_MODE1, MODE1_VALUE);           // production filter
        writeSingleRegister(REG_ADDR_MODE2,
                            (uint8_t) ((PGA_GAIN_CODE << 4) | DATA_RATE_CODE));
        writeSingleRegister(REG_ADDR_INPMUX,
                            (uint8_t) (INPMUX_MUXP_AINCOM | INPMUX_MUXN_AINCOM));
        sendCommand(OPCODE_STOP1);
        setSTART(HIGH);
        sendCommand(OPCODE_START1);

        constexpr float lsbUv = VREF / (float) PGA_GAIN / (float) (2u << 30) * 1e6f;
        /* Chop doubles the first-conversion latency, so allow generously and
         * discard the settling conversions rather than assuming a count. */
        const uint32_t budgetMs = 2000 + (uint32_t) n * 400;
        double sum = 0, sumSq = 0;
        uint32_t got = 0, first = 0, last = 0, discard = 2;

        for (uint32_t t0 = millis(); (uint32_t) (millis() - t0) < budgetMs && got < n;) {
            uint8_t status = 0, checksum = 0, data[4] = {0};
            const uint32_t faultsBefore = ads126xHalFaults();
            const int32_t code = readData(&status, data, &checksum);
            if (ads126xHalFaults() != faultsBefore) return false;
            if (!(status & STATUS_ADC1)) continue;
            if (checksum != calculateChecksum(data, 4)) continue;
            if (discard) { --discard; continue; }

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
    if ((uint32_t) (millis() - lastEdgeMs) >= DRDY_TIMEOUT_MS) {
        ESP_LOGW("ads1262", "no DRDY edge for %u ms -- reading to re-arm",
                 (unsigned) DRDY_TIMEOUT_MS);
        lastEdgeMs = millis();
        return true;
    }
    return false;
}


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
                          int8_t pinCs, int8_t pinStart)
            : dev(dev), uPair(uPair), iPair(iPair), shuntOhm(shuntOhm),
              dividerRatio(dividerRatio), storageId(storageId), pinSck(pinSck), pinMosi(pinMosi),
              pinMiso(pinMiso), pinCs(pinCs), pinStart(pinStart) {}

    uint8_t getStorageId() const override { return storageId; }

    bool init() override {
        if (!(shuntOhm > 0) || !(dividerRatio > 0)) {
            ESP_LOGE("ads1262", "shuntOhm/dividerRatio not configured");
            return false;
        }
        // idempotent: whichever facade initialises first brings up the shared ADC
        return dev.init(pinSck, pinMosi, pinMiso, pinCs, pinStart);
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
        }

        dev.pump();

        /* Only publish on a COMPLETED scan, so u and i belong to the same pass
         * over the multiplexer. They are still not simultaneous -- one ADC
         * cannot be -- but they are at least adjacent and consistently ordered. */
        if (dev.generation() == lastGen) return false;
        lastGen = dev.generation();

        const float vU = dev.volts(uPair), vI = dev.volts(iPair);
        if (std::isnan(vU) || std::isnan(vI)) return false;

        lastSample.setTimeNow();
        lastSample.u = vU * dividerRatio;
        lastSample.i = vI / shuntOhm;
        lastSample.temp = dev.dieTempC();
        /* Non-consuming: the other facade sharing this device must see the same
         * diagnostic rather than a zero because we got here first. */
        lastSample.diag = dev.diag();
        return true;
    }

    Sample getSample() override { return lastSample; }
};
