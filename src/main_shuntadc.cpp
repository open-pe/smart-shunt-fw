// Bench bring-up for the pwr-metering shunt-adc board (revB) on an Adafruit
// Feather ESP32-S3. Deliberately standalone: no WiFi, no BLE, no I2C, no
// InfluxDB. The point is to answer, in order, the questions that must all be
// "yes" before any measurement means anything:
//
//   1. Does SPI reach the ADC at all?          -> register readback in init()
//   2. Is it on the EXTERNAL clock?            -> EXTCLK, checked every read
//   3. Are DRDY edges arriving?                -> edge/conversion counters
//   4. Do the four input pairs read plausibly? -> per-pair microvolts
//
// I2C is kept out on purpose: on this board SCLK is wired to GPIO4, which the
// Feather variant defines as SCL. Bringing up Wire would fight the ADC clock.
// GPIO7 (nCS here) is the variant's PIN_I2C_POWER; nothing in this build drives
// it as a power rail.

#include <Arduino.h>

#include "adc/ads1262.h"
#include "util.h"
#include "ble.h"
#include "energy_counter.h"   // WireSample -- the same struct telemetry.h sends

/* Stream the FIRST channel only, for now: J3 (IIN) raw differential volts plus
 * the ADC die temperature, over BLE. NO WiFi: the radio is never started, and
 * nothing here calls connect_wifi_async() or creates wifiTask.
 *
 * Sent as a WireSample, byte-identical to what telemetry.h pushes over the same
 * characteristic, so an existing BLE client parses it unchanged. Field mapping,
 * because it is NOT the usual one:
 *     data.u    = raw DIFFERENTIAL VOLTS at the ADC input (not a scaled bus V)
 *     data.i    = NAN -- shuntOhm is unknown, so there is no current to report
 *     data.temp = ADS1262 die temperature, degC
 * Publishing a made-up scale factor would put plausible wrong numbers on the
 * wire; NAN says "not measured" and cannot be mistaken for a reading. */
static constexpr Ads1262ShuntAdc::Pair STREAM_PAIR = Ads1262ShuntAdc::PAIR_IIN;
static constexpr const char *STREAM_CHANNEL = "IIN";
static constexpr const char *STREAM_DEVICE = "shunt-adc-IIN";

// Wiring to J2 on the shunt-adc board. J2 pin 7 (the revA /DRDY position) is
// left UNCONNECTED -- guard_host_header_pin7_open in the schematic notes.
static constexpr int8_t PIN_SCLK = 4;   // J2 SCLK
static constexpr int8_t PIN_DIN = 5;    // J2 DIN   (host MOSI)
static constexpr int8_t PIN_DOUT = 6;   // J2 DOUT  (host MISO) -- DRDY rides this
static constexpr int8_t PIN_NCS = 7;    // J2 nCS   (held low for the whole session)
static constexpr int8_t PIN_START = 16; // J2 START

static Ads1262ShuntAdc dev;
static BleSrv bleSrv;

/* Declared extern in ble.h and set by BleSrv::otaQuiesceHook while an OTA-over-BLE
 * transfer is running. Sampling must stop for the duration -- the transfer wants
 * the CPU, and we would otherwise keep hammering the SPI bus throughout. */
volatile bool g_samplingHalted = false;
static uint8_t wireIdx = 0;

static const char *pairName(uint8_t p) {
    switch (p) {
        case Ads1262ShuntAdc::PAIR_IIN:  return "IIN  J3 AIN0/1";
        case Ads1262ShuntAdc::PAIR_IOUT: return "IOUT J4 AIN2/3";
        case Ads1262ShuntAdc::PAIR_VIN:  return "VIN  J5 AIN4/5";
        case Ads1262ShuntAdc::PAIR_VOUT: return "VOUT J6 AIN6/7";
        default: return "?";
    }
}

/// One InfluxDB point per completed scan. Fields are deliberately named for what
/// they actually are: `v` is volts at the ADC input, not a scaled current.
static uint32_t publishOk = 0, publishSkipped = 0;

static void publish(float volts, float dieTempC) {
    /* No subscriber means nothing to send. Counted rather than ignored --
     * silence must not be mistakable for success. */
    if (!bleSrv.isConnected()) {
        ++publishSkipped;
        return;
    }

    WireSample ws{};
    ws.v = 1;
    ws.idx = wireIdx++;
    strncpy(ws.dev, STREAM_DEVICE, sizeof(ws.dev) - 1);
    ws.data.u = volts;          // see the field-mapping note above
    ws.data.i = NAN;
    ws.data.p_ = NAN;
    ws.data.e = NAN;
    ws.data.temp = dieTempC;
    ws.data.setTimeNow();
    ws.u_max = volts;
    ws.i_max = NAN;
    ws.diag = dev.diag();
    ws.crc = WireSample::compute_crc16((uint8_t *) &ws, (uint8_t *) &ws.crc - (uint8_t *) &ws);

    bleSrv.send((uint8_t *) &ws, sizeof(ws));
    ++publishOk;
}

void setup() {
    Serial.begin(115200);
    // Native USB CDC: give the host time to enumerate, but do not block forever
    // on a port nobody opens -- a headless run must still produce log output.
    for (uint32_t t0 = millis(); !Serial && (millis() - t0) < 3000;) delay(10);

    Serial.println();
    Serial.println("=== shunt-adc (ADS1262) bring-up ===");
    Serial.printf("SCLK=%d DIN=%d DOUT/DRDY=%d nCS=%d START=%d  (J2.7 open)\n",
                  PIN_SCLK, PIN_DIN, PIN_DOUT, PIN_NCS, PIN_START);
    Serial.println("expect: G=32, FIR, 20 SPS, internal 2.5V ref, EXTCLK=1");

    /* Before SPI claims any pin: is the ADC side driving DOUT at all? Answers
     * the one thing a register dump cannot -- "nobody is driving this" vs
     * "something is holding it low". */
    Serial.println("--- line probe (before SPI init) ---");
    Ads1262ShuntAdc::probePinDrive("DOUT/DRDY", PIN_DOUT);
    /* Outputs: can the ESP32 actually move each line? SCLK idles LOW in SPI
     * mode 1, so a line stuck high is either a dead pad or a short. */
    Ads1262ShuntAdc::probePinDriveOut("SCLK     ", PIN_SCLK);
    Ads1262ShuntAdc::probePinDriveOut("DIN      ", PIN_DIN);
    Ads1262ShuntAdc::probePinDriveOut("nCS      ", PIN_NCS);
    Ads1262ShuntAdc::probePinDriveOut("START    ", PIN_START);
    /* Arbiter: talk to the ADC with no SPI peripheral involved at all. */
    Ads1262ShuntAdc::bitbangProbe(PIN_SCLK, PIN_DIN, PIN_DOUT, PIN_NCS);
    Serial.println("--- end line probe ---");

    bleSrv.begin();
    Serial.println("BLE started -- advertising, telemetry as WireSample per scan");

#ifdef SHUNTADC_BUS_EXERCISER
    /* Bench mode: clock the bus continuously so it can be traced with a meter.
     * Never returns. Opt-in via -D SHUNTADC_BUS_EXERCISER, because it replaces
     * normal operation entirely. */
    Ads1262ShuntAdc::busExerciser(PIN_SCLK, PIN_DIN, PIN_DOUT, PIN_NCS, PIN_START);
#endif
    Serial.println();
}

void loop() {
    static bool up = false;
    static uint32_t lastGen = 0, lastReport = 0, accepted = 0, pumpCalls = 0;

    if (!up) {
        Serial.println("init...");
        up = dev.init(PIN_SCLK, PIN_DIN, PIN_DOUT, PIN_NCS, PIN_START);
        if (!up) {
            Serial.printf("init FAILED (halFaults=%u)\n", (unsigned) ads126xHalFaults());
            static bool probed = false;
            if (!probed) {  // once: the dump is long and does not change while broken
                probed = true;
                dev.diagnose();
            }
            delay(2000);
            return;
        }
        Serial.println("init OK -- SPI readback matched and EXTCLK confirmed");
        dev.chopExperiment();   // once, before normal scanning resumes
        lastReport = millis();
        return;
    }

    bleSrv.tick();
    if (g_samplingHalted) {   // OTA in progress: leave the bus and the CPU alone
        delay(10);
        return;
    }

    ++pumpCalls;
    if (dev.pump()) ++accepted;

    /* A completed scan means all four pairs have a fresh reading. At 20 SPS with
     * 3 settling discards per mux change that is ~600 ms per scan. */
    if (dev.generation() != lastGen) {
        lastGen = dev.generation();
        Serial.printf("--- scan %u ---\n", (unsigned) lastGen);
        for (uint8_t p = 0; p < Ads1262ShuntAdc::PAIR_COUNT; p++) {
            const float v = dev.volts((Ads1262ShuntAdc::Pair) p);
            Serial.printf("  %s : %+10.3f uV%s\n", pairName(p), v * 1e6f,
                          p == STREAM_PAIR ? "   <- streamed" : "");
        }
        Serial.printf("  die temperature : %.3f C\n", dev.dieTempC());
        publish(dev.volts(STREAM_PAIR), dev.dieTempC());

        const uint32_t diag = dev.diag();
        if (diag)
            Serial.printf("  diag=0x%08x (reason=%u sign=%u code=%u)\n", (unsigned) diag,
                          (unsigned) (diag >> 24), (unsigned) ((diag >> 20) & 1),
                          (unsigned) (diag & 0xFFFFF));
    }

    if (millis() - lastReport >= 5000) {
        lastReport = millis();
        /* Measured scan period, not modelled. This is the number that matters
         * for multiplex skew: every pair restarts conversions, so a scan pays
         * the FIRST-conversion latency four times, and chop doubles that
         * (SBAS661C Eq.19) even though steady-state throughput barely moves
         * (Eq.21: 19.15 vs 20.00 SPS). */
        static uint32_t genAtLastReport = 0, tAtLastReport = 0;
        const uint32_t gen = dev.generation(), now = millis();
        const uint32_t dGen = gen - genAtLastReport, dT = now - tAtLastReport;
        Serial.printf("[health] scans=%u accepted=%u pumps=%u halFaults=%u | ble=%s "
                      "sent=%u skipped=%u",
                      (unsigned) gen, (unsigned) accepted, (unsigned) pumpCalls,
                      (unsigned) ads126xHalFaults(),
                      bleSrv.isConnected() ? "CONNECTED" : "advertising",
                      (unsigned) publishOk, (unsigned) publishSkipped);
        if (dGen && tAtLastReport)
            Serial.printf(" | scan=%.1f ms (%.1f ms/pair)", (float) dT / (float) dGen,
                          (float) dT / (float) dGen / (float) Ads1262ShuntAdc::PAIR_COUNT);
        Serial.println();
        genAtLastReport = gen;
        tAtLastReport = now;
        if (accepted == 0)
            Serial.println("[health] NO conversions accepted -- see the reason logged above; "
                           "a silent stall here is the DRDY-on-DOUT path, not the SPI path");
    }
}
