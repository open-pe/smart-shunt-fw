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

// Wiring to J2 on the shunt-adc board. J2 pin 7 (the revA /DRDY position) is
// left UNCONNECTED -- guard_host_header_pin7_open in the schematic notes.
static constexpr int8_t PIN_SCLK = 4;   // J2 SCLK
static constexpr int8_t PIN_DIN = 5;    // J2 DIN   (host MOSI)
static constexpr int8_t PIN_DOUT = 6;   // J2 DOUT  (host MISO) -- DRDY rides this
static constexpr int8_t PIN_NCS = 7;    // J2 nCS   (held low for the whole session)
static constexpr int8_t PIN_START = 16; // J2 START

static Ads1262ShuntAdc dev;

static const char *pairName(uint8_t p) {
    switch (p) {
        case Ads1262ShuntAdc::PAIR_IIN:  return "IIN  J3 AIN0/1";
        case Ads1262ShuntAdc::PAIR_IOUT: return "IOUT J4 AIN2/3";
        case Ads1262ShuntAdc::PAIR_VIN:  return "VIN  J5 AIN4/5";
        case Ads1262ShuntAdc::PAIR_VOUT: return "VOUT J6 AIN6/7";
        default: return "?";
    }
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
    Serial.println("--- end line probe ---");
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
        lastReport = millis();
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
            Serial.printf("  %s : %+10.3f uV\n", pairName(p), v * 1e6f);
        }
        const uint32_t diag = dev.diag();
        if (diag)
            Serial.printf("  diag=0x%08x (reason=%u sign=%u code=%u)\n", (unsigned) diag,
                          (unsigned) (diag >> 24), (unsigned) ((diag >> 20) & 1),
                          (unsigned) (diag & 0xFFFFF));
    }

    if (millis() - lastReport >= 5000) {
        lastReport = millis();
        Serial.printf("[health] scans=%u accepted=%u pumps=%u halFaults=%u\n",
                      (unsigned) dev.generation(), (unsigned) accepted, (unsigned) pumpCalls,
                      (unsigned) ads126xHalFaults());
        if (accepted == 0)
            Serial.println("[health] NO conversions accepted -- see the reason logged above; "
                           "a silent stall here is the DRDY-on-DOUT path, not the SPI path");
    }
}
