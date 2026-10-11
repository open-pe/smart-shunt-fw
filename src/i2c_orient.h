#pragma once

/*
 * Boot-time check for a TMP117 wired with SDA and SCL swapped.
 *
 * A TMP117 added to a harness by hand can easily end up with its SDA on the bus's SCL
 * line and vice versa. On such a part every normal transfer just NACKs, so it reads as
 * "absent" -- the same as no part at all -- and nothing says why.
 *
 * This runs BEFORE Wire is started on the bus, bit-banged on the raw pins, so it never
 * has to tear the IDF driver down and bring it back (see i2c_recover_bus() for how
 * fragile that is). It is a DIAGNOSTIC ONLY: it never runs the bus swapped afterwards.
 * On a bus shared with other parts -- the two INA228s on the stackVolt harness share
 * GPIO3/GPIO2 with the TMP117s -- a swapped bus would break every other device on it,
 * so the only correct fix for a swapped TMP117 is the wiring. The check exists to say
 * so in the boot log and on the `i2c` console command instead of leaving "init failed".
 *
 * Safe for the other parts on the bus, but only because of WHEN it runs:
 *  - The swapped pass clocks the shared bus with the roles reversed, which the other
 *    parts see as random START/STOP/data. That could in principle land a write in an
 *    INA228 register. It runs before samplers.initAll(), and PowerSampler_INA228::init()
 *    begins with a CONFIG.RST (0x8000) and then rewrites SHUNT_CAL and every config
 *    register from NVS, so nothing written here survives. The INA228 has no NVM.
 *  - It ends with a 9-clock bus clear and a STOP in the NORMAL orientation, so no part
 *    is left mid-byte holding SDA when Wire starts.
 *  - The swapped pass only runs if NOTHING in 0x48..0x4B ACKs normally, so a board whose
 *    TMP117s are wired correctly never sees reversed traffic at all.
 */

#include <Arduino.h>

namespace i2c_orient {

enum class Result : uint8_t { NOT_RUN, NORMAL, SWAPPED, NONE };

struct Report {
    Result result = Result::NOT_RUN;
    uint8_t normalMask = 0;  // bit n: 0x48+n ACKed in the normal orientation
    uint8_t swappedMask = 0; // bit n: 0x48+n read DEVICE_ID 0x0117 with SDA/SCL swapped
    int8_t sda = -1, scl = -1;
};

inline Report &report() {
    static Report r;
    return r;
}

/// Open-drain bit-bang master on two pins. ~50 kHz (10 us half period); every SCL
/// release waits, bounded, for the line to rise (pull-up rise, or clock stretching).
class BitBang {
public:
    BitBang(uint8_t sda, uint8_t scl) : sda(sda), scl(scl) {
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, INPUT_PULLUP);
        digitalWrite(sda, HIGH); // latch high before the output is enabled: no glitch low
        digitalWrite(scl, HIGH);
        pinMode(sda, OUTPUT_OPEN_DRAIN | PULLUP);
        pinMode(scl, OUTPUT_OPEN_DRAIN | PULLUP);
        delayMicroseconds(SETTLE_US);
    }

    /// Hands the pins back as plain inputs with pull-ups, for Wire to take over.
    ~BitBang() {
        pinMode(sda, INPUT_PULLUP);
        pinMode(scl, INPUT_PULLUP);
    }

    bool linesHigh() { return digitalRead(sda) && digitalRead(scl); }

    void start() {
        sdaHigh();
        sclHigh();
        digitalWrite(sda, LOW);
        delayMicroseconds(HALF_US);
        sclLow();
    }

    void stop() {
        digitalWrite(sda, LOW);
        delayMicroseconds(HALF_US);
        sclHigh();
        sdaHigh();
    }

    /// True on ACK.
    bool write(uint8_t b) {
        for (int i = 7; i >= 0; --i) {
            digitalWrite(sda, (b >> i) & 1);
            delayMicroseconds(HALF_US);
            sclHigh();
            sclLow();
        }
        sdaHigh();
        sclHigh();
        const bool ack = !digitalRead(sda);
        sclLow();
        return ack;
    }

    uint8_t read(bool ack) {
        uint8_t b = 0;
        sdaHigh();
        for (int i = 0; i < 8; ++i) {
            sclHigh();
            b = (uint8_t) ((b << 1) | (digitalRead(sda) ? 1 : 0));
            sclLow();
        }
        digitalWrite(sda, ack ? LOW : HIGH);
        delayMicroseconds(HALF_US);
        sclHigh();
        sclLow();
        sdaHigh();
        return b;
    }

    /// Address-only probe (write direction, no data).
    bool probe(uint8_t addr) {
        start();
        const bool ack = write((uint8_t) (addr << 1));
        stop();
        return ack;
    }

    /// 16-bit register read with a repeated start. False on any NACK.
    bool readReg16(uint8_t addr, uint8_t reg, uint16_t &out) {
        start();
        bool ok = write((uint8_t) (addr << 1)) && write(reg);
        if (ok) {
            start(); // repeated start
            ok = write((uint8_t) ((addr << 1) | 1));
            if (ok) {
                const uint8_t hi = read(true);
                const uint8_t lo = read(false);
                out = (uint16_t) ((hi << 8) | lo);
            }
        }
        stop();
        return ok;
    }

    /// Release a part left mid-byte (up to 9 clocks while SDA is low), then STOP.
    void clear() {
        sdaHigh();
        for (int i = 0; i < 9 && !digitalRead(sda); ++i) {
            sclLow();
            sclHigh();
        }
        sclLow();
        stop();
    }

private:
    static constexpr uint32_t HALF_US = 10, SCL_RISE_MAX_US = 1000, SETTLE_US = 50;

    void sdaHigh() {
        digitalWrite(sda, HIGH);
        delayMicroseconds(HALF_US);
    }

    void sclHigh() {
        digitalWrite(scl, HIGH);
        for (uint32_t us = 0; !digitalRead(scl) && us < SCL_RISE_MAX_US; ++us) delayMicroseconds(1);
        delayMicroseconds(HALF_US);
    }

    void sclLow() {
        digitalWrite(scl, LOW);
        delayMicroseconds(HALF_US);
    }

    uint8_t sda, scl;
};

/// Probe 0x48..0x4B with the pins as configured; if nothing answers, try reading the
/// TMP117 DEVICE_ID with SDA and SCL swapped. MUST run before Wire is started on these
/// pins and before any sampler configures its part (see the file comment).
inline const Report &checkTmp117Orientation(int8_t sda, int8_t scl) {
    static constexpr char TAG[] = "i2c_orient";
    static constexpr uint8_t ADDR_MIN = 0x48, ADDR_MAX = 0x4B, REG_DEVICE_ID = 0x0F;
    static constexpr uint16_t DEVICE_ID = 0x0117;

    Report &r = report();
    r = Report{};
    r.sda = sda, r.scl = scl;
    if (sda < 0 || scl < 0) return r;

    {
        BitBang bb((uint8_t) sda, (uint8_t) scl);
        if (!bb.linesHigh()) bb.clear();
        for (uint8_t a = ADDR_MIN; a <= ADDR_MAX; ++a)
            if (bb.probe(a)) r.normalMask |= (uint8_t) (1u << (a - ADDR_MIN));
        bb.clear();
    }
    if (r.normalMask) {
        r.result = Result::NORMAL;
        ESP_LOGI(TAG, "SDA=GPIO%d SCL=GPIO%d: TMP117-range ACK mask 0x%X (bit0 = 0x48), "
                      "orientation normal", sda, scl, r.normalMask);
        return r;
    }

    {
        BitBang swapped((uint8_t) scl, (uint8_t) sda); // roles reversed
        for (uint8_t a = ADDR_MIN; a <= ADDR_MAX; ++a) {
            uint16_t id = 0;
            if (swapped.readReg16(a, REG_DEVICE_ID, id) && (id & 0x0FFF) == DEVICE_ID)
                r.swappedMask |= (uint8_t) (1u << (a - ADDR_MIN));
        }
        swapped.clear();
    }
    {
        // Leave the bus idle in the NORMAL orientation before Wire takes it.
        BitBang bb((uint8_t) sda, (uint8_t) scl);
        bb.clear();
    }

    if (r.swappedMask) {
        r.result = Result::SWAPPED;
        ESP_LOGE(TAG, "TMP117 answers ONLY WITH SDA/SCL SWAPPED (mask 0x%X, bit0 = 0x48): its "
                      "SDA is on GPIO%d and its SCL on GPIO%d. Not usable -- this bus is "
                      "shared, so the firmware will not run it swapped. Swap the two wires.",
                 r.swappedMask, scl, sda);
    } else {
        r.result = Result::NONE;
        ESP_LOGW(TAG, "SDA=GPIO%d SCL=GPIO%d: no TMP117 at 0x48..0x4B in either "
                      "orientation (absent, unpowered, or no ground?)", sda, scl);
    }
    return r;
}

inline const char *resultName(Result r) {
    switch (r) {
        case Result::NOT_RUN: return "not run";
        case Result::NORMAL:  return "normal";
        case Result::SWAPPED: return "SWAPPED (rewire SDA/SCL)";
        case Result::NONE:    return "no TMP117 either way";
    }
    return "?";
}

} // namespace i2c_orient
