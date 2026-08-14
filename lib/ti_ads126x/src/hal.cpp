/*
 * hal.cpp -- Arduino implementation of TI's ADS1262/ADS1263 HAL contract.
 *
 * Replaces upstream hal.c (MSP432E401Y + driverlib), kept verbatim in
 * ../upstream/hal.c. Timing constants and the delay/CS/START/RESET sequencing
 * are taken from that file so the behaviour ads1263.c relies on is preserved.
 *
 * SPI mode: the ADS1262/ADS1263 use SPI MODE 1 (CPOL=0, CPHA=1). Upstream's
 * InitSPI() configures SSI_FRF_MOTO_MODE_1; getting this wrong is the single
 * most common reason the part reads back all zeros or garbage.
 */

#include "ti_ads126x.h"
#include "hal.h"

namespace {

Ads126xHalConfig g_cfg;
bool g_ready = false;
uint32_t g_faults = 0;

/* Whether a beginTransaction() is currently open. setCS() opens/closes it, so
 * that a multi-byte sequence framed by setCS(LOW)...setCS(HIGH) in ads1263.c
 * runs inside ONE transaction -- SPI.beginTransaction() is not re-entrant on
 * ESP32 (it takes the bus mutex), so a transaction per byte would deadlock or
 * assert. */
bool g_inTransaction = false;

/* Last commanded level of each output. Used by the get*() test helpers, which
 * upstream implements as a real pin read; digitalRead() on an output pin is
 * not portable (and impossible for a pin we do not drive), so we shadow. */
bool g_stateCS = true;
bool g_stateStart = false;
bool g_stateResetPwdn = true;

inline void drive(int8_t pin, bool state) {
    if (pin >= 0) digitalWrite((uint8_t) pin, state ? HIGH : LOW);
}

}  // namespace

//*****************************************************************************
//
// Configuration
//
//*****************************************************************************

bool ads126xHalBegin(const Ads126xHalConfig &cfg) {
    g_ready = false;
    g_inTransaction = false;

    /* A HAL with no bus or no chip select cannot talk to the part at all.
     * Refuse rather than come up "configured" and hand back invented bytes. */
    if (cfg.spi == nullptr || cfg.pinCS < 0) {
        g_faults++;
        return false;
    }

    g_cfg = cfg;

    pinMode((uint8_t) g_cfg.pinCS, OUTPUT);
    /* In csHoldLow mode /CS is asserted here and never released -- the
     * DOUT/DRDY function on the ADC is gated on it (SBAS661C sec.9.4.5). */
    digitalWrite((uint8_t) g_cfg.pinCS, g_cfg.csHoldLow ? LOW : HIGH);
    g_stateCS = !g_cfg.csHoldLow;

    if (g_cfg.pinSTART >= 0) {
        pinMode((uint8_t) g_cfg.pinSTART, OUTPUT);
        digitalWrite((uint8_t) g_cfg.pinSTART, LOW);
    }
    g_stateStart = false;

    if (g_cfg.pinResetPwdn >= 0) {
        pinMode((uint8_t) g_cfg.pinResetPwdn, OUTPUT);
        digitalWrite((uint8_t) g_cfg.pinResetPwdn, HIGH);
    }
    g_stateResetPwdn = true;

    /* NEVER pinMode() the DRDY pin when it shares DOUT: that pin belongs to the
     * SPI peripheral, and arduino-esp32's pinMode() calls perimanClearPinBus()
     * and re-registers the pin as plain GPIO (esp32-hal-gpio.c:124/161),
     * DETACHING MISO from the SPI controller. The bus then reads a constant
     * 0x00 while the line is physically driven -- every register readback comes
     * back zero and nothing reports an error, because the transfers themselves
     * "succeed". Cost us a bench session on 2026-08-14.
     *
     * attachInterrupt() is safe here by contrast: it does not touch the pin bus,
     * so an edge interrupt can coexist with SPI ownership of the pin. */
    if (g_cfg.pinDRDY >= 0 && !g_cfg.drdyOnDout) pinMode((uint8_t) g_cfg.pinDRDY, INPUT);

    g_ready = true;
    return true;
}

bool ads126xHalReady() { return g_ready; }

uint32_t ads126xHalFaults() { return g_faults; }

void ads126xHalClearFaults() { g_faults = 0; }

bool ads126xHalPinConnected(int8_t pin) { return pin >= 0; }

//*****************************************************************************
//
// Initialization
//
//*****************************************************************************

void InitADCPeripherals(void) {
    /* Upstream brings up the MCU's SPI and GPIO here. On Arduino the
     * application owns SPI.begin() and the pin map, so that work is done in
     * ads126xHalBegin(); this is the remaining ADC-side startup. */
    if (!g_ready) {
        g_faults++;
        return;
    }
    setCS(true);
    adcStartupRoutine();
}

//*****************************************************************************
//
// Timing
//
//*****************************************************************************

void delay_ms(uint32_t delay_time_ms) { delay(delay_time_ms); }

void delay_ns(uint32_t delay_time_ns) {
    /* Rounded UP to whole microseconds: every caller is a minimum-width
     * requirement (td(SCCS)=80 ns, tRESET=2 tCLK=125 ns, START=250 ns), so
     * waiting longer is always safe and waiting less never is. */
    delayMicroseconds((delay_time_ns + 999u) / 1000u);
}

//*****************************************************************************
//
// GPIO
//
//*****************************************************************************

bool getCS(void) { return g_stateCS; }
bool getPWDN(void) { return g_stateResetPwdn; }
bool getRESET(void) { return g_stateResetPwdn; }
bool getSTART(void) { return g_stateStart; }

void setCS(bool state) {
    if (!g_ready) {
        g_faults++;
        return;
    }

    if (!state) {
        /* Entering a transfer. Mask the host's DRDY handler BEFORE any SCLK
         * can move -- on a DRDY-rides-DOUT board every data bit is an edge on
         * that line. */
        if (g_cfg.onBusBusy) g_cfg.onBusBusy(true);

        /* Open the SPI transaction first, so SCLK/mode are settled before the
         * part is selected. */
        if (!g_inTransaction) {
            g_cfg.spi->beginTransaction(SPISettings(g_cfg.spiHz, MSBFIRST, SPI_MODE1));
            g_inTransaction = true;
        }
        if (!g_cfg.csHoldLow) drive(g_cfg.pinCS, false);
    } else {
        if (!g_cfg.csHoldLow) drive(g_cfg.pinCS, true);
        if (g_inTransaction) {
            g_cfg.spi->endTransaction();
            g_inTransaction = false;
        }

        /* Unmask last, so the handler cannot see the tail of our own clocking. */
        if (g_cfg.onBusBusy) g_cfg.onBusBusy(false);
    }
    /* Track the PIN, not the request: in csHoldLow mode it is low throughout,
     * and getCS() must not claim otherwise. */
    g_stateCS = g_cfg.csHoldLow ? false : state;

    // td(SCCS) & td(CSSC) delay
    delay_ns(80);
}

void setPWDN(bool state) {
    drive(g_cfg.pinResetPwdn, state);
    g_stateResetPwdn = state;

    // Minimum nPWDN width: 2 tCLKs
    delay_ns(125);

    // Reset register array when powering down
    if (!state) { restoreRegisterDefaults(); }
}

void setRESET(bool state) {
    drive(g_cfg.pinResetPwdn, state);
    g_stateResetPwdn = state;
}

void setSTART(bool state) {
    drive(g_cfg.pinSTART, state);
    g_stateStart = state;

    // Minimum START width ~4 tCLKs
    // REFERENCE: https://e2e.ti.com/support/data-converters/f/73/p/431463/1543880
    delay_ns(250);
}

void toggleRESET(void) {
    drive(g_cfg.pinResetPwdn, false);

    // Minimum nRESET width: 2 tCLKs
    delay_ns(125);

    drive(g_cfg.pinResetPwdn, true);
    g_stateResetPwdn = true;
}

//*****************************************************************************
//
// /DRDY
//
//*****************************************************************************

bool waitForDRDYinterrupt(uint32_t timeout_ms) {
    /* Polled rather than interrupt-driven: an ISR would have to be registered
     * by the application anyway, and polling keeps this module free of
     * platform-specific attachInterrupt semantics.
     *
     * Every path that cannot OBSERVE /DRDY returns false. "I could not tell"
     * must not be reported as "conversion ready" -- that would make the caller
     * read stale or partial data and never notice.
     *
     * NOT USABLE on a board where DRDY rides the DOUT pin: that pin belongs to
     * the SPI peripheral, so digitalRead() of it is meaningless. Such a host
     * must drive the ready signal from an interrupt plus `onBusBusy` masking
     * instead, and set cfg.drdyOnDout so this function refuses rather than
     * inventing a verdict from a pin the SPI peripheral owns. */
    if (!g_ready || g_cfg.pinDRDY < 0 || g_cfg.drdyOnDout) {
        g_faults++;
        return false;
    }

    const uint32_t start = millis();
    do {
        if (digitalRead((uint8_t) g_cfg.pinDRDY) == LOW) return true;
    } while ((uint32_t) (millis() - start) < timeout_ms);

    /* One final read: with timeout_ms == 0 the loop body still runs once, and
     * for nonzero timeouts this closes the race on the last millisecond. */
    return digitalRead((uint8_t) g_cfg.pinDRDY) == LOW;
}

//*****************************************************************************
//
// SPI
//
//*****************************************************************************

void spiSendReceiveArrays(uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength) {
    if (!g_ready) {
        /* Do not leave DataRx holding whatever the caller's stack had; fill it
         * with 0xFF (the idle/bus-floating pattern) and count the fault, so a
         * caller that checks ads126xHalFaults() can tell this was not a read. */
        for (uint8_t i = 0; i < byteLength; i++) DataRx[i] = 0xFF;
        g_faults++;
        return;
    }

    setCS(false);
    for (uint8_t i = 0; i < byteLength; i++) {
        DataRx[i] = spiSendReceiveByte(DataTx[i]);
    }
    setCS(true);
}

uint8_t spiSendReceiveByte(uint8_t dataTx) {
    /* NOTE: does not control /CS -- ads1263.c brackets multi-byte sequences
     * (readMultipleRegisters, writeMultipleRegisters) with its own setCS()
     * calls, matching upstream. */
    if (!g_ready) {
        g_faults++;
        return 0xFF;
    }

    if (!g_inTransaction) {
        /* Called outside a setCS(LOW)...setCS(HIGH) frame. Upstream would have
         * clocked the bus regardless; do the same, but in a transaction so the
         * mode/clock are still correct -- and still mask the DRDY handler,
         * because this clocks SCLK just like any other transfer. */
        if (g_cfg.onBusBusy) g_cfg.onBusBusy(true);
        g_cfg.spi->beginTransaction(SPISettings(g_cfg.spiHz, MSBFIRST, SPI_MODE1));
        const uint8_t rx = g_cfg.spi->transfer(dataTx);
        g_cfg.spi->endTransaction();
        if (g_cfg.onBusBusy) g_cfg.onBusBusy(false);
        return rx;
    }

    return g_cfg.spi->transfer(dataTx);
}
