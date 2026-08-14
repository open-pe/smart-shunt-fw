/*
 * ti_ads126x.h -- Arduino-facing entry point for TI's ADS1262/ADS1263 driver.
 *
 * Include THIS header, not ads1263.h directly: the upstream headers have no
 * extern "C" guards, so including them raw from C++ gives the prototypes C++
 * linkage and they will not resolve against the C objects built from
 * ads1263.c.
 *
 * Usage:
 *
 *     #include <ti_ads126x.h>
 *
 *     Ads126xHalConfig cfg;
 *     cfg.spi           = &SPI;
 *     cfg.spiHz         = 8000000;      // ADS1262 max SCLK is 8 MHz
 *     cfg.pinCS         = 10;
 *     cfg.pinSTART      = 9;            // -1 if tied high / unused
 *     cfg.pinResetPwdn  = 8;            // -1 if tied high
 *     cfg.pinDRDY       = 7;            // -1 if not wired
 *
 *     SPI.begin(sck, miso, mosi, cs);
 *     if (!ads126xHalBegin(cfg)) return;   // config error -- do not proceed
 *
 *     adcStartupRoutine();              // upstream: reset, defaults, START high
 *     if (waitForDRDYinterrupt(100)) {
 *         uint8_t status, crc;
 *         uint8_t data[4];
 *         int32_t raw = readData(&status, data, &crc);
 *     }
 *
 * See README.md for the ADS1262-vs-ADS1263 register-count caveat.
 */

#ifndef TI_ADS126X_H_
#define TI_ADS126X_H_

#include <Arduino.h>
#include <SPI.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include "ads1263.h"    // pulls in hal.h
}

/** Pin/bus configuration for the Arduino HAL. Use -1 for a signal that is not
 *  wired to the MCU (strapped on the board instead). */
struct Ads126xHalConfig {
    SPIClass *spi = nullptr;
    uint32_t spiHz = 8000000;   ///< ADS1262/ADS1263 max SCLK = 8 MHz
    int8_t pinCS = -1;          ///< /CS -- REQUIRED
    int8_t pinSTART = -1;       ///< START; -1 => use the START/STOP opcodes instead
    int8_t pinResetPwdn = -1;   ///< /RESET//PWDN (one shared pin on this part)
    int8_t pinDRDY = -1;        ///< /DRDY; -1 => waitForDRDYinterrupt() always fails

    /**
     * Assert /CS low once at begin() and leave it low forever, instead of
     * framing each transfer with it.
     *
     * Required by boards that read the ready signal off the DOUT/DRDY pin:
     * SBAS661C sec.9.4.5 says "CS must be low to enable the DOUT/DRDY pin", so
     * a /CS that returns high between transfers disables the ready function
     * outright and no edge ever arrives.
     *
     * The cost, SBAS661C sec.10.1.9: "If CS is tied low, glitches at SCLK
     * power on can interrupt synchronization to the serial interface and must
     * be avoided. In this case, reset the ADC using the RESET/PWDN input." A
     * board in this mode with no /RESET pin brought out can only recover by
     * power cycling -- see `onBusBusy` for the masking that keeps the host from
     * making it worse.
     *
     * setCS() still opens and closes the SPI transaction and still fires
     * `onBusBusy`, so the logical frame around each transfer is unchanged; only
     * the physical pin stops moving.
     */
    bool csHoldLow = false;

    /**
     * Called with true immediately before the first SCLK of a transfer and
     * false immediately after the last, from whatever context drives the ADC.
     *
     * Exists for the DRDY-rides-DOUT wiring: the ready line IS the data line,
     * so an edge-triggered interrupt on it fires on every one of the 40+ data
     * bits in a conversion read (SBAS661C sec.9.4.5). The handler must be
     * masked for the whole transfer and any edge latched during it discarded.
     * Keep the callback ISR-safe and allocation-free.
     */
    void (*onBusBusy)(bool busy) = nullptr;

    /**
     * The ready signal shares the DOUT pin rather than having its own line.
     * Makes waitForDRDYinterrupt() refuse instead of polling: that pin belongs
     * to the SPI peripheral, so digitalRead() of it means nothing, and a
     * poll that returned "ready" off it would be pure invention. Hosts in this
     * wiring must use an interrupt plus `onBusBusy` masking.
     */
    bool drdyOnDout = false;
};

/**
 * Configures the HAL. Does NOT call spi->begin() -- do that yourself, so the
 * bus can be shared and the pin mapping stays with the application.
 *
 * @return true if the configuration is usable. false if `spi` is null or
 *         `pinCS` is unset; in that case the HAL stays un-ready and every
 *         subsequent SPI helper reports a fault rather than returning
 *         plausible-looking data.
 */
bool ads126xHalBegin(const Ads126xHalConfig &cfg);

/** True once ads126xHalBegin() has succeeded. */
bool ads126xHalReady();

/**
 * Number of HAL operations that could not be carried out (SPI transfer while
 * un-configured, DRDY wait with no DRDY pin, ...). A nonzero count means some
 * value returned by the ADS1263 module was fabricated, not read from the part.
 * Check it after a read sequence; it never decreases except via
 * ads126xHalClearFaults().
 */
uint32_t ads126xHalFaults();

/** Resets the fault counter. */
void ads126xHalClearFaults();

/** Reports whether a given signal is actually wired to the MCU. getCS() and
 *  friends return the last *commanded* state, which for an unwired signal is
 *  a wish, not a measurement. */
bool ads126xHalPinConnected(int8_t pin);

#endif /* TI_ADS126X_H_ */
