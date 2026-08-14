/*
 * hal.h -- Hardware abstraction layer for TI's ADS1262/ADS1263 example driver.
 *
 * This file REPLACES the upstream `hal.h`, which targets an MSP432E401Y and
 * pulls in TI driverlib. The function prototypes below are byte-for-byte the
 * contract that the *unmodified* upstream `ads1263.c` expects; only the
 * processor-specific parts (driverlib include, MSP432 port/pin macros,
 * `#include "settings.h"`) were dropped. The implementation lives in `hal.cpp`
 * and is written against the Arduino SPI/GPIO API.
 *
 * The original is kept verbatim in ../upstream/hal.h for reference.
 *
 * Configure the HAL through ti_ads126x.h (ads126xHalBegin) before calling any
 * of the ADS1263 module functions.
 */

#ifndef TI_ADS126X_HAL_H_
#define TI_ADS126X_HAL_H_

#include <stdbool.h>
#include <stddef.h>     /* NULL -- ads1263.c uses it but includes neither this
                         * nor <stdio.h>; xtensa-gcc happens to pull it in
                         * transitively, arm-none-eabi-gcc does not. Provided
                         * here so ads1263.c can stay byte-identical upstream. */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
// Function prototypes (upstream contract -- called from ads1263.c)
//
//*****************************************************************************

void    InitADCPeripherals(void);
void    delay_ms(uint32_t delay_time_ms);
void    delay_ns(uint32_t delay_time_ns);
void    setCS(bool state);
void    setSTART(bool state);
void    setPWDN(bool state);
void    toggleRESET(void);
void    spiSendReceiveArrays(uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength);
uint8_t spiSendReceiveByte(uint8_t dataTx);
bool    waitForDRDYinterrupt(uint32_t timeout_ms);

// Functions used for testing only
bool    getCS(void);
bool    getPWDN(void);
bool    getRESET(void);
bool    getSTART(void);
void    setRESET(bool state);

//*****************************************************************************
//
// Macros
//
//*****************************************************************************

/* ads1263.c is compiled as C and does NOT include Arduino.h, so it needs these
 * from here. Guarded with #ifndef because hal.cpp *does* include Arduino.h,
 * which defines HIGH/LOW as 0x1/0x0 -- same truth values, so whichever
 * definition wins, the bool parameters of setCS()/setSTART()/setPWDN() see the
 * same state. */

/** Alias used for setting GPIOs pins to the logic "high" state */
#ifndef HIGH
#define HIGH                ((bool) true)
#endif

/** Alias used for setting GPIOs pins to the logic "low" state */
#ifndef LOW
#define LOW                 ((bool) false)
#endif

#ifdef __cplusplus
}
#endif

#endif /* TI_ADS126X_HAL_H_ */
