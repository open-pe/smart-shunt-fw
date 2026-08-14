# ti_ads126x — vendored TI ADS1262/ADS1263 driver

Texas Instruments' official example driver for the ADS1262 / ADS1263 32-bit
delta-sigma ADC, plus an Arduino HAL so it builds on both targets of this repo
(`esp32s3`, `stm32h5`).

## Provenance

| | |
|---|---|
| Upstream | <https://github.com/TexasInstruments/precision-adc-examples> |
| Path | `devices/ads1262/` |
| Commit | `fb4a86a5eb1b96162414adc6a0c29ef950665e0f` (2026-08-07) |
| Vendored | 2026-08-13 |
| License | BSD-3-Clause (see `LICENSE`; per-file headers say the same) |

To refresh, re-download the four files from that path and re-apply the notes
under *Modifications* — `ads1263.c` / `ads1263.h` should drop in unchanged.

## Layout

```
src/ads1263.c     upstream, UNMODIFIED — the actual driver
src/ads1263.h     upstream, UNMODIFIED — register map, opcodes, bit masks
src/hal.h         ours — same prototype contract, MSP432 specifics removed
src/hal.cpp       ours — Arduino SPI/GPIO implementation of that contract
src/ti_ads126x.h  ours — the header applications should include
upstream/hal.c    TI's MSP432E401Y HAL, kept verbatim for reference
upstream/hal.h    TI's original hal.h, kept verbatim for reference
upstream/README.md TI's original readme (note: its body is copy-pasted ADS1258 text)
```

`upstream/` sits outside `src/`, so PlatformIO never compiles it. The library
as a whole is only built when something `#include`s it — nothing in `src/`
does today, so adding it does not change the current firmware builds.

## Modifications

Only the HAL was rewritten; the driver itself is byte-identical to upstream.

1. **`hal.h` replaced.** Upstream's includes `ti/devices/msp432e4/driverlib`
   (or, in its non-`EXAMPLE_CODE` branch, a bare `#include "settings.h"` that
   would have picked up *this repo's* `src/settings.h`) and defines MSP432 port
   and pin macros. Ours keeps the exact function prototypes `ads1263.c` calls,
   adds `extern "C"` guards, and defines `HIGH`/`LOW` only if Arduino has not
   already. It also includes `<stddef.h>`, which `ads1263.c` needs for its
   `NULL` checks but does not include — `xtensa-esp32s3-gcc` pulls it in
   transitively and `arm-none-eabi-gcc` does not, so without this the STM32H5
   build fails. Fixed here so `ads1263.c` can stay byte-identical to upstream.
2. **`hal.c` replaced by `hal.cpp`.** Arduino `SPI`/`digitalWrite` instead of
   driverlib. All timing constants (td(SCCS) 80 ns, /RESET 125 ns, START
   250 ns) are carried over from upstream unchanged.
3. **`ti_ads126x.h` added.** Upstream's headers have no `extern "C"` guards, so
   including `ads1263.h` directly from C++ gives its prototypes C++ linkage and
   they fail to link against the C objects built from `ads1263.c`. Include
   `ti_ads126x.h` instead.

## Usage

```cpp
#include <ti_ads126x.h>

Ads126xHalConfig cfg;
cfg.spi          = &SPI;
cfg.spiHz        = 8000000;   // 8 MHz is the part's max SCLK
cfg.pinCS        = 10;
cfg.pinSTART     = 9;         // -1 if strapped high on the board
cfg.pinResetPwdn = 8;         // -1 if strapped high
cfg.pinDRDY      = 7;         // -1 if not wired to the MCU

SPI.begin(sck, miso, mosi, cfg.pinCS);
if (!ads126xHalBegin(cfg)) { /* bad config — do not proceed */ }

adcStartupRoutine();          // upstream: settle, /PWDN high, reset, defaults

if (waitForDRDYinterrupt(100)) {
    uint8_t status, crc, data[4];
    int32_t raw = readData(&status, data, &crc);
}
```

The driver keeps a shadow copy of the register map; read it with
`getRegisterValue(REG_ADDR_MODE0)` and change registers through
`writeSingleRegister()` / `writeMultipleRegisters()` so the shadow stays in
sync.

## Gotchas

- **SPI mode 1.** The ADS1262/ADS1263 are CPOL=0 / CPHA=1. `hal.cpp` sets
  `SPI_MODE1`; if you share the bus with a mode-0 device, make sure every
  participant uses `beginTransaction()` (this HAL does — it opens the
  transaction on CS-low and closes it on CS-high).
- **`ADS1263_ONLY_FEATURES` is enabled by default.** `ads1263.h:52` defines it
  unconditionally, which sets `NUM_REGISTERS` to 27 and makes
  `adcStartupRoutine()` write the six ADC2 registers (`ADC2CFG`…`ADC2FSC1`).
  Those registers **do not exist on the ADS1262** — it is the single-ADC part.
  For a true ADS1262, comment out that `#define`; `NUM_REGISTERS` drops to 21.
  This is left as-is here so the file stays identical to upstream — decide it
  per board.
- **Single global instance.** `ads1263.c` holds one static `registerMap[]` and
  the HAL one static config, so this library drives exactly one ADC. Two parts
  on one bus need a different structure.
- **`waitForDRDYinterrupt()` polls.** Upstream waits on a GPIO interrupt; this
  implementation polls the /DRDY pin against a `millis()` deadline, so it
  blocks the calling task for up to `timeout_ms`. It returns `false` — never
  `true` — when /DRDY is not wired or the HAL is unconfigured.
- **`ads126xHalFaults()`.** Any HAL operation that could not actually reach the
  hardware increments this counter and returns `0xFF` / `false` rather than
  something plausible. A nonzero count after a read sequence means the values
  you got were fabricated; check it rather than trusting the data.
- **`getCS()` / `getSTART()` / `getPWDN()` / `getRESET()`** return the last
  *commanded* level, not a pin read (upstream reads the pin). For a signal
  configured as `-1` that is a wish, not a measurement —
  `ads126xHalPinConnected()` tells you which.

## Relationship to `src/adc/ads1262/`

`src/adc/ads1262.h` (`PowerSampler_ADS1262`) uses **this** library. The
**ProtoCentral** copy still sitting in `src/adc/ads1262/` is what it used
before; nothing includes it any more, so it is dead code and can be deleted.

The sampler deliberately does **not** call `adcStartupRoutine()` — see the
comment in its `init()`. Two upstream problems make it unsafe on an ADS1262:
it writes the register block sized by `NUM_REGISTERS` (27, because
`ADS1263_ONLY_FEATURES` is on), which runs past the last real ADS1262 register
`GPIODAT` (0x14) into the ADS1263-only ADC2 block; and it never assigns
`initRegisterMap[REG_ADDR_REFMUX]`, so it writes uninitialised stack memory to
the register that selects the voltage reference. The sampler writes each
register itself and reads them all back instead.

Note also that `writeMultipleRegisters(startAddress, count, regData)` indexes
`regData` by **absolute register address**, not from 0 — `regData` must be an
array of at least `startAddress + count` entries.
