# Architecture Investigation Brief

## Objective

Investigate the smart-shunt-fw codebase (worktree: `/Users/fab/dev/pv/smart-shunt-fw-stm32h5`, branch `port/stm32h5`) for opportunities to improve readability, reduce file sizes, and bundle semantically connected data and methods into cohesive classes. This is an organically grown project — expect accumulated cruft, duplicated logic, and missed abstraction boundaries.

## Context

The firmware measures DC power/energy via TI INA228 I2C ADCs, integrates it over time, and publishes it over BLE (ESP32) or UART-to-BLE-bridge (STM32H5). It was originally written for ESP8266, ported to ESP32, then ESP32-S3, and now STM32H5. A recent refactor introduced `platform.h`, `ble_transport.h`, `sampler_registry.h`, `console.h`, and `telemetry.h` abstractions, and split `main.cpp` into `main_esp32.cpp` + `main_stm32.cpp`.

## Scope

Investigate ALL source files under `src/`, `ble-bridge/src/`, and the build configs. Do NOT make changes — produce findings only.

## What to look for

### 1. Abstraction candidates — data/methods that belong together but are scattered

- **`settings.h`**: Pin mappings, EEPROM calibration storage, and `checkCalibrationFactorBounds` are all in one file but serve three different concerns. Should pin config, calibration storage, and validation be separate types?
- **`i2c.h`**: The `I2cLock` class and the free functions `i2c_write_buf`, `i2c_read_buf`, etc. — could these be a class with the lock as a member? The `esp_err_t` return type is an ESP-IDF concept that leaks into MCU-independent code.
- **`aux_switch.h`**: State, persistence, drive, parse, and deep-sleep hold are all free functions sharing a file-level `static bool auxState`. Should this be a class?
- **`util.h` / `util.cpp`**: Contains `timeStr()`, `scan_i2c()`, `UART_LOG()`, `getTimeStamp()`, WiFi/InfluxDB functions, and the `TaskNotification` class — five unrelated concerns in one file. What should be split out?
- **`energy_counter.h`**: The `WireSample` struct (with `getInfluxDbPoint()` and `compute_crc16`) and the `EnergyCounter` class are coupled. Should `WireSample` be its own type? The `UIP<T>` template — is it used anywhere else?
- **`sampling.h`**: `Sample` struct, `PowerSampler` ABC, `getTimeStamp()` declaration, and the dead `getTimeStamp` implementation comment. Should `Sample` have methods (e.g. `p()`, `setTimeNow()`) moved out of the struct into a class?

### 2. File size — files that are too large or do too much

- List every `.h`/`.cpp` file with its line count
- Identify files > 200 lines that could be split
- Identify headers that define multiple unrelated classes/structs

### 3. Duplicated logic across the two targets

- Compare `main_esp32.cpp` and `main_stm32.cpp` — what's genuinely platform-specific vs what was duplicated because it was easier?
- Compare `ble.h` (ESP32 NimBLE) and `ble_bridge.h` (STM32 UART) — both implement `BleTransport`, but do they share any logic that could be factored out?
- The `ble-bridge/src/main.cpp` (ESP32 relay) reimplements the UART frame parser from `ble_bridge.h` — should `ble_protocol.h` provide a shared `FrameParser` class?

### 4. Dead code and accumulated cruft

- Commented-out code blocks (there are many — `readBuf2`, old ADS131 pin configs, `PointDefaultConstructor`, etc.)
- `#ifdef` guards that are always-true or always-false on current targets
- Unused includes (e.g. `#include <SPI.h>` in `ina228.h` on ESP32)
- Files excluded by `build_src_filter` but still present (`adc_esp.h`, `adc_esp_dma.h`, `ads1220.h`, etc.) — are they kept for reference or safe to remove?
- The `PowerSampler_Dummy` class — is it used?
- The `sui.h` empty stub
- The `lcd.h` / `WITH_LCD` code path

### 5. Header inclusion hygiene

- Which headers include `<Arduino.h>` when they only need specific types?
- Which headers pull in platform-specific includes (`esp_log.h`, `driver/gpio.h`, `stm32h5xx_hal.h`) that should be behind `platform.h` instead?
- Circular include risks — `energy_counter.h` includes `settings.h` includes `EEPROM.h`; `sampling.h` includes `esp_compat.h` on STM32 but `sys/time.h` on ESP32
- Missing `#pragma once` guards (the `energy_counter.h` bug that was just fixed — are there others?)

### 6. Class design opportunities

- Should `PowerSampler` have a `stopReading()` or `deinit()` method? The `startReading()` is used for stall recovery but there's no symmetric cleanup.
- Should `EnergyCounter` own its `PowerSampler*` instead of holding a raw pointer? Should it be constructed with the sampler name as `std::string_view` instead of copying a `std::string`?
- Should `SamplerRegistry` be a stack-allocated singleton rather than passed by reference everywhere?
- `Telemetry` has a `virtual onIdleSleep()` overridden by `EspTelemetry` — is there a cleaner way to express this (strategy pattern, function pointer, callback)?

## Output format

Produce a markdown report with these sections:

1. **File inventory** — table of every source file, line count, and what it does
2. **Abstraction opportunities** — ranked by impact (high/medium/low)
3. **Duplicated logic** — table comparing the two targets, what's shared vs duplicated
4. **Dead code** — list of files, functions, and code blocks safe to remove
5. **Include hygiene** — list of files with unnecessary or platform-leaking includes
6. **Class design recommendations** — concrete proposals with rationale
7. **Suggested file reorganization** — proposed directory structure if things were cleaned up
