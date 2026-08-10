# Code Review Brief — Refactored Abstraction Layer

## Objective

Review the refactored smart-shunt-fw at `/Users/fab/dev/pv/smart-shunt-fw-stm32h5` (branch `port/stm32h5`) for correctness, regressions, and design issues. A large refactor just extracted shared logic into abstraction layers and split `main.cpp` into two platform files. Verify nothing broke.

## Context

The firmware targets two MCUs from one codebase:
- **ESP32-S3**: `main_esp32.cpp` — NimBLE BLE, WiFi/InfluxDB, OTA, dual-core tasks
- **STM32H5**: `main_stm32.cpp` — UART BLE bridge stub, single-core, STLINK console

Both share: `platform.h`, `ble_transport.h`, `sampler_registry.h`, `console.h`, `telemetry.h`.

## What changed in this refactor

1. `main.cpp` (1100-line `#ifdef` wall) → deleted, replaced by `main_esp32.cpp` (~100 lines) + `main_stm32.cpp` (~80 lines)
2. New `platform.h` — abstracts `micros64()`, `restart()`, `gpioHoldEn/Dis()`, `deepSleep()`, EEPROM access, task creation
3. New `ble_transport.h` — virtual `BleTransport` interface; `BleSrv` (ESP32) and `BleBridge` (STM32) implement it; `StubBleTransport` for testing
4. New `sampler_registry.h` — replaces `std::map<std::string, PowerSampler*>` (crashed on STM32) with a fixed vector-based registry
5. New `console.h` — shared `handleConsoleInput()` extracted from both main files
6. New `telemetry.h` — shared `Telemetry` class with telemetry output, aux notification, console polling, idle-sleep voting; `EspTelemetry` overrides `onIdleSleep()` for deep sleep
7. `ble.h` — `BleSrv` now extends `BleTransport`, methods marked `override`
8. `ble_bridge.h` — `BleBridge` now extends `BleTransport`, methods marked `override`
9. `energy_counter.h` — added `#pragma once` and `#include "settings.h"`
10. `platformio.ini` — `build_src_filter` added to both envs to select correct main file

## What to review

### 1. Behavioral equivalence — did the refactor change any behavior?

Compare the old `main.cpp` (available in git history, commit `35006d3`) with the new split files + abstractions. Check:

- **Telemetry output cadence**: Old code had `LastTimeOut` / `LastTimePrint` as file-scope globals. New code has them as `Telemetry` member fields. Is the timing identical?
- **Idle-sleep voting**: The `anyPowerVote` / `anyActive` logic and the `looksActive()` function — is the logic preserved exactly?
- **Console command handling**: The old ESP32 code had TWO console input paths (raw UART `uart_read_bytes` + `Serial.readString`). The new `telemetry.h` only has `Serial.readString()`. Was the UART path important?
- **OTA handling**: The old ESP32 `update()` called `otaBleTick()`, `markOtaValidIfHealthy()`, and checked `otaBleActive()` before sleep. The new `main_esp32.cpp` puts these in `appTask` around `telemetry->update()`. Is the ordering correct?
- **BLE flush**: Old code called `bleSrv.flush()` after the telemetry window. New code calls `ble.flush()` inside `Telemetry::update()`. Same thing?
- **Boot watchdog**: Old code armed/disarmed the boot watchdog in `setup()`. New code does the same. Correct?

### 2. `platform.h` — are the abstractions correct?

- `micros64()`: On STM32 it calls `esp_timer_get_time()` (the overflow-tracking `micros()` extension). On ESP32 it calls `esp_timer_get_time()` (native 64-bit timer). Both correct?
- `createRealTimeTask()`: ESP32 pins to core 1, STM32 uses `xTaskCreate`. Are the stack sizes and priorities consistent?
- `createAppTask()`: ESP32 pins to core 0, STM32 uses `xTaskCreate`. Correct?
- `eepromGet/Put/Read/Write`: ESP32 wraps with `begin()/commit()/end()`. STM32 calls directly. Are the EEPROM address layouts identical?
- `deepSleep()`: STM32 is a no-op. Is this safe?
- `gpioHoldEn/Dis()`: STM32 is a no-op. Is this safe given Stop mode retains GPIO?

### 3. `BleTransport` interface — is it complete?

- Does `BleSrv` implement all 6 virtual methods correctly with `override`?
- Does `BleBridge` implement all 6 virtual methods correctly with `override`?
- `StubBleTransport` — is it used in the right place? (STM32 `main_stm32.cpp` uses it)
- Are there any `BleSrv` methods that the ESP32 `main_esp32.cpp` calls directly (bypassing the interface) that should be in the interface? (e.g. OTA-related methods)

### 4. `SamplerRegistry` — is it a correct replacement for `std::map`?

- The old ESP32 code used `std::map<std::string, PowerSampler*>` with named entries like `"ESP32_INA228"`. The new code uses `SamplerRegistry::add("name", &sampler)`. Are the names identical?
- `EnergyCounter` is constructed with `e.name = samplers[i].name` (a `const char*`). The old code passed `p.first` (a `std::string`). Does `EnergyCounter::name` (a `std::string`) correctly construct from `const char*`?
- `findByName()` does `ec.name == name` where `name` is `const std::string&`. Is this correct for `const char*` arguments?

### 5. `Telemetry` class — is the inheritance safe?

- `EspTelemetry` overrides `onIdleSleep()` and calls `ESP.deepSleep()`. After `deepSleep()`, execution never returns (it resets on wake). Is the `virtual` call chain correct?
- `Telemetry` is heap-allocated (`new Telemetry(...)`). Is this safe on STM32 with 32KB SRAM? Should it be static?
- `Telemetry::update()` calls `processConsole()` and `checkIdleSleep()` — are these always safe to call, or do they need preconditions?

### 6. `console.h` — shared command handler

- The function takes `SamplerRegistry&` and `BleTransport&` by reference. Are these always valid when called?
- The old ESP32 code had `bootinfo` and `wifi on/off` commands that are now missing. Is this intentional?
- The `ina22x-resistor-range` command accesses `ina228_instance[devIdx]` and `ina228_mux_instance` — are these still declared and accessible from `console.h`?

### 7. Build system

- `build_src_filter` in `[env:stm32h5]`: excludes `main.cpp`, `main_esp32.cpp`, and ESP-only files. Does it also exclude `ble.h`? Should it exclude `console.h` (it includes `ina228.h` which includes `INA226_WE.h`)?
- `build_src_filter` in `[env:esp32s3]`: excludes `main.cpp`, `main_stm32.cpp`, `esp_compat.h`, `ble_bridge.h`, `ble_protocol.h`. Does it correctly include all the new shared headers (`platform.h`, `ble_transport.h`, `sampler_registry.h`, `console.h`, `telemetry.h`)?
- `secrets.h` was created as a stub in the worktree. Is this file gitignored? Will it conflict with the real `secrets.h` in the main repo?

### 8. `energy_counter.h` changes

- `#pragma once` added — was there any reason it was missing before? Could any code have relied on the double-include behavior?
- `#include "settings.h"` added — does this create a circular include? `settings.h` includes `EEPROM.h` and `esp_compat.h` (on STM32). `energy_counter.h` includes `util.h` which includes... what?

## Output format

For each issue found, classify as:
- **BUG**: Will cause incorrect behavior or crash at runtime
- **REGRESSION**: Behavior changed from the pre-refactor version
- **RISK**: May cause issues in specific scenarios
- **NIT**: Minor style or improvement

Read the files, compare with git history where needed, and provide a detailed review. Do NOT make any changes.
