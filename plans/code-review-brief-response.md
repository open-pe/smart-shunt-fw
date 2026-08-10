# Code Review Response — Refactored Abstraction Layer

The refactor is not behaviorally equivalent. I found one introduced OTA safety bug, three clear ESP32 regressions, and a pre-existing STM32 runtime blocker. Both intended environments compile and link successfully; no source files were changed during the review.

## Findings

### 1. BUG — OTA can reach the sleep decision before the active-OTA guard

The new app loop calls `telemetry->update()` before recording that OTA is active (`src/main_esp32.cpp:206`). But `Telemetry::update()` performs `checkIdleSleep()` internally, which may call `ESP.deepSleep()` before control returns (`src/telemetry.h:77`).

In `35006d3`, `otaBleActive()` called `noteWakeEvent()` at `main.cpp:857-862`, before the sleep decision at lines 991-1006. The new ordering defeats that explicit safety guard. If OTA remains active after a disconnect while the idle timer has expired, the device can sleep before the guard executes.

### 2. REGRESSION — ESP32 WiFi/Influx telemetry output was removed

`Telemetry::update()` now sends each summary only through BLE (`src/telemetry.h:48`). The old code also appended summaries to `wire_sample_buf` and called `influxWritePointUDP()` when WiFi was enabled (`35006d3:src/main.cpp:899,941-945`).

The new ESP main still initializes WiFi and configures an `InfluxDBClient` (`src/main_esp32.cpp:147`), but no telemetry path consumes that configuration. Additionally, `wifi on/off` commands disappeared. WiFi is therefore reduced to optional boot-time time synchronization; the former runtime telemetry feature no longer works.

### 3. REGRESSION — the deployed ESP32 UART console path was removed

The old code explicitly documented that `Serial.available()` did not work under PlatformIO, then read UART0 using `uart_get_buffered_data_len()` and `uart_read_bytes()`. The new code still installs the raw UART driver but polls only `Serial.available()` and `Serial.readString()` (`src/telemetry.h:82`, `src/main_esp32.cpp:217`).

This likely makes the console unreachable on the hardware/configuration for which the raw path was added. Even if `Serial.available()` now works, the new local `String r` discards a trailing partial line on every call; the old ESP path retained partial input in a static `serialBuf`.

### 4. BUG — STM32 never starts the FreeRTOS scheduler (pre-existing)

STM32 setup creates two tasks and returns, but never calls `vTaskStartScheduler()` (`src/main_stm32.cpp:63`). The subsequent Arduino `loop()` calls `vTaskDelay()` even though no scheduler was started.

STM32duino FreeRTOS requires an explicit scheduler start; its bundled examples all do so. Consequently, the sampling and telemetry tasks will not run correctly. This was already present in commit `35006d3`, so it is not caused by this refactor.

### 5. RISK — STM32 task allocation can exhaust SRAM, and failures are ignored

`xTaskCreate(..., 2048, ...)` uses 2,048 `StackType_t` words on STM32, or 8 KiB per task. Two tasks therefore request 16 KiB (`src/main_stm32.cpp:66`). The STM32 build already uses 11,360 of 32,768 bytes before runtime allocations.

In the current ARM binary, each `EnergyCounter` is 448 bytes. With five counters, the vector's capacity-eight allocation is 3,584 bytes, and the five queues require at least another 2,560 bytes for sample payloads, excluding their metadata. That does not fit alongside the two task stacks, TCBs, mutexes, registry entries, and other heap allocations.

The wrappers discard the `xTaskCreate()` result (`src/platform.h:95`), so a failed app or sampling task is silent. The heap-allocated `Telemetry` object itself is only tens of bytes and is not the main problem; making it static would help slightly but would not solve the stack/counter budget.

### 6. REGRESSION — OTA safety observability was stripped

The shared console no longer has `bootinfo`, formerly described as the only way to observe a rollback (`src/console.h:94`). The new boot-watchdog creation path also returns silently on failure, whereas the old version emitted a high-severity warning. Likewise, failure to mark an OTA image valid is no longer logged.

The watchdog is still armed and disarmed at the correct points, but failure of the safety mechanism is now considerably harder to detect.

### 7. RISK — `publishAux(bool)` has incompatible meanings across transports

The interface describes its argument as the desired aux state (`src/ble_transport.h:6`), and `BleBridge` sends that state. `BleSrv`, however, interprets the same argument as a `notify` flag and obtains the state from global `auxGet()` (`src/ble.h:181`).

Current calls happen to pass `auxGet()`, masking the mismatch. It can also produce duplicate "ON" notifications because `BleSrv::tick()` already publishes state changes before the generic telemetry layer publishes again. The abstraction cannot safely be used with a caller-supplied state as written.

### 8. RISK — the retained `esp32` environment compiles both platform mains

Source selection was added for `esp32s3` and `stm32h5`, but not for the older `[env:esp32]` environment (`platformio.ini:11`). A build confirms it attempts to compile both `main_esp32.cpp` and `main_stm32.cpp`.

That environment currently has additional pre-existing dependency/configuration failures, but the split introduced another deterministic failure once those are fixed. It needs the same main-file filtering if it remains supported.

### 9. RISK — `telemetry.h` is not self-contained

`Telemetry` calls `handleConsoleInput()` but does not include or declare it (`src/telemetry.h:3`). Both mains happen to include `console.h` first, so current builds pass. Any test or translation unit including `telemetry.h` directly will fail depending on include order.

## Checks that passed

- Telemetry cadence, rollover-safe subtraction, idle vote, `looksActive()`, and BLE flush placement are otherwise preserved.
- ESP32 and STM32 sampler names and ordering match their old implementations.
- `const char*` correctly constructs `EnergyCounter::name`; `findByName()` comparisons are valid.
- Active NimBLE `BleSrv`, `BleBridge`, and `StubBleTransport` implement all six virtual methods.
- The `EspTelemetry` virtual sleep dispatch is valid, and referenced registry/transport objects have static lifetime.
- `micros64()` is correct for both current targets, provided the STM32 overflow extension continues to be called more often than once per 32-bit wrap.
- EEPROM byte layouts remain identical. The new platform EEPROM wrappers are currently unused.
- STM32 no-op sleep/GPIO-hold functions are harmless in the current path because Stop mode is not implemented and those functions are not called; they are not safe general-purpose implementations of those contracts.
- `energy_counter.h` now has proper include protection and no circular include.
- Both intended source filters work. `console.h` compiles on STM32 because `ina228.h` guards `INA226_WE.h`.
- `src/secrets.h` is ignored and untracked, so it will not conflict through Git.

## Verification

- `pio run -e stm32h5`: success; RAM 34.7%, flash 57.0%.
- `pio run -e esp32s3 -t checkprogsize`: success; RAM 18.5%, flash 78.2%.
- `pio run -e esp32 -t checkprogsize`: failed and confirmed both new mains are selected.
- `git diff --check 35006d3..HEAD`: clean.
- Worktree remained clean until this requested response file was added.
