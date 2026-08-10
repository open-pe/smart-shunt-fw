# Architecture investigation response

Scope: branch `port/stm32h5`, all `.h`/`.cpp` files under `src/` and `ble-bridge/src/`, plus the PlatformIO, board, and partition configuration. This is a findings-only review; no firmware source was changed.

Validation snapshot:

- `pio run -e stm32h5` succeeds: 74,664 bytes flash, 11,360 bytes RAM.
- `pio run -e esp32s3 -t buildprog` succeeds: 1,384,550 bytes flash, 60,772 bytes RAM.
- `pio run -e esp32 -t buildprog` fails. The non-S3 settings branch declares `settings_t_01` but instantiates `settings_t`, NimBLE is not a dependency of this environment, and both target mains are selected.
- `pio run -d ble-bridge -e esp32_ble_bridge -t buildprog` fails because `ble_protocol.h` is outside that project's include path. If the path is fixed as-is, `ble_crc16_update` is then defined both by the header and by `ble-bridge/src/main.cpp`.
- More importantly, the successful STM32 build constructs `StubBleTransport`; `BleBridge` is not included or instantiated anywhere. The current STM32 image therefore has no UART telemetry/aux link despite the intended architecture.

## 1. File inventory

There are 44 `.h`/`.cpp` files totaling 10,005 physical lines (`wc -l`). “Active” means reachable from a successful current build, not merely selected by a source filter.

### Application, services, and support

| File | Lines | Status | Responsibility |
|---|---:|---|---|
| `src/atomicops.h` | 777 | Both, vendored | Moodycamel low-level atomics/semaphore support used by `ReaderWriterQueue`. |
| `src/aux_switch.h` | 123 | Both | Aux GPIO state, EEPROM persistence, command parsing, logging, and ESP deep-sleep pad hold. |
| `src/ble.h` | 695 | ESP32-S3 | NimBLE GATT server, telemetry indication buffering/ack handling, OTA transport, connection tuning, aux characteristic, and callbacks. Also contains a dead alternate BLE implementation. |
| `src/ble_bridge.h` | 171 | Intended STM32; currently unused | `BleTransport` over framed UART, inbound aux/link-status dispatch, and frame parser/writer. |
| `src/ble_protocol.h` | 53 | Intended shared; currently unused by successful builds | UART message IDs, an unused frame header, and Modbus CRC helpers. |
| `src/ble_transport.h` | 25 | Both | Telemetry transport interface and no-op stub. |
| `src/console.h` | 104 | Both | Parses reset, calibration, INA range, aux, and help commands; directly reaches driver globals. |
| `src/energy_counter.h` | 377 | Both | Sample queue, calibration, power/charge integration, stall recovery, windows, logging, wire summary, Influx conversion, and CRC. |
| `src/esp_compat.h` | 85 | STM32 | ESP-style errors/logging/time shims over Arduino STM32, FreeRTOS, and STM32 HAL. |
| `src/i2c.h` | 113 | Both | Global recursive bus lock and register read/write/probe helpers over `Wire`. |
| `src/lcd.h` | 45 | Dormant ESP32 | SSD1306 wrapper; the `WITH_LCD` path is not configured and no code calls `init()` or `updateValues()`. |
| `src/main_esp32.cpp` | 238 | ESP32-S3 | Object graph, setup/tasks, OTA boot validation/watchdog, Wi-Fi/Influx setup, deep sleep, and raw UART setup. |
| `src/main_stm32.cpp` | 86 | STM32 | Object graph, setup, and two tasks; currently selects the stub transport. |
| `src/mean_window.h` | 36 | Both | Running mean/max accumulator. |
| `src/platform.h` | 111 | Both | Time, restart, GPIO hold, sleep, EEPROM, and task-creation wrappers; only time and task creation are used. |
| `src/readerwriterqueue.h` | 982 | Both, vendored | Moodycamel single-producer/single-consumer queue; includes a large commented-out blocking queue. |
| `src/sampler_registry.h` | 68 | Both | Registers samplers, creates counters, and fans out lifecycle/update/reset operations. |
| `src/secrets.h` | 1 | Local ESP32 input | Generated/ignored Wi-Fi AP-list macro. |
| `src/settings.h` | 167 | Both | Board pins, calibration validation, and target-specific EEPROM storage. |
| `src/sui.h` | 2 | Unused | Empty “Serial User Interface” placeholder. |
| `src/telemetry.h` | 115 | Both | Periodic summary publication, console polling, aux publication, activity voting, and idle-sleep dispatch. |
| `src/util.cpp` | 227 | Both | Time formatting, timestamping, I2C scan, logging, Wi-Fi, UDP Influx batching, and dead Point helpers. |
| `src/util.h` | 78 | Both | Declarations for those utilities plus the unrelated `TaskNotification` synchronization class. |
| `ble-bridge/src/main.cpp` | 238 | Relay, build broken | Standalone ESP32 NimBLE relay, UART frame parser/writer, GATT setup, and aux/link callbacks. |

### ADC and sampler code

| File | Lines | Status | Responsibility |
|---|---:|---|---|
| `src/adc/sampling.h` | 80 | Both | Packed `Sample`, `PowerSampler` interface, timestamp declaration, and an obsolete commented timestamp implementation. |
| `src/adc/ina228.h` | 656 | Both | Direct INA228 register I/O, configuration/calibration, ISR routing, polling, conversion, diagnostics, and `PowerSampler` adapter. |
| `src/adc/ina228_mux.h` | 393 | Both | A second INA228 implementation specialized for mux sequencing, zero calibration, diagnostics, plus per-channel sampler adapters. |
| `src/adc/tmp117.h` | 154 | Both | TMP117 sampler interface and the unrelated dummy sampler. |
| `src/adc/tmp117.cpp` | 131 | Both | TMP117 initialization, DRDY polling, checked reads, and error throttling. |
| `src/adc/adc_ads.h` | 199 | Dormant but linked on ESP32-S3 | ADS1115 voltage/current sampler. A global instance exists but is never registered. |
| `src/adc/adc_esp.h` | 55 | Dormant | Legacy ESP ADC one-shot sampler; included by the ESP main but never instantiated. |
| `src/adc/adc_esp_dma.h` | 70 | Unused | Incomplete continuous/DMA experiment that still performs one-shot reads. |
| `src/adc/ads1220.h` | 100 | Dormant | ADS1220 sampler experiment; included but never instantiated. |
| `src/adc/ads1262.h` | 119 | Dormant | ADS1262 `PowerSampler` wrapper; included but never instantiated. |
| `src/adc/ads1262/ads1262.h` | 98 | Dormant third-party-derived code | ProtoCentral-style ADS1262 register driver declaration. |
| `src/adc/ads1262/ads1262.cpp` | 207 | Compiled on ESP32-S3, linker-GC candidate | ProtoCentral-style ADS1262 SPI register implementation. |
| `src/adc/ads131.h` | 188 | Dormant but linked on ESP32-S3 | ADS131 sampler and external clock programming. A global instance exists but is never registered. |
| `src/adc/ads131/ADS131M02.h` | 270 | Unused legacy variant | Two-channel ADS131 register map and driver declaration. |
| `src/adc/ads131/ADS131M02.cpp` | 414 | Compiled on ESP32-S3, linker-GC candidate | Two-channel ADS131 implementation. |
| `src/adc/ads131/ADS131M04.h` | 270 | Unused legacy variant | Four-channel ADS131 register map and driver declaration. |
| `src/adc/ads131/ADS131M04.cpp` | 507 | Compiled on ESP32-S3, linker-GC candidate | Four-channel ADS131 implementation. |
| `src/adc/ads131/ADS131M0x.h` | 302 | Dormant but linked on ESP32-S3 | Combined ADS131M02/M04 register map and driver declaration. |
| `src/adc/ads131/ADS131M0x.cpp` | 721 | Dormant but linked on ESP32-S3 | Combined ADS131 implementation used by the unregistered global ADS131 sampler. |
| `src/adc/ina226.h` | 154 | Dormant but linked on ESP32-S3 | INA226 sampler. A global instance exists but is never registered. |

### Build configuration

| File | Lines | Responsibility and finding |
|---|---:|---|
| `platformio.ini` | 138 | Defines legacy ESP32, ESP32-S3, and STM32H5 environments. Filters are long, header-oriented, and contain stale entries such as absent `main.cpp`; the legacy ESP32 environment is broken. |
| `ble-bridge/platformio.ini` | 24 | Relay environment. It has no `lib_extra_dirs`/include path for the shared protocol and emits repeated core-macro redefinition warnings. |
| `boards/nucleo_h503rb.json` | 38 | Local STM32H503 board definition. |
| `parts.csv` | 17 | ESP32-S3 OTA/NVS/SPIFFS/coredump partition map; SPIFFS is intentionally reserve space and is not mounted. |

### Files over 200 lines

| File(s) | Why large | Recommendation |
|---|---|---|
| `src/ble.h` (695) | GATT server, reliable telemetry stream, OTA, aux, link tuning, and callbacks | Split by service/characteristic and transport concern. |
| `src/energy_counter.h` (377) | Integration, recovery, queues, windows, serialization, CRC, persistence, and printing | Split domain state from transport encoding and presentation. |
| `src/adc/ina228.h` (656), `ina228_mux.h` (393) | Duplicate register codec/device logic plus two scheduling strategies | Extract an INA228 device/codec, then keep direct and mux sampler policies small. |
| `src/main_esp32.cpp` (238) | Shared boot plus ESP-only OTA/Wi-Fi/UART/deep-sleep concerns | Introduce a common `FirmwareApp`; retain thin platform composition roots. |
| `src/util.cpp` (227) | Five unrelated utility families | Split clock, log, I2C diagnostics, Wi-Fi, and Influx modules. |
| `ble-bridge/src/main.cpp` (238) | Protocol, UART, BLE service, and Arduino entry points | Reuse shared protocol classes and extract a relay GATT object. |
| `src/adc/ads1262/ads1262.cpp` (207) | Legacy third-party-derived driver | Archive/delete if unsupported; otherwise place in its own library, not production `src`. |
| Six `src/adc/ads131/*` files (270–721) | Three overlapping driver variants with copied register maps/implementations | Keep one supported variant or archive the family; consolidation matters more than mechanically splitting it. |
| `src/atomicops.h` (777), `readerwriterqueue.h` (982) | Vendored library, including unused platform support/commented code | Do not refactor locally; move to `lib/` or a pinned dependency with license/version metadata. |

## 2. Abstraction opportunities

### High impact

| Rank | Opportunity | Evidence | Concrete direction |
|---:|---|---|---|
| 1 | Make the UART protocol a real shared module | The two 8-state parsers and frame writers are nearly line-for-line copies. The relay cannot include the current header, and the STM image uses a stub. | Create a small platform-free library containing `MessageType`, `Crc16Modbus`, `FrameWriter`, and incremental `FrameParser`. Give it callbacks/spans, explicit big-endian length encoding, and tests for resync, zero-length, oversize, CRC failure, and fragmented input. Reference it from both PlatformIO projects. |
| 2 | Introduce a common firmware composition root | Both mains instantiate the same INA228, mux channels, TMP117s, registry, telemetry, and task loops. Most differences are boot services and platform policies. | Add `FirmwareApp`/`AppContext` that owns the registry, sensors, transport, and telemetry and implements common setup/task bodies. Inject `Board`, `Transport`, `IdlePolicy`, and optional OTA hooks; leave each `main_*.cpp` as wiring. |
| 3 | Separate measurement domain state from wire/presentation code | `EnergyCounter` owns integration, calibration, queueing, stall recovery, windows, printing, `Point`, packed wire layout, and CRC. `summary()` currently forgets to put `Energy` into `ws.data.e`. | Split into `SampleProcessor`/`EnergyIntegrator`, `SamplerMonitor`, `SummaryWindow`, and a versioned `TelemetryEncoder`. Put Influx conversion in an ESP-only adapter. This also gives reset and summary semantics one owner each. |
| 4 | Extract an INA228 device abstraction on top of a portable I2C bus | Direct and mux drivers duplicate registers, reset/ID checks, shunt calibration, temperature/voltage conversion, diagnostics, and alert handling. `ina228_mux.h` explicitly duplicates constants to avoid ODR problems in `ina228.h`. | Build `I2cBus` plus `Ina228Device` (`probe`, `reset`, `configure`, `readStatus`, `readVbus`, `readTemp`, `setShunt`). Keep `ContinuousIna228Sampler` and `MuxedIna228Sampler` as scheduling policies. |
| 5 | Replace address arithmetic with a persistence schema | Pins, EEPROM access, validation, energy calibration, resistor/range data, and aux state share raw offsets. Comments already document slot collisions and reserved bytes. | Create `BoardPins` as `inline constexpr` data and a `SettingsStore`/`CalibrationStore` with typed keys, schema version/magic, bounds checking, and target-specific backend. One backend should own `EEPROM.begin/commit/end`. |

### Medium impact

| Rank | Opportunity | Evidence | Concrete direction |
|---:|---|---|---|
| 6 | Make aux switch state cohesive | File-level `static auxState` plus free persistence, GPIO, sleep-hold, and parser functions. Header-local state can produce a separate instance per translation unit. | `AuxSwitch` should own state and receive `DigitalOutput`, `ByteStore`, and `SleepHold` collaborators. Keep a platform-free `parseAuxCommand` beside the shared protocol so the relay and firmware use identical semantics. |
| 7 | Decompose the ESP BLE server | `BleSrv` handles GATT construction, acked telemetry buffering, session state, OTA, aux requests, security, connection parameters, and all callbacks. | Compose `SmartShuntGatt`, `IndicationQueue`, `OtaGattService`, and `AuxGattService`. The direct ESP firmware and relay can share the base GATT/UUID layer while retaining different telemetry sources. |
| 8 | Replace the “utility drawer” with focused services | `util.*` mixes clocks, logs, I2C scan, task notification, Wi-Fi, UDP Influx, and Point adapters. | Move to `clock`, `log`, `i2c_scanner`, `rtos_notification`, and ESP-only `wifi`/`influx` modules. Keep platform code out of domain headers. |
| 9 | Make console commands operate on interfaces | `console.h` includes both full INA228 headers and reaches `ina228_instance[]`/`ina228_mux_instance` globals. | Register commands against `CalibrationService`, `AuxSwitch`, and a typed device-control registry. This removes concrete drivers from the console parser. |
| 10 | Treat active sensor selection as configuration | ESP constructs three unused legacy samplers while only five current devices are registered; filters try to select code by exclusion. | Declare the supported sensor set once in the composition root or generated board config. Compile experimental drivers as separate libraries/environments. |

### Low impact

- Keep `Sample::p()` as a pure derived accessor if convenient, but move `setTimeNow()` out. A packed transport-facing value should not pull `gettimeofday`, ESP compatibility, and HAL headers into every sampler.
- Replace generic `UIP<T>` with a domain-named `MeasurementWindows`/`ChannelStats`; `UIP` is used only twice, both inside `EnergyCounter`.
- Move implementation-heavy headers (`console`, `telemetry`, `energy_counter`, INA drivers) into `.cpp` files. This reduces transitive includes, ODR risk, and rebuild time without changing runtime architecture.
- Rename `BleTransport` to `TelemetryTransport` if UART remains an implementation; the current name describes one backend rather than the interface.

## 3. Duplicated logic

### ESP32-S3 versus STM32 main

| Area | Shared/duplicated | Genuinely platform-specific | Recommended home |
|---|---|---|---|
| Sensor object graph | Same INA228 at `0x40`, same mux backend at `0x41`, same two mux channels, same two TMP117s | ESP prefixes published names with `ESP32_`; pin data differs through settings | `FirmwareApp` + `BoardConfig` |
| Startup sequence | Serial, aux restore, I2C, transport begin, ABI size assertions, add/init/start samplers, create telemetry, start two tasks | ESP boot watchdog, Wi-Fi/time sync, OTA setup; different `Wire.begin` overload | Common `FirmwareApp::start`, platform hooks around it |
| Registry population | Same five `add()` calls and mux `init()` | None apart from names | Static sensor descriptor table or common method |
| Real-time task | Infinite `samplers.updateAll()` loop | STM adds `vTaskDelay(1)`; ESP honors OTA `g_samplingHalted` and pins a core | Common loop with injected yield/quiesce policy |
| App task | Infinite `telemetry->update()` plus 10-tick delay | ESP runs OTA tick/liveness confirmation | Common loop with optional app hooks |
| Idle behavior | Same activity detection in `Telemetry` | ESP deep-sleeps and holds aux; STM only logs | Injected `IdlePolicy` |
| Task creation | Same two logical tasks/priorities | ESP core affinity and larger stacks; STM generic FreeRTOS tasks | `TaskSpawner`/platform adapter |
| Transport | Both satisfy `BleTransport` in type | ESP uses NimBLE; STM currently uses no-op stub, not the intended UART bridge | Platform composition root; fix STM selection first |

### BLE and UART transport

| Concern | `BleSrv` | `BleBridge` | Factor? |
|---|---|---|---|
| Telemetry delivery | Buffered NimBLE indications with acknowledgements/backpressure | Encodes one UART frame | Keep backend-specific behind `TelemetryTransport`. |
| Flush/tick | Retires indications, handles timeout, OTA, aux, link tuning | UART flush and byte parser | Backend-specific. |
| Connection state | NimBLE server/subscription state | Link-status frame from relay | Shared interface semantic, different implementation. |
| Aux state | Creates characteristic, parses requests, defers NVS write | Parses AUX_SET and publishes AUX_STATE frames | Share pure command/value types and parser; keep I/O separate. |
| GATT identifiers/setup | Defines service/telemetry/aux UUIDs and callbacks | Relay duplicates those identifiers and much of basic server setup | Extract `GattIds` and possibly a reusable `SmartShuntGatt` component. |
| OTA | Full OTA service and link tuning | None | ESP-direct only unless the relay protocol later tunnels OTA. |

### UART frame protocol duplication

`BleBridge::tick()` and `processUartRx()` in the relay have the same state enum, fields, byte transitions, oversize handling, CRC calculation, and resynchronization. `BleBridge::sendFrame()` and `sendFrameToSTM32()` likewise duplicate framing. Both use fixed 255-byte payloads and a big-endian length/CRC, while the unused `BleFrameHeader` exposes a native-endian `uint16_t` and should not be treated as an on-wire struct.

The shared API should look conceptually like:

- `FrameParser<MaxPayload>::push(byte)` returning either “need more”, a validated frame view, or an error;
- `encodeFrame(type, payload, outputSpan)` returning encoded length/error;
- one CRC implementation shared with `WireSample` if the protocol intentionally continues using the same Modbus polynomial;
- application callbacks mapping frames to aux/link/telemetry behavior.

This is the strongest duplication removal because it also fixes two independently evolving protocol implementations.

## 4. Dead code

### Safe to remove now (no call sites in this repository)

| Item | Evidence/notes |
|---|---|
| `PowerSampler_Dummy` in `tmp117.h` | No instantiation or reference outside its declaration. Put a dummy in tests if still useful. |
| `readBuf2()` and `PowerSampler_INA228::read_voltage_current()` | The helper unconditionally returns `false`; its remaining body is unreachable. The only caller is the otherwise-unused combined read method. |
| `PointDefaultConstructor` and `pointFromSample()` in `util.cpp` | No callers. |
| `influxWritePointsUDP()` / `influxWritePointUDP()` / their buffer helpers | No production caller. The plural function also mistakenly sends `p[0]` on every iteration. |
| `BleFrameHeader` and `ble_crc16()` | Neither is referenced. Keep only the incremental CRC used by an actual codec. |
| `platform::{restart,gpioHoldEn,gpioHoldDis,deepSleep,eepromGet,eepromPut,eepromRead,eepromWrite}` | No call sites; ESP code bypasses them with direct APIs and persistence code bypasses the EEPROM wrappers. Either adopt them through focused interfaces or delete them. |
| `esp_compat.h` EEPROM macros, OTA placeholder structs, and `ESP_restart` macro | No call sites. |
| `EnergyCounter::maxDtReported`, `startTime`, and `numTimeouts` | Written but only read in commented-out code. `TotalCharge`/`LastI` are computed but never published or queried; remove them unless charge is about to become a first-class output. |
| `EnergyCounter::summary(dt_us, ..., outNewSamples)` extra plumbing | `dt_us` is unused. The caller invokes `summary` only after `newSamplesSinceLastSummary`, so the output flag is redundant. |
| Duplicate `getTimeStamp` declaration and commented implementation in `sampling.h` | One declaration in a clock header is sufficient. |
| `adc_ads.h::chooseGain`, legacy ADC `readADC_Cal` helpers, unused pin constants and buffer counters | No live call sites; several have only commented references. |
| `sui.h` | Empty, unreferenced, and explicitly filtered from STM32. |

### Remove from active builds immediately

`main_esp32.cpp` constructs `PowerSampler_ADS ads`, `PowerSampler_ADS131 ads131`, and `PowerSampler_INA226 ina226`, but never adds them to `SamplerRegistry`. This is not cost-free dead code: the final ESP32-S3 ELF contains all three objects/vtables and substantial ADS131/ADS/INA226 methods. The objects reserve 120, 148, and 92 bytes respectively, plus a 4-byte instance pointer for each, and the linked methods include multi-hundred-byte initialization paths.

Remove those globals and their includes from the production composition root. Then remove the unused Adafruit ADS1X15, INA226_WE, and ADS1220 dependencies from the active ESP32-S3 environment if nothing else needs them.

The following driver families have no registered sampler in either current image:

- `adc_esp.h`, `adc_esp_dma.h`;
- `adc_ads.h`;
- `ads1220.h`;
- `ads1262.h` and `ads1262/`;
- `ads131.h` and all three `ads131/ADS131M*` variants;
- `ina226.h`.

Deleting them is safe for the two currently registered hardware sets, but this is a product-support decision: if they are retained as bench references, move them to `experimental/` or separate PlatformIO library/environments so they do not compile/link in production. Git history is a better archive than commented production code.

### Dormant feature branches to decide explicitly

- `USE_ARDUINO_BLE` is never defined. Its old BLE branch is dead and already causes LDF trouble documented in `platformio.ini`; remove it.
- `WITH_LCD` is never defined in any environment. Even when defined, the global `LCD` is not initialized or updated. Delete the file/dependencies or create a tested LCD environment.
- The ESP8266 branch in `util.cpp` has no PlatformIO environment.
- `XIAO_ESP32S3` is hard-coded true while WaveShare/green-board branches are false; `FMETAL` is not configured. Move real board variants to build configuration or delete them.
- `APPLY_U_GAIN_CAL` is hard-coded zero, making its application branch dead. Replace the macro with one deliberate behavior or a real build option.
- Wi-Fi/Influx is runtime-disabled by globals initialized to `true`, and no current path changes `disableWifi` to false. It still adds dependencies and a global client. Decide whether it is supported; do not leave it as source-edit configuration.
- UART message IDs `CALIB_SET`, `RESET_CMD`, `RESISTOR_RANGE`, `BOOTINFO`, and `CONSOLE_LOG` are declared but have no sender/handler. Mark them reserved with protocol documentation or remove them until implemented.

`ble_bridge.h` is also unreachable, but it is not a removal candidate: it represents required STM32 functionality. The right action is to instantiate it and make the relay build, not to classify it as harmless dead code.

### Commented-out blocks to delete during cleanup

- The alternate timestamp implementation in `sampling.h`.
- The old `setVal` implementation and advertising experiments in `ble.h`; remove the entire alternate BLE branch as above.
- Disabled fields/Influx metrics and serial-print block in `energy_counter.h`.
- `readBuf2`'s unreachable body and the large unreachable old combined-read implementation in `ina228.h`; remove scattered commented individual reads and bus-power comparison too.
- Driver experiments throughout `adc_ads.h`, `ads1220.h`, `ads1262.h/.cpp`, `ads131.h`, `ADS131M02.*`, and `ADS131M04.*`. If the drivers remain supported, preserve rationale in documentation/tests rather than commented executable code.
- Commented boards, ports, obsolete platform packages, and upload fragments in `platformio.ini` once supported environments are settled.
- The commented `BlockingReaderWriterQueue` inside the vendored queue. Prefer updating/re-vendoring an unmodified upstream copy rather than editing it piecemeal.

### Build/configuration cruft

- The legacy `esp32` environment does not build for several independent reasons and selects both mains. Remove it or restore it as a tested target with an explicit source filter and dependencies.
- The relay has no route to the shared header. Add a shared library path, not a relative include copied between projects.
- `build_src_filter` excludes many headers, but filters should primarily select translation units/directories. It also excludes absent `main.cpp` and generated/example secrets files. A directory layout per target makes filters short and auditable.
- A plain `pio run -e esp32s3` attempted upload because of project target configuration; use build-only defaults for repeatable CI and explicit upload commands for hardware workflows.

## 5. Include hygiene

### Unnecessary or leaking includes

| File | Problem | Recommendation |
|---|---|---|
| `settings.h` | Includes `EEPROM.h` even for pin consumers and mixes target storage implementation into every include. It also relies on indirect math/assert declarations on ESP. | Split `board_pins.h`, `calibration.h`, and a `.cpp` persistence backend; include `<cmath>`/`<cassert>` where actually used. |
| `adc/sampling.h` | Includes `esp_compat.h` on STM, which brings Arduino, FreeRTOS, and `stm32h5xx_hal.h`, solely to timestamp a sample. | Use standard integer types in the interface and obtain time from an injected/platform clock in `.cpp`. |
| `i2c.h` | Exposes `esp_err_t` to both targets and includes ESP compatibility/RTOS details. The `port` argument is ignored. | Return a portable `I2cResult`/`bool`, own `TwoWire&` in `I2cBus`, and hide mutex implementation in `.cpp`. |
| `aux_switch.h` | Pulls Arduino, EEPROM, ESP GPIO/logging, parsing, and persistence into all consumers. | Split pure parser, `AuxSwitch` interface, persistence backend, and platform GPIO/hold implementation. |
| `util.h` | Pulls FreeRTOS and ESP-only `Point.h` into every utility consumer because it also defines `TaskNotification`. | Give notification and Influx their own headers; forward-declare where possible. |
| `energy_counter.h` | Includes Arduino, settings/EEPROM, util/FreeRTOS/Point, and the full queue implementation in a public domain header. | Move implementation to `.cpp`; depend on narrow sampler, store, clock, queue, and logger interfaces. |
| `ina228.h` | On ESP includes `SPI.h` and `INA226_WE.h` although neither is used. It also uses direct `Wire` alongside `i2c.h`. | Remove both unused includes and route all register access through `I2cBus`. |
| `ina226.h` | Includes unused `SPI.h` and all of `ina228.h` merely for register constants/I2C helpers, thereby importing globals and definitions. | Include a small INA register/constants/device header. |
| `ina228_mux.h` | Includes `settings.h` and `util.h` mainly for persistence/notification; duplicates INA constants to avoid including `ina228.h`. | The proposed device, store, and notification interfaces remove this workaround. |
| `tmp117.h/.cpp` | Header includes all Arduino for basic declarations; `.cpp` includes `esp32-hal.h` even though no ESP-only symbol requires it. | Use `<cstdint>`/sampler interface in the header and a platform logger/clock in `.cpp`. |
| `platform.h` / `esp_compat.h` | Public headers expose EEPROM, ESP logging/timer/GPIO or STM32 HAL/FreeRTOS. Most wrappers are unused. | Define narrow platform interfaces; put ESP/STM includes and implementations in target `.cpp` files. |
| `ble.h` | Includes a dead ESP BLE stack branch, direct ESP APIs, OTA, aux persistence, and all callbacks in one header. | Keep NimBLE/OTA in ESP-only `.cpp` files and expose only the transport class declaration. |
| `ble_protocol.h` | Includes `string.h` only to obtain transitive basic declarations; defines an unused native-layout header. | Use `<cstdint>`, `<cstddef>`, spans/views, and explicit encoding functions. |
| `main_esp32.cpp` | Unused `<map>`, `USB.h`, and numerous unused legacy ADC headers; dormant Influx/Wi-Fi objects retain dependencies. | Include only selected devices/services. |
| `main_stm32.cpp` | Includes console, settings, I2C, aux, util, and energy headers already reached through higher-level headers; more importantly omits `ble_bridge.h`. | Let a composition-root header expose only construction types and instantiate the real bridge. |
| `sampler_registry.h` | Includes `util.h` for scan/log side effects and exposes public vectors. | Move implementation to `.cpp` and inject diagnostics/logger. |
| `lcd.h` | Relies on transitive declarations of `Wire` and `Sample`; it is not self-contained. | If retained, include its direct dependencies and compile only in an LCD library/target. |
| `atomicops.h`, `readerwriterqueue.h` | Duplicate `<cerrno>`/`<new>` includes and carry broad upstream platform support. | Keep an unmodified, versioned upstream dependency rather than locally grooming it. |

### Missing guards and self-containment

These headers have neither `#pragma once` nor a conventional include guard:

- `adc/adc_ads.h`
- `adc/adc_esp.h`
- `adc/adc_esp_dma.h`
- `adc/ads1220.h`
- `adc/ads1262.h`
- `adc/ads131.h`
- `adc/ina226.h`
- generated `secrets.h`

This is especially risky because several define non-inline globals and free functions (`*_instance`, ISR functions) in the header. `ina228.h`, `ina228_mux.h`, and `ble_bridge.h` do have guards but still define external-linkage globals/functions in headers; including them from another translation unit would cause multiple definitions. Use `.cpp` definitions or C++17 `inline` variables only where a global is truly appropriate.

Several legacy headers also fail standalone compilation by relying on include order: `adc_esp*.h` do not include `sampling.h`; `ads1220.h`, `ads1262.h`, and `ads131.h` use `settings` without including its declaration; `lcd.h` uses `Wire` and `Sample` indirectly.

### Inclusion graph risks

No literal include cycle was found, but there are strong transitive-coupling chains that behave like one during maintenance:

- `energy_counter.h -> settings.h -> EEPROM.h` makes the core integrator depend on persistence and Arduino.
- `sampling.h -> esp_compat.h -> stm32h5xx_hal.h` makes the sampler interface depend on a concrete MCU HAL.
- `ina226.h -> ina228.h` imports a different complete driver, global ISR registry, settings, I2C, and utility layer for a few constants/helpers.
- `console.h -> sampler_registry.h -> energy_counter.h` and `console.h -> ina228*.h` make a parser rebuild/import nearly the whole firmware.
- `util.h -> Point.h` makes even task notifications and logging depend on the ESP Influx library.

The current `static settings_t settings` and `static bool auxState` in headers create translation-unit-local state. That avoids linker errors but does not create one coherent application object; mutations can be observed through a different copy depending on which inline function is emitted. Explicit instances passed by reference are safer and clearer.

## 6. Class design recommendations

| Type/decision | Recommendation | Rationale |
|---|---|---|
| `PowerSampler` lifecycle | Separate `start()` (initial activation) from `recover()` (stall action). Add `stop()`/`shutdown()` only when OTA, reconfiguration, or low-power code has a real caller; a default no-op is sufficient initially. | `startReading()` currently means “begin next conversion” for ADS, “re-arm configuration” for INA228, reset mux state, and no-op for TMP117. A symmetric pure virtual now would add boilerplate without defined semantics. |
| `PowerSampler` base | Add a virtual destructor if objects may ever be owned/deleted polymorphically; otherwise document non-owning/static lifetime and use references. Add `override` consistently. | The base is polymorphic but has no virtual destructor. Current global lifetime happens to avoid deletion. |
| `EnergyCounter::sampler` | Use `PowerSampler&` or `std::reference_wrapper<PowerSampler>`, not an owning smart pointer and not a nullable raw pointer. | Samplers have static/application lifetime, and two mux channel samplers share one backend. Ownership belongs in `FirmwareApp`/registry; the counter only borrows. |
| Counter name | Use `std::string_view` for static names, or a fixed validated device-name type used directly by the encoder. | Every current name is a string literal. Dynamic `std::string` allocation/copy is unnecessary; a fixed type also handles the 16-byte wire field and termination/truncation deliberately. |
| `EnergyCounter` state | Define and test invariants for reset, summary, and recovery. | `reset()` currently leaves charge/`LastI`, stall timers/streak/backoff, queued samples, last diagnostic, print counters, and wire index partly intact. It is unclear whether “reset” means energy only or full session reset. |
| Wire summary | Encode explicitly and set every semantic field, especially `data.e`. Do not serialize a packed native C++ object as the protocol definition. | Current summary captures `Energy` for printing but omits it from `WireSample`; native floats/endian/layout and an unaligned packed `uint64_t` bind the protocol to compiler/MCU ABI. `Sample::diag` and `WireSample::diag` are also redundant on wire. |
| `SamplerRegistry` | Do not make it a singleton. Make it an owned member of `FirmwareApp`, keep vectors private, and expose iteration/find/control methods. | Passing a reference is explicit and testable. A singleton would increase hidden coupling; the current global instance already supplies the required lifetime without making global access desirable. |
| Registry construction | Let registry/app own sampler objects or receive a static descriptor span, then construct counters once after successful initialization. | The separate public `entries` and `counters` vectors duplicate associations and allow invalid mutation. |
| `Telemetry::onIdleSleep` | Inject an `IdlePolicy&` or a small function pointer/context pair. Avoid `std::function` if heap/code size matters. | There is one platform variation, so inheritance and a vtable are more machinery than needed. A policy also makes STM “log only” and ESP deep sleep explicit. If inheritance stays, add a virtual destructor. |
| `Telemetry` ownership | Construct telemetry as an application member/static object, not `new` a never-deleted pointer after setup. | Lifetime is deterministic and dependencies already exist before tasks start. |
| `Sample` | Keep it a plain measurement value with pure helpers such as `power()`. Move clock acquisition to samplers/a `SampleFactory`. Consider separate `MeasurementSample` and `WireSampleV1` types. | Domain measurement, acquisition time source, and protocol packing are separate concerns. |
| `I2cBus` | Own `TwoWire&` and one mutex; expose `probe`, `write`, `read`, `readU16BE`, and a portable result. A nested RAII transaction can hold the lock across a repeated-start sequence. | This removes the ignored port parameter, ESP error leak, duplicated direct-Wire helpers, and ambiguity about lock ownership. |
| `AuxSwitch` | Own state and coordinate GPIO/persistence/hold through injected adapters; keep parsing pure. | Bundles the exact data and methods currently scattered around one header-local boolean. |
| `TaskNotification` | Rename to a domain-neutral RTOS primitive or `DataReadySignal` and isolate its platform implementation. | It is synchronization, not a general utility, and currently forces FreeRTOS into unrelated headers. |
| `BleTransport` | Keep backend polymorphism, but consider `TelemetryTransport`; add typed aux/link methods only if they truly belong to every backend. | Direct BLE and UART framing share an interface contract, not implementation. Factoring them into one class would blur responsibilities. |
| BLE UART parser | Prefer one reusable stateful class over free/static file state. | The relay parser currently uses many file-level globals; the STM parser embeds the same state in `BleBridge`. One tested class serves both. |

`UIP<T>` is not a generally reused abstraction: it appears only as `UIP<MeanWindow>` for `winPoint` and `winPrint`. Replace it with a named `MeasurementWindows` struct containing `voltage`, `current`, `power`, and `temperature`; that communicates the invariant and avoids template indirection with no reuse.

## 7. Suggested file reorganization

One reasonable end state is:

```text
lib/
  shunt-protocol/
    include/shunt_protocol/
      aux_command.h
      crc16.h
      frame.h
      frame_parser.h
      gatt_ids.h
      telemetry_v1.h
    src/
      frame.cpp
      frame_parser.cpp
  readerwriterqueue/             # pinned, unmodified upstream + license/version

src/
  app/
    firmware_app.h
    firmware_app.cpp
    app_context.h
  config/
    board_pins.h
    settings_store.h
    calibration_store.h
  domain/
    measurement_sample.h
    energy_integrator.h
    energy_integrator.cpp
    measurement_windows.h
    telemetry_summary.h
  sampling/
    power_sampler.h
    sampler_monitor.h
    sampler_registry.h
    sampler_registry.cpp
  drivers/
    i2c/
      i2c_bus.h
      i2c_bus.cpp
    ina228/
      ina228_device.h
      ina228_device.cpp
      continuous_sampler.h
      muxed_sampler.h
    tmp117/
      tmp117_sampler.h
      tmp117_sampler.cpp
    aux/
      aux_switch.h
      aux_switch.cpp
  services/
    console.h
    console.cpp
    telemetry.h
    telemetry.cpp
    log.h
    clock.h
  transport/
    telemetry_transport.h
    esp32_nimble/
      nimble_transport.h
      nimble_transport.cpp
      indication_queue.h
      ota_gatt_service.h
    stm32_uart/
      uart_bridge_transport.h
      uart_bridge_transport.cpp
  platform/
    esp32/
      board.cpp
      clock.cpp
      settings_backend.cpp
      wifi.cpp
      influx.cpp
    stm32h5/
      board.cpp
      clock.cpp
      settings_backend.cpp
  main_esp32.cpp
  main_stm32.cpp

ble-bridge/
  src/
    main.cpp
    relay_app.h
    relay_app.cpp
    relay_gatt.h
    relay_gatt.cpp

experimental/                    # only if legacy hardware must remain buildable
  ads1115/
  ads1220/
  ads1262/
  ads131/
  esp_adc/
  ina226/
  lcd/
```

Both root firmware and `ble-bridge` should consume `lib/shunt-protocol` through an explicit shared library path (or be environments in one PlatformIO project). Target directories make `build_src_filter` small: common sources plus one platform directory and one main, instead of a growing blacklist of headers and abandoned drivers.

Recommended implementation order:

1. Fix the functional architecture: make the relay build, share/test the frame codec/parser, and instantiate `BleBridge` in the STM32 composition root.
2. Remove unregistered legacy globals/includes and either delete or isolate legacy drivers; make all declared environments build-only by default and CI-tested.
3. Introduce `FirmwareApp` to remove duplicated setup/task code without touching driver behavior.
4. Extract `I2cBus`/`Ina228Device`, typed persistence, and `AuxSwitch`; these eliminate the most damaging platform/global coupling.
5. Split `EnergyCounter` and `BleSrv`, with explicit telemetry serialization and tests for energy, reset, queue, CRC, and reconnect behavior.
