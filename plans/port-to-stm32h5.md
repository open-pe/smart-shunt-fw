# Port Plan: smart-shunt-fw to STM32H5 Nucleo-64 (MB1814B / NUCLEO-H503RB)

## Target Board

- **Board**: ST Nucleo-64 (MB1814B, revision B)
- **MCU**: STM32H503RB — ARM Cortex-M33 @ 250 MHz, **128 KB flash, 32 KB SRAM**, LQFP64
- **Onboard debugger**: STLINK-V3EC (USB CDC virtual COM port via USART3)
- **USB**: USB FS device mode on PA11/PA12 (user USB-C connector)
- **Headers**: Arduino Uno V3 + ST morpho (full GPIO access)
- **Clocks**: 24 MHz HSE, 32.768 kHz LSE

## Current Firmware (ESP32-S3)

- Arduino framework on ESP-IDF, PlatformIO build
- Dual-core: realTimeTask (core 1, prio 20) + nonRealTimeTask (core 0, prio 1)
- BLE: NimBLE-Arduino GATT server (telemetry indicate, OTA, aux switch)
- WiFi: disabled by default (`disableWifi = true`)
- OTA: dual-partition with rollback safety net
- I2C: INA228 + INA228 mux + TMP117 (primary ADCs)
- SPI: ADS1220/ADS1262/ADS131 (optional, not active)
- ADC drivers behind clean `PowerSampler` abstract base class
- Energy integration: trapezoidal, `WireSample` (64 bytes, CRC16) for telemetry

## Architecture Decision

**Split-firmware with external BLE module:**

```
[INA228 sensor board] --I2C--> [STM32H503RB] --UART--> [ESP32 BLE module] --BLE--> [Phone app]
                                  measurement           GATT relay
                                  core                  (existing protocol)
```

- **STM32H503RB**: measurement core (INA228 sampling, energy counting, calibration, aux switch, serial console via STLINK VCP)
- **External BLE module** (ESP32/nRF52): runs GATT server with the **same service/characteristic UUIDs and WireSample payload format** as the existing `ble.h`, so the existing phone app works unchanged
- **UART serial bridge**: binary framed protocol between STM32 and BLE module

This preserves the existing BLE protocol exactly. The BLE module firmware reuses the existing `ble.h` NimBLE code with a UART relay layer added.

## Build System

### New PlatformIO environment

Add `[env:stm32h5]` to `platformio.ini`:

```ini
[env:stm32h5]
platform = ststm32
board = nucleo_h503rb
framework = arduino
lib_deps =
    stm32duino/FreeRTOS@^1.0.0
    adafruit/Adafruit ADS1X15@^2.4.0
    wollewald/INA226_WE@^1.2.6
build_flags =
    -DTARGET_STM32H5
    -DARDUINO_NUCLEO_H503RB
    -O2
build_unflags = -Os
upload_protocol = stlink
monitor_speed = 115200
```

- Board `nucleo_h503rb` is supported in STM32duino since release 2.7.0
- STLINK-V3EC handles upload + serial monitor (no external probe needed)
- FreeRTOS library provides task management on single-core
- WiFi/InfluxDB/NimBLE libraries NOT included (no radio on STM32H503)

## File Plan

### Keep as-is (MCU-independent, no ESP deps)

| File | Notes |
|---|---|
| `src/sampling.h` | PowerSampler ABC + Sample struct. Pure C++, no deps. |
| `src/energy_counter.h` | Energy integration, WireSample, queue. Needs `#ifdef` to exclude InfluxDB Point. |
| `src/mean_window.h` | Running mean/max. No deps. |
| `src/readerwriterqueue.h` | Lock-free SPSC queue. No deps. |
| `src/atomicops.h` | Atomics. No deps. |
| `src/adc/ina228.h` | INA228 raw register driver. Uses i2c.h + TaskNotification. Already has `IRAM_ATTR` guard. |
| `src/adc/ina228_mux.h` | INA228 + TMUX8612 mux. Uses i2c.h + GPIO. |
| `src/adc/tmp117.h` / `.cpp` | TMP117 temperature sensor. Uses i2c.h. |
| `src/adc/ina226.h` | INA226 via INA226_WE library. Optional. |
| `src/adc/adc_ads.h` | ADS1115 via Adafruit library. Optional. |

### Adapt (ESP-specific to STM32)

| File | Changes needed |
|---|---|
| `src/settings.h` | New pin mappings for Nucleo-64 Arduino headers. EEPROM API: STM32duino `EEPROM` uses `EEPROM.put()` / `EEPROM.get()` or `EEPROM.write()` / `EEPROM.read()` with `EEPROM.commit()`. Verify API compatibility. Replace `ESP_LOGI` in `storeCalibrationFactors` with `Serial.printf`. |
| `src/i2c.h` | Replace `<freertos/FreeRTOS.h>` / `<freertos/semphr.h>` with `<FreeRTOS.h>` / `<semphr.h>`. Replace `esp_err_t` returns with `int` (or include `esp_compat.h`). Replace `ESP_LOGE` with compat shim. `Wire` API is the same. |
| `src/util.h` | Replace ESP-IDF FreeRTOS paths. Remove `IRAM_ATTR` (or define as no-op). `TaskNotification` class: replace `IRAM_ATTR` on `notifyFromIsr()`. Replace `Point.h` include (InfluxDB) with `#ifdef`. |
| `src/util.cpp` | Drop all WiFi/InfluxDB/UDP code. Keep `scan_i2c()` and `UART_LOG()`. Replace ESP-IDF UART driver with `Serial`. |
| `src/aux_switch.h` | Replace `#include <driver/gpio.h>` with Arduino `pinMode`/`digitalWrite`. Replace `gpio_hold_en` / `gpio_deep_sleep_hold_en` (deep sleep hold) — stub for now, implement with STM32 RTC backup registers later. Replace `AUX_PIN` from `GPIO_NUM_10` to an Arduino pin (D7 = PA8). EEPROM API adaptation. Replace `ESP_LOGI`. |
| `src/main.cpp` | **Major rewrite.** See dedicated section below. |
| `src/adc/ina228.h` | Remove stray `#include <SPI.h>`. The `INA226_WE.h` include is likely unnecessary for INA228 raw register access — verify and remove if possible. |

### Drop

| File | Reason |
|---|---|
| `src/ble.h` | Replaced by `ble_bridge.h` (UART to external BLE module) |
| `src/adc/adc_esp.h` | ESP32 internal ADC — not applicable |
| `src/adc/adc_esp_dma.h` | ESP32 DMA ADC — not applicable |
| `src/adc/ads1220.h` | SPI ADC — flash budget, add later if needed |
| `src/adc/ads1262.h` + `src/adc/ads1262/` | SPI ADC — flash budget |
| `src/adc/ads131.h` + `src/adc/ads131/` | SPI ADC — flash budget |
| `src/lcd.h` | SSD1306 — flash budget, optional |
| `src/sui.h` | Empty stub |
| `parts.csv` | ESP32 partition table — not applicable |
| `esp-ota-ble` symlink dep | OTA dropped (128 KB can't do dual-bank) |

### Create

| File | Purpose |
|---|---|
| `src/esp_compat.h` | Thin shims: `esp_err_t`, `ESP_OK`/`ESP_FAIL`, `ESP_LOG*`, `IRAM_ATTR`, `esp_timer_get_time()`, `ESP_ERROR_CHECK`. Allows keeping existing driver code with minimal edits. |
| `src/ble_bridge.h` | UART serial protocol to external BLE module. Provides same interface as `BleSrv` (send, tick, isConnected, flush) but communicates over UART instead of NimBLE. |
| `src/ble_protocol.h` | Shared frame format definitions (message types, frame structure). Included by both STM32 firmware and BLE module firmware. |
| `ble-bridge/` (new directory) | Separate PlatformIO env or project for the ESP32 BLE relay module firmware. Reuses existing `ble.h` + adds UART relay. |

## ESP Compatibility Shim (`esp_compat.h`)

Rather than editing `esp_err_t` / `ESP_LOG*` / `IRAM_ATTR` / `esp_timer_get_time()` in every file, create a thin header:

```cpp
#pragma once
#include <Arduino.h>

// esp_err_t
typedef int esp_err_t;
#define ESP_OK          0
#define ESP_FAIL        -1
#define ESP_ERR_TIMEOUT -2

// ESP_LOG* → Serial.printf
#define ESP_LOGI(tag, fmt, ...)  do { Serial.printf("[%s] " fmt "\r\n", tag, ##__VA_ARGS__); } while(0)
#define ESP_LOGW(tag, fmt, ...)  do { Serial.printf("[%s] W: " fmt "\r\n", tag, ##__VA_ARGS__); } while(0)
#define ESP_LOGE(tag, fmt, ...)  do { Serial.printf("[%s] E: " fmt "\r\n", tag, ##__VA_ARGS__); } while(0)

// ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) do { auto _r = (x); if (_r != ESP_OK) { Serial.printf("ESP_ERROR_CHECK failed: %d\r\n", _r); } } while(0)

// IRAM_ATTR — no-op on STM32 (code runs from flash, no IRAM concept)
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// esp_timer_get_time() — 64-bit microsecond uptime
// millis() * 1000 is coarse but safe; for finer resolution use a TIM timer
inline int64_t esp_timer_get_time() {
    return (int64_t)micros();
    // NOTE: micros() is 32-bit and wraps at ~71.6 min. The idle-sleep logic
    // uses esp_timer_get_time() for 1-hour idle detection, so this wrap is
    // a known issue. Fix: use a 64-bit extension with TIM2 overflow counting.
    // For initial port, use millis() * 1000 (coarser, but no wrap):
    // return (int64_t)millis() * 1000;
}

// ESP.restart() → NVIC_SystemReset()
inline void esp_restart_impl() { NVIC_SystemReset(); }
#define ESP.restart() esp_restart_impl()
```

**Known issue**: `micros()` wraps at ~71.6 minutes (32-bit). The idle-sleep logic uses `esp_timer_get_time()` for 1-hour detection. For the initial port, use `millis() * 1000` (coarser but no wrap). A proper fix uses TIM2 as a 64-bit microsecond counter with overflow ISR.

## FreeRTOS Adaptation

STM32duino provides FreeRTOS via the `stm32duino/FreeRTOS` library.

| ESP-IDF | STM32duino |
|---|---|
| `<freertos/FreeRTOS.h>` | `<FreeRTOS.h>` |
| `<freertos/task.h>` | `<task.h>` |
| `<freertos/semphr.h>` | `<semphr.h>` |
| `xTaskCreatePinnedToCore(fn, name, stack, arg, prio, handle, core)` | `xTaskCreate(fn, name, stack, arg, prio, handle)` |
| `xPortGetCoreID()` | remove (single-core) |
| `xTaskGetAffinity()` | remove |
| `vTaskNotifyGiveFromISR` + `portYIELD_FROM_ISR` | same API (CMSIS) |

**Task structure** (single-core, priority-based preemption):
- `realTimeTask`: priority 20 (high) — tight sampling loop
- `nonRealTimeTask`: priority 1 (low) — BLE bridge, console, telemetry output
- FreeRTOS preemption lets the high-prio task run whenever it has work, and the low-prio task runs during `vTaskDelay` gaps

## Pin Mapping (Nucleo-64 Arduino Headers)

From STM32duino variant `variant_NUCLEO_H503RB.h`:

| Signal | Arduino Pin | STM32 Pin | Variant Index | Notes |
|---|---|---|---|---|
| I2C1 SCL | I2C header (CN5) | PB8 | 47 | Wire default SCL |
| I2C1 SDA | I2C header (CN5) | PB9 | (morpho) | Wire default SDA |
| INA228 ALERT (0x40) | D2 | PA10 | 2 | GPIO interrupt (falling) |
| INA228 mux ALERT (0x41) | D3 | PB3 | 3 | GPIO interrupt (falling) |
| Mux S1 | D4 | PB5 | 4 | GPIO output |
| Mux S2 | D5 | PB4 | 5 | GPIO output |
| Mux Zero | D6 | PB10 | 6 | GPIO output |
| Aux switch | D7 | PA8 | 7 | GPIO output (no deep-sleep hold yet) |
| BLE UART TX | D1 | PB14 | 1 | UART to BLE module |
| BLE UART RX | D0 | PB15 | 0 | UART from BLE module |
| Console (STLINK VCP) | — | PA3/PA4 | — | USART3, Serial default |
| User LED | LED_BUILTIN | PA5 | 13 (D13) | Status indicator |
| User button | USER_BTN | PC13 | 21 | Optional |

**Note**: PB9 (I2C SDA) is on the morpho connector (CN7/CN10), not an Arduino numbered pin. It is accessible via Wire's default configuration. If PB9 is hard to reach, use `Wire.setSDA(PB7)` / `Wire.setSCL(PB6)` as alternates (both on Arduino header).

**BLE UART**: D0/D1 (PB15/PB14) can be used with a second `HardwareSerial` instance (e.g., `Serial2` on USART2 if those pins map to it, or software serial). Exact USART peripheral assignment TBD during implementation — STM32duino's `HardwareSerial` supports pin remapping.

## main.cpp Rewrite

### Remove
- All OTA code: `esp_ota_ops.h`, `bootWatchdog`, `markOtaValidIfHealthy`, `verifyRollbackLater`
- All BLE code: `ble.h` include, `BleSrv bleSrv`, `bleSrv.begin()`, `bleSrv.send()`, `bleSrv.tick()`, `bleSrv.flush()`
- WiFi/InfluxDB: `WiFi.h`, `InfluxDbClient.h`, `connect_wifi_async()`, `wait_for_wifi()`, `timeSync()`
- ESP-IDF UART: `driver/uart.h`, `uartInit()`, `uart_get_buffered_data_len()`, `uart_read_bytes()`
- Dual-core: `xTaskCreatePinnedToCore()`, `xPortGetCoreID()`, core affinity asserts
- Deep sleep: `ESP.deepSleep()`, `gpio_hold_en()`, `gpio_deep_sleep_hold_en()` (stub for now)
- `USB.h` (ESP USB CDC)

### Keep (with adaptation)
- `Wire.begin()` — same API, STM32duino Wire
- Sampler initialization loop — same logic
- `realTimeTask` — same sampling loop, `xTaskCreate` instead of `xTaskCreatePinnedToCore`
- `nonRealTimeTask` / `update()` — same structure, with `bleSrv` replaced by `bleBridge`
- Console command parser (`handleConsoleInput`) — keep `calibrate`, `ina22x-resistor-range`, `aux`, `reset`, `help`; drop `bootinfo`, `wifi`
- Serial input — use `Serial.available()` / `Serial.readString()` (STM32duino Serial works reliably, unlike the ESP32 PlatformIO workaround)
- Idle-sleep vote logic — keep the safety logic, stub the actual sleep (or implement STM32 Stop mode)

### New structure

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>

#include "esp_compat.h"
#include "settings.h"
#include "i2c.h"
#include "ble_bridge.h"
#include "energy_counter.h"
#include "util.h"
#include "aux_switch.h"
#include "adc/ina228.h"
#include "adc/ina228_mux.h"
#include "adc/tmp117.h"

// Samplers (same as ESP32-S3, minus disabled SPI ADCs)
PowerSampler_INA228 ina228_40{0x40};
INA228MuxBackend muxBackend{0x41, ...};
PowerSampler_MuxChannel mux_chA{muxBackend, ...};
PowerSampler_MuxChannel mux_chB{muxBackend, ...};
PowerSampler_TMP117 tmp117{0x48};

BleBridge bleBridge;  // replaces BleSrv

void setup() {
    Serial.begin(115200);
    auxBegin();
    Wire.begin();  // default Nucleo I2C pins, 100kHz (bump to 400kHz)
    bleBridge.begin();  // init UART to BLE module
    // ... sampler init loop (same) ...
    xTaskCreate(realTimeTask, "rt", 4096, NULL, 20, NULL);
    xTaskCreate(nonRealTimeTask, "nrt", 4096, NULL, 1, NULL);
}

void loop() { vTaskDelay(1000); }
```

## BLE Bridge Serial Protocol (`ble_protocol.h` + `ble_bridge.h`)

### Frame format

```
[0xAA] [0x55] [TYPE] [LEN_HI] [LEN_LO] [PAYLOAD...] [CRC16_HI] [CRC16_LO]
```

- Start: `0xAA 0x55` (2 bytes)
- Type: 1 byte
- Length: 2 bytes big-endian (payload length, max 255)
- Payload: `LEN` bytes
- CRC16: 2 bytes (CRC-16/CCITT over Type + Length + Payload)

### Message types

| Type | Direction | Payload | Notes |
|---|---|---|---|
| `0x01 TELEMETRY` | STM32→BLE | WireSample (64 bytes) | BLE module sends as indication on telemetry characteristic |
| `0x02 AUX_SET` | BLE→STM32 | 1 byte (0/1) | BLE module relays aux write from phone app |
| `0x03 AUX_STATE` | STM32→BLE | 1 byte (0/1) | BLE module updates aux characteristic + notify |
| `0x04 CALIB_SET` | BLE→STM32 | idx(1) + dim(1) + factor(4) | Calibration factor write |
| `0x05 RESET_CMD` | BLE→STM32 | (none) | Reset energy counters |
| `0x06 RESISTOR_RANGE` | BLE→STM32 | devIdx(1) + res(4) + range(4) | Set INA22x resistor range |
| `0x07 BOOTINFO` | STM32→BLE | text string | Boot info for BLE read requests |
| `0x08 LINK_STATUS` | BLE→STM32 | 1 byte (connected/subscribed) | Lets STM32 know if BLE client is active |
| `0x09 CONSOLE_LOG` | STM32→BLE | text string | Mirror serial log to BLE (optional, for debugging) |

### `BleBridge` class interface

Provides the same methods the existing `BleSrv` exposes that `main.cpp` calls:

```cpp
class BleBridge {
    HardwareSerial &uart;  // Serial2 or similar
public:
    void begin();
    void send(const uint8_t *data, size_t len);  // telemetry → UART frame
    void flush();
    void tick();  // process incoming UART frames, dispatch aux/calib commands
    bool isConnected() const;  // link status from BLE module
    // aux notification relay
    void publishAux(bool on);
};
```

## BLE Module Firmware (separate project/env)

The BLE relay module runs on an ESP32 (or ESP32-C3, or nRF52 with ArduinoBLE). For ESP32:

- Reuses existing `ble.h` NimBLE GATT server code **as-is** (same service/characteristic UUIDs)
- Adds a UART receive loop that:
  - Parses incoming frames from STM32
  - On `TELEMETRY` frame: calls `bleSrv.send()` with the WireSample payload
  - On `AUX_STATE` frame: updates aux characteristic + notify
  - On `BOOTINFO` frame: stores for read requests
- BLE write callbacks:
  - On aux write: sends `AUX_SET` frame to STM32 via UART
  - On calibrate (if we add a characteristic): sends `CALIB_SET` frame
- Link status: sends `LINK_STATUS` frame on connect/disconnect

**Directory**: `ble-bridge/` in the same repo, with its own `platformio.ini` env targeting ESP32.

## EEPROM Adaptation

STM32duino provides `EEPROM` via flash emulation. API differences:

| ESP32 (arduino-esp32) | STM32duino |
|---|---|
| `EEPROM.begin(size)` | `EEPROM.begin(size)` (or implicit) |
| `EEPROM.writeFloat(addr, val)` | `EEPROM.put(addr, val)` (template) |
| `EEPROM.readFloat(addr)` | `EEPROM.get(addr, val)` (template) |
| `EEPROM.commit()` | `EEPROM.commit()` or automatic on `end()` |
| `EEPROM.end()` | `EEPROM.end()` |

**Action**: Create `#ifdef TARGET_STM32H5` blocks in `settings.h` and `aux_switch.h` that use `EEPROM.put()` / `EEPROM.get()` instead of `EEPROM.writeFloat()` / `EEPROM.readFloat()`. The NVS layout (16-byte header + calibration slots) stays the same.

## Flash Budget (128 KB)

| Component | Est. size |
|---|---|
| STM32duino core (HAL + Arduino runtime) | ~25-30 KB |
| FreeRTOS | ~8 KB |
| Wire (I2C) | ~3-5 KB |
| INA228 driver + INA228 mux | ~5-8 KB |
| TMP117 driver | ~2-3 KB |
| Energy counter + sampling + mean window | ~5-8 KB |
| BLE bridge (UART protocol) | ~3-5 KB |
| Console (Serial) + command parser | ~3-5 KB |
| EEPROM + calibration + aux switch | ~3-5 KB |
| ReaderWriterQueue + atomics | ~1-2 KB |
| esp_compat.h shims | ~0.5 KB |
| **Total estimate** | **~60-75 KB** |
| **Available** | **128 KB** |
| **Headroom** | **~50-65 KB** |

Fits with comfortable margin. SPI ADC drivers (ADS1220/ADS1262/ADS131) can be added later if needed (~10-15 KB each with their libraries).

**Dropped to fit**: OTA (no dual-bank), WiFi/InfluxDB, LCD/SSD1306, ESP internal ADC, USB CDC (use STLINK VCP instead).

## Implementation Order

### Phase 1: Bring up the measurement core (no BLE)

1. **Create `esp_compat.h`** — shims for esp_err_t, ESP_LOG*, IRAM_ATTR, esp_timer_get_time
2. **Add `[env:stm32h5]` to `platformio.ini`** — ststm32 platform, nucleo_h503rb board
3. **Rewrite `settings.h`** — new pin mapping for Nucleo-64 Arduino headers, EEPROM adaptation
4. **Adapt `i2c.h`** — FreeRTOS include paths, esp_err_t via compat shim
5. **Adapt `util.h` + `util.cpp`** — drop WiFi/InfluxDB, fix FreeRTOS paths, keep scan_i2c + UART_LOG
6. **Adapt `aux_switch.h`** — STM32 pin, drop gpio_hold, EEPROM adaptation
7. **Adapt `ina228.h`** — remove stray `#include <SPI.h>`, verify no other ESP deps
8. **Rewrite `main.cpp`** — single-core tasks, no OTA/BLE/WiFi, STLINK VCP console, stub BLE bridge
9. **Build + flash** — verify I2C scan finds INA228, verify sampling works, console output on STLINK VCP

### Phase 2: BLE bridge

10. **Create `ble_protocol.h`** — frame format, message types
11. **Create `ble_bridge.h`** — BleBridge class (UART send/receive, same interface as BleSrv)
12. **Integrate into `main.cpp`** — replace stub with BleBridge, wire up telemetry send + aux relay
13. **Create `ble-bridge/` project** — ESP32 firmware reusing ble.h + UART relay
14. **Test end-to-end** — STM32 → UART → ESP32 BLE → phone app (verify WireSample protocol)

### Phase 3: Polish (optional, later)

15. **64-bit microsecond timer** — TIM2 overflow ISR for proper `esp_timer_get_time()` (fixes 71.6-min wrap)
16. **STM32 Stop mode** — implement idle sleep using WFI + RTC wake (replaces ESP deep sleep)
17. **USB CDC console** — use STM32H5 USB FS for console instead of STLINK VCP (faster, no STLINK needed)
18. **SPI ADC drivers** — add ADS1220/ADS131 if needed (flash budget allows)

## Key Risks

1. **I2C SDA pin (PB9)**: On the morpho connector, not an Arduino header pin. Verify it's accessible on the board. Fallback: use `Wire.setSDA(PB7)` / `Wire.setSCL(PB6)` which are on Arduino header.
2. **EEPROM wear**: STM32duino EEPROM emulation uses flash pages. Frequent writes (calibration, aux state) have limited endurance. Same concern as ESP32 NVS, but STM32 flash may have fewer write cycles. Mitigation: only write on actual changes (already done in aux_switch.h).
3. **FreeRTOS stack sizes**: 4096*4 = 16 KB per task on ESP32. STM32H5 has 32 KB SRAM total. Reduce to 2048 words (8 KB) per task. With two tasks + Arduino + FreeRTOS idle, ~20 KB used, leaving ~12 KB for heap/stacks.
4. **`micros()` wrap**: 32-bit micros() wraps at 71.6 min. Idle-sleep detection needs 1-hour timing. Use `millis() * 1000` initially (1 ms resolution is fine for 1-hour idle detection).
5. **BLE UART throughput**: WireSample is 64 bytes + frame overhead ~6 bytes = ~70 bytes per sample. At 115200 baud that's ~6 ms per sample. With 400 ms cadence (current firmware), this is fine. If faster telemetry is needed, bump UART to 921600 baud.
6. **STM32duino library compatibility**: `Adafruit ADS1X15` and `INA226_WE` libraries should work on STM32duino (they use Wire), but verify during build. `NimBLE-Arduino` definitely does NOT work — that's why we use the external BLE module approach.
