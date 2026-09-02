#pragma once

#include <Arduino.h>
#include "sampler_registry.h"
#include "ble_transport.h"
#include "aux_switch.h"
#include "settings.h"
#include "util.h"
#include "adc/ina228.h"
#include "adc/ina228_mux.h"
#ifdef WITH_RELAY_MUX
#include "relay_mux.h"
#include "adc/relay_mux_adc.h"
/* Defined in main_esp32.cpp, which includes this header BEFORE it defines them. */
extern RelayMux2 relayMux;
extern RelayMuxAdcBackend relayMuxAdc;
#endif

#ifndef TARGET_STM32H5
#include <esp_ota_ops.h>
extern bool otaBleActive();
extern bool disableWifi;
extern bool wifiTimeSyncOnly;
extern volatile int g_wifiRequest;
#endif

inline void handleConsoleInput(const String &buf, SamplerRegistry &registry, BleTransport &ble) {
    while (1) {
        String inp(buf);
        inp.trim();
        if (inp.length() == 0) break;

        if (inp == "reset") {
            ESP_LOGW("main", "Reset in 1s!");
            delay(1000);
            registry.resetAll();
        } else if (inp.startsWith("calibrate ")) {
            std::string samplerName = inp.substring(10, inp.indexOf(' ', 10)).c_str();
            size_t i = 10 + samplerName.size() + 1;

            EnergyCounter *ec = registry.findByName(samplerName);
            if (!ec) {
                ESP_LOGW("main", "Sampler with name %s not found", samplerName.c_str());
                break;
            }

            auto dim = inp.substring(i, i + 1);
            dim.toUpperCase();
            i += 2;
            bool multiply = (inp.charAt(i) == '*');
            if (multiply)++i;
            auto factor = inp.substring(i).toFloat();
            if (!checkCalibrationFactorBounds(factor)) {
                ESP_LOGW("main", "Calibration factor %.9f out of bounds, rejected", factor);
                break;
            }

#if !APPLY_U_GAIN_CAL
            if (dim == "U") {
                ESP_LOGW("main", "U gain calibration is disabled (APPLY_U_GAIN_CAL=0)");
                break;
            }
#endif
            float was = dim == "U" ? ec->calibFactorU : ec->calibFactorI;
            UART_LOG("%s: set calibration factor for [%s] = %.8f (was %.8f)", samplerName.c_str(), dim.c_str(),
                     multiply ? (was * factor) : factor, was);
            if (dim == "U") {
                ec->setCalibrationFactors(factor, NAN, multiply);
            } else if (dim == "I") {
                ec->setCalibrationFactors(NAN, factor, multiply);
            } else {
                ESP_LOGW("main", "unknown dim %s", dim.c_str());
            }
        } else if (inp.startsWith("ina22x-resistor-range")) {
            int devIdx = 0;
            size_t i = strlen("ina22x-resistor-range");
            if (inp.length() > i && (inp[i] >= '2' and inp[i] <= '3')) {
                devIdx = 1 + (inp[i] - '2');
                ++i;
            }
            auto resStr = inp.substring(i + 1, inp.indexOf(' ', i + 1));
            i += resStr.length() + 1;
            auto range = inp.substring(i).toFloat();
            auto res = resStr.toFloat();

            if (ina228_instance[devIdx]) {
                UART_LOG("INA228[%i] setResistorRange(%.3fmOhm, %.3fA)", devIdx, res * 1e3f, range);
                ina228_instance[devIdx]->setResistorRange(res, range);
            } else if (devIdx == 1 && ina228_mux_instance) {
                UART_LOG("INA228 mux setResistorRange(%.3fmOhm, %.3fA)", res * 1e3f, range);
                ina228_mux_instance->setResistorRange(res, range);
            } else {
                ESP_LOGW("main", "No INA22x instance!");
            }
        } else if (inp == "aux" || inp.startsWith("aux ")) {
            String arg = inp.substring(3);
            arg.trim();
            if (arg.length() == 0) {
                UART_LOG("aux = %s", auxGet() ? "ON" : "off");
            } else {
                bool want;
                if (!auxParseCommand((const uint8_t *) arg.c_str(), arg.length(), want)) {
                    ESP_LOGW("aux", "expected: aux [on|off|toggle]");
                } else {
                    auxSet(want);
                    UART_LOG("aux = %s", auxGet() ? "ON" : "off");
                }
            }
        }
#ifndef TARGET_STM32H5
        else if (inp == "wifi on") {
            disableWifi = false;
            wifiTimeSyncOnly = false;
            g_wifiRequest = 1;
            UART_LOG("WiFi connect requested (async via wifiTask)");
        } else if (inp == "wifi off") {
            disableWifi = true;
            g_wifiRequest = 0;
            UART_LOG("WiFi disconnect requested (async via wifiTask)");
        } else if (inp == "board-prefix" || inp.startsWith("board-prefix ")) {
            /* Per-board identity, persisted in NVS. Deliberately requires a reset to
             * take effect: sampler names are baked into the EnergyCounters at
             * initAll(), and renaming a live counter would split its series
             * mid-flight and orphan the energy total accumulated under the old
             * name. */
            String v = inp.substring(strlen("board-prefix"));
            v.trim();
            if (v.length() == 0) {
                const char *p = boardPrefixGet();
                UART_LOG("board-prefix = '%s'%s", p, p[0] ? "" : " (unset -- names are unprefixed)");
            } else if (boardPrefixSet(v.c_str())) {
                UART_LOG("board-prefix = '%s', reset to apply", boardPrefixGet());
            } else {
                UART_LOG("board-prefix rejected (1..%u chars of [A-Za-z0-9_])",
                         (unsigned) BOARD_PREFIX_MAX);
            }
        }
#ifdef WITH_RELAY_MUX
        else if (inp == "relaymux" || inp.startsWith("relaymux ")) {
            /* Bench control for the Coto relay mux. DESIGN.md's revision-A open
             * gates require observing the contacts through supply, temperature and
             * asynchronous-command tests, and measuring the release time the
             * dead-time constant is currently only guessing at -- none of which is
             * possible while the round-robin moves the contact every ~600 ms.
             * Parking a channel is what makes that measurement available without a
             * rebuild. Publishing stops while parked: a sample taken under manual
             * control has no defensible channel identity. */
            String v(inp.substring(8));
            v.trim();
            v.toLowerCase();
            if (v.length() == 0) {
                UART_LOG("relaymux %s: state=%s requested=%s settled=%s serving=%s gen=%lu",
                         relayMuxAdc.isManual() ? "MANUAL (not publishing)" : "auto",
                         relayMux.stateName(),
                         RelayMux2::targetName(relayMux.requested()),
                         RelayMux2::targetName(relayMux.settledTarget()),
                         RelayMux2::targetName(relayMuxAdc.servingChannel()),
                         (unsigned long) relayMux.generation());
                UART_LOG("  dead %lums + settle %lums per switch; 'relaymux a|b|off|auto'",
                         (unsigned long) RelayMux2::DEAD_TIME_MS,
                         (unsigned long) RelayMux2::SETTLE_MS);
                break;
            }
            if (v == "auto") {
                relayMuxAdc.setManual(false);
                UART_LOG("relaymux: round-robin resumed");
                break;
            }
            RelayMux2::Target t;
            if (v == "a")        t = RelayMux2::Target::CH_A;
            else if (v == "b")   t = RelayMux2::Target::CH_B;
            else if (v == "off") t = RelayMux2::Target::NONE;
            else {
                UART_LOG("relaymux: expected a, b, off or auto");
                break;
            }
            relayMuxAdc.setManual(true);
            relayMux.select(t);
            /* The command RETURNS BEFORE THE CONTACT MOVES. select() only starts
             * the documented sequence; ARM does not even rise until the dead time
             * expires. Saying "selected" here would be a claim about the hardware
             * that this firmware has not yet made, let alone verified -- so report
             * the request and the time it will take, and let `relaymux` with no
             * argument report what actually settled. */
            UART_LOG("relaymux: requested %s, settles in ~%lums (re-run 'relaymux' to confirm)",
                     RelayMux2::targetName(t), (unsigned long) RelayMux2::switchLatencyMs());
        }
#endif
        else if (inp == "cpufreq" || inp.startsWith("cpufreq ")) {
            /* Read-back always reports the die temperature too, because the whole
             * point of the knob is trading clock for heat and there is otherwise no
             * number for "the chip feels warm". temperatureRead() is the S3's
             * on-die sensor: coarse (a few K) and self-heated, so it is a trend
             * indicator, not a calibrated measurement. */
            String v(inp.substring(7));
            v.trim();
            if (v.length() == 0) {
                UART_LOG("cpufreq %u MHz (xtal %u, apb %u), die %.1f C",
                         (unsigned) getCpuFrequencyMhz(), (unsigned) getXtalFrequencyMhz(),
                         (unsigned) (getApbFrequency() / 1000000), temperatureRead());
                break;
            }
            const long mhz = v.toInt();
            /* 80 is the floor: below it the PLL powers down, taking the
             * USB-Serial-JTAG console with it -- i.e. the command would remove the
             * only way to undo itself, recoverable only by a physical BOOT+RESET.
             * See the CPU_FREQ_MHZ note in main_esp32.cpp. */
            if (mhz != 80 && mhz != 160 && mhz != 240) {
                UART_LOG("cpufreq: only 80, 160 or 240 (80 is the floor: lower kills the PLL and this console)");
                break;
            }
            const unsigned was = getCpuFrequencyMhz();
            if (!setCpuFrequencyMhz((uint32_t) mhz)) {
                UART_LOG("cpufreq: setCpuFrequencyMhz(%ld) refused, still %u MHz", mhz, was);
                break;
            }
            /* NOT persisted. A frequency that turns out to break sampling or BLE is
             * then one power-cycle away from gone, instead of latched into NVS on a
             * board that may no longer be reachable. Make it permanent by building
             * with -D CPU_FREQ_MHZ=<n>. */
            UART_LOG("cpufreq %u -> %u MHz (apb %u MHz, die %.1f C) -- not persisted, reboots to %u",
                     was, (unsigned) getCpuFrequencyMhz(),
                     (unsigned) (getApbFrequency() / 1000000), temperatureRead(),
                     (unsigned) CPU_FREQ_MHZ);
        } else if (inp == "bootinfo") {
            const esp_partition_t *running = esp_ota_get_running_partition();
            esp_ota_img_states_t st;
            const char *state = "unknown";
            if (running && esp_ota_get_state_partition(running, &st) == ESP_OK) {
                switch (st) {
                    case ESP_OTA_IMG_NEW:            state = "NEW"; break;
                    case ESP_OTA_IMG_PENDING_VERIFY: state = "PENDING_VERIFY"; break;
                    case ESP_OTA_IMG_VALID:          state = "VALID"; break;
                    case ESP_OTA_IMG_INVALID:        state = "INVALID"; break;
                    case ESP_OTA_IMG_ABORTED:        state = "ABORTED"; break;
                    case ESP_OTA_IMG_UNDEFINED:      state = "UNDEFINED"; break;
                }
            }
            const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
            UART_LOG("running=%s state=%s next=%s slot=%u B, uptime=%lus, ota=%s",
                     running ? running->label : "?", state, next ? next->label : "none",
                     (unsigned) (next ? next->size : 0), (unsigned long) (millis() / 1000),
                     otaBleActive() ? "ACTIVE" : "idle");
        }
#endif
        else if (inp == "help") {
            UART_LOG("ina22x-resistor-range <resistance> <max expected current>");
            UART_LOG("calibrate <sampler> <U|I> [*]<factor>");
            UART_LOG("aux [on|off|toggle]");
            UART_LOG("reset   zero the energy counters");
#ifndef TARGET_STM32H5
            UART_LOG("bootinfo              running slot + OTA verify state");
            UART_LOG("wifi on|off           enable/disable WiFi + InfluxDB telemetry");
            UART_LOG("cpufreq [80|160|240]  get/set CPU clock + die temp (not persisted)");
#ifdef WITH_RELAY_MUX
            UART_LOG("relaymux [a|b|off|auto] park the relay mux for bench work, or resume");
#endif
#endif
            UART_LOG("board-prefix [<name>] get/set this board's series-name prefix (NVS, needs reset)");
#ifndef TARGET_STM32H5
#endif
        } else {
            UART_LOG("Unknown command '%s'. enter 'help' for help", inp.c_str());
        }
        break;
    }
}
