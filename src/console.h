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
#include <esp_system.h>
#include <esp_timer.h>
#include <soc/rtc_cntl_reg.h>
extern bool otaBleActive();
extern bool disableWifi;
extern bool wifiTimeSyncOnly;
extern volatile int g_wifiRequest;
/* Free functions, not the Telemetry object: telemetry.h includes THIS header to call
 * handleConsoleInput(), so including it back would be a cycle. Defined in
 * main_esp32.cpp, where the instance is actually in scope. */
extern unsigned long telemetryGetIntervalUs();
extern bool telemetrySetIntervalUs(unsigned long us);
extern unsigned long telemetryMinIntervalUs();
#endif

/* EVERY CONSOLE RESPONSE GOES THROUGH UART_LOG, INCLUDING THE ERRORS.
 *
 * That is not a style preference. A command can now arrive over BLE (see the remote-console
 * section in ble.h), and the reply reaches that caller through the UART_LOG sink installed
 * for the duration of the command -- not through esp_log, which is deliberately not hooked
 * because it would splice every other task's output into the reply.
 *
 * So an error reported with ESP_LOGW prints on the serial port and leaves a remote caller
 * with an empty reply, which reads exactly like success. Six of them were converted for this
 * reason; do not add a seventh. */
inline void handleConsoleInput(const String &buf, SamplerRegistry &registry, BleTransport &ble) {
    while (1) {
        String inp(buf);
        inp.trim();
        if (inp.length() == 0) break;

        if (inp == "reset-counters") {
            ESP_LOGW("main", "Reset in 1s!");
            delay(1000);
            registry.resetAll();
            /* The counters are zeroed in place -- nothing reboots -- so this reply does
             * reach a remote caller, unlike the `download` case help warns about. */
            UART_LOG("energy counters zeroed");
        } else if (inp.startsWith("calibrate ")) {
            std::string samplerName = inp.substring(10, inp.indexOf(' ', 10)).c_str();
            size_t i = 10 + samplerName.size() + 1;

            EnergyCounter *ec = registry.findByName(samplerName);
            if (!ec) {
                UART_LOG("calibrate: no sampler named '%s' (names carry the board prefix)",
                         samplerName.c_str());
                break;
            }

            auto dim = inp.substring(i, i + 1);
            dim.toUpperCase();
            i += 2;
            bool multiply = (inp.charAt(i) == '*');
            if (multiply)++i;
            auto factor = inp.substring(i).toFloat();
            if (!checkCalibrationFactorBounds(factor)) {
                UART_LOG("calibrate: factor %.9f out of bounds, rejected", factor);
                break;
            }

#if !APPLY_U_GAIN_CAL
            if (dim == "U") {
                UART_LOG("calibrate: U gain calibration is disabled (APPLY_U_GAIN_CAL=0)");
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
                UART_LOG("calibrate: unknown dimension '%s', expected U or I", dim.c_str());
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
                UART_LOG("ina22x-resistor-range: no INA22x instance on this board");
            }
        } else if (inp == "aux" || inp.startsWith("aux ")) {
            String arg = inp.substring(3);
            arg.trim();
            if (arg.length() == 0) {
                UART_LOG("aux = %s", auxGet() ? "ON" : "off");
            } else {
                bool want;
                if (!auxParseCommand((const uint8_t *) arg.c_str(), arg.length(), want)) {
                    UART_LOG("aux: expected on, off or toggle");
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
            /* Per-board identity, persisted in NVS. Deliberately requires a reboot to
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
                UART_LOG("board-prefix = '%s', reboot to apply", boardPrefixGet());
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
             * control has no defensible channel identity.
             *
             * `hold a|b` is the OTHER kind of park -- same stopped relays, but the
             * held channel keeps publishing. It exists because an idle round-robin
             * ticks the contacts 0.428 times per second per relay for no reader.
             * See the Mode comment in relay_mux_adc.h for what a hold gives up. */
            String v(inp.substring(8));
            v.trim();
            v.toLowerCase();
            if (v.length() == 0) {
                UART_LOG("relaymux %s%s%s: state=%s requested=%s settled=%s serving=%s gen=%lu",
                         RelayMuxAdcBackend::modeName(relayMuxAdc.requestedMode()),
                         relayMuxAdc.isHeld() ? " on " : "",
                         relayMuxAdc.isHeld()
                             ? RelayMux2::targetName(relayMuxAdc.heldChannel()) : "",
                         relayMux.stateName(),
                         RelayMux2::targetName(relayMux.requested()),
                         RelayMux2::targetName(relayMux.settledTarget()),
                         RelayMux2::targetName(relayMuxAdc.servingChannel()),
                         (unsigned long) relayMux.generation());
                UART_LOG("  dead %lums + settle %lums per switch (floor %lums)",
                         (unsigned long) RelayMux2::DEAD_TIME_MS,
                         (unsigned long) relayMux.settleMs(),
                         (unsigned long) RelayMux2::SETTLE_FLOOR_MS);
                uint32_t lastMs, maxMs, closeMs, noStep, unsettled, widened, pairBroken, unmatched;
                float spanV, stableEps;
                relayMuxAdc.settleStats(lastMs, maxMs, closeMs, noStep, unsettled, widened,
                                        pairBroken, unmatched, spanV, stableEps);
                UART_LOG("  span %.6g V, stability gate %s (stableEpsV %.6g) -- set it from "
                         "the span, not by guessing",
                         (double) spanV, stableEps > 0 ? "ON" : "OFF (fixed timer only)",
                         (double) stableEps);
                /* Learned from the conversions the discard throws away. settle is
                 * an UPPER bound: stability needs 3 agreeing conversions, ~156 ms
                 * at 19.15 SPS, so a faster contact cannot be resolved here. */
                UART_LOG("  observed: step at %lums, settled by %lums (worst %lums)",
                         (unsigned long) closeMs, (unsigned long) lastMs, (unsigned long) maxMs);
                UART_LOG("  faults: no-step %lu, never-settled %lu, auto-widened %lu",
                         (unsigned long) noStep, (unsigned long) unsettled,
                         (unsigned long) widened);
                /* NOT a fault: the reading matched neither channel's last value,
                 * which is what a changed input looks like. Published anyway, with
                 * DIAG_RELAY_UNMATCHED. It rises once per deliberate source change
                 * and stays put; a count that climbs steadily is worth a look. */
                UART_LOG("  unmatched (published, diag-flagged): %lu",
                         (unsigned long) unmatched);
                /* Each published sample is the mean of AVG_CONVERSIONS CONSECUTIVE
                 * conversions; restarts count runs where generation() jumped by more
                 * than one, i.e. the RT task was too slow to keep the run adjacent.
                 * A rising count is a starved loop, not a bad contact. */
                UART_LOG("  averaging %lu consecutive conversions/sample, %lu run restarts",
                         (unsigned long) RelayMuxAdcBackend::avgConversions(),
                         (unsigned long) pairBroken);
                UART_LOG("  'relaymux a|b|off|auto' (park, silent) | 'relaymux hold a|b' "
                         "(park, keeps publishing) | 'relaymux settle <ms>'");
                break;
            }
            if (v.startsWith("hold")) {
                String n(v.substring(4));
                n.trim();
                RelayMux2::Target h;
                if (n == "a")      h = RelayMux2::Target::CH_A;
                else if (n == "b") h = RelayMux2::Target::CH_B;
                else {
                    /* UART_LOG, not ESP_LOGW: an error reported through the logger
                     * reaches the serial port only, and leaves a caller on the BLE
                     * console with an empty reply that reads exactly like success
                     * (doc/ota-over-ble.md). */
                    UART_LOG("relaymux hold: expected a or b. A hold with no channel is "
                             "'relaymux off', which also stops publishing.");
                    break;
                }
                /* Both the mode and its target cross to the RT task in one call, in
                 * the order the consumer acquires them -- unlike the manual path
                 * below, which posts the mode here and the target into the mux's
                 * own inbox as two independent writes. */
                if (!relayMuxAdc.setHold(h)) {
                    UART_LOG("relaymux hold: refused (no channel)");
                    break;
                }
                /* A hold is applied from the RT task's hasDataFor(), which returns
                 * early when the ADC never came up -- unlike `relaymux a`, whose
                 * target reaches tick() through the mux's own inbox regardless. Say
                 * so rather than reporting a park that will not happen. */
                if (!relayMuxAdc.isInitialised()) {
                    UART_LOG("relaymux hold: ADC not initialised -- the mode is recorded but "
                             "NOTHING WILL PARK. Use 'relaymux a|b' (silent) to move the relay "
                             "on a board whose ADC is down.");
                    break;
                }
                UART_LOG("relaymux: holding %s -- it keeps publishing, %s goes silent, and "
                         "the relays stop switching (in ~%lums if it has to move first)",
                         RelayMux2::targetName(h),
                         RelayMux2::targetName(h == RelayMux2::Target::CH_A
                                                   ? RelayMux2::Target::CH_B
                                                   : RelayMux2::Target::CH_A),
                         (unsigned long) (RelayMux2::DEAD_TIME_MS + relayMux.settleMs()));
                /* Said at the point of use, not only in a header comment: a held
                 * channel looks exactly like a healthy round-robin channel in
                 * InfluxDB, and the thing that changed is invisible from there. */
                UART_LOG("  the A/B ratio is NOT valid across a hold -- the two channels are "
                         "no longer sampled close together, so ADC drift stops cancelling.");
                UART_LOG("  'relaymux auto' resumes; treat the first cycles after that as "
                         "warm-up (one relay has been carrying all 71mW of coil).");
                /* The one thing an operator cannot infer from the data itself. */
                UART_LOG("  NOTE: the no-step guard is inactive while held -- nothing switches, "
                         "so a dropped coil reads as a plausible voltage, not a fault.");
                UART_LOG("  Held samples carry diag bit 0x%06lx so the database can tell them "
                         "from round-robin ones.",
                         (unsigned long) RelayMuxAdcBackend::diagHeldBit());
                break;
            }
            if (v.startsWith("settle")) {
                String n(v.substring(6));
                n.trim();
                if (n.length() == 0) {
                    UART_LOG("relaymux settle = %lums (floor %lums)",
                             (unsigned long) relayMux.settleMs(),
                             (unsigned long) RelayMux2::SETTLE_FLOOR_MS);
                    break;
                }
                /* Narrowing is a HUMAN decision on purpose. The firmware widens
                 * itself when it observes a late contact, but will not shorten a
                 * physical settle on the strength of its own measurements -- see
                 * the auto-widen note in relay_mux_adc.h. Not persisted: it
                 * reverts to the built-in default on reset. */
                /* Parse SIGNED and reject non-positive. `relaymux settle -5` used to
                 * become (uint32_t)-5 = 4294967291, sail past the floor check, and
                 * install a ~49-day settle from which the mux never leaves
                 * SETTLING -- and 3*settleMs downstream then overflowed. */
                const long want = n.toInt();
                if (want <= 0) {
                    UART_LOG("relaymux settle: expected a positive millisecond value "
                             "(%lu..%lu)", (unsigned long) RelayMux2::SETTLE_FLOOR_MS,
                             (unsigned long) RelayMux2::SETTLE_MAX_MS);
                    break;
                }
                const uint32_t got = relayMux.setSettleMs((uint32_t) want);
                UART_LOG("relaymux settle -> %lums%s (not persisted; the driver may widen "
                         "it again if a contact settles later than this)",
                         (unsigned long) got,
                         (unsigned long) got != (unsigned long) want ? " (clamped)" : "");
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
                UART_LOG("relaymux: expected a, b, off, auto, hold a|b or settle <ms>");
                break;
            }
            relayMuxAdc.setManual(true);
            /* NOT select(): that mutates the state machine, and this handler runs
             * on appTask while tick() runs on realTimeTask. */
            relayMux.requestFromOtherTask(t);
            /* The command RETURNS BEFORE THE CONTACT MOVES. select() only starts
             * the documented sequence; ARM does not even rise until the dead time
             * expires. Saying "selected" here would be a claim about the hardware
             * that this firmware has not yet made, let alone verified -- so report
             * the request and the time it will take, and let `relaymux` with no
             * argument report what actually settled. */
            /* The LIVE settle, not switchLatencyMs()'s compile-time default -- after a
             * `relaymux settle 40` or an auto-widen the two disagree, and the status
             * line one command away would contradict this one. */
            UART_LOG("relaymux: requested %s, settles in ~%lums (re-run 'relaymux' to confirm)",
                     RelayMux2::targetName(t),
                     (unsigned long) (RelayMux2::DEAD_TIME_MS + relayMux.settleMs()));
        }
#endif
        else if (inp == "bletx" || inp.startsWith("bletx ")) {
            String v(inp.substring(5));
            v.trim();
            if (v.length() == 0) {
                UART_LOG("bletx %d dBm", NimBLEDevice::getPower());
                break;
            }
            const long dbm = v.toInt();
            /* The radio accepts only a discrete ladder and rounds to it, so the
             * READ-BACK after the set is the truth, not the argument. Out-of-range
             * setPower() fails and the old value stands, which would otherwise be
             * indistinguishable from a successful set. */
            if (dbm < -27 || dbm > 20) {
                UART_LOG("bletx: -27..20 dBm");
                break;
            }
            const int was = NimBLEDevice::getPower();
            if (!NimBLEDevice::setPower((int8_t) dbm))
                UART_LOG("bletx: setPower(%ld) refused, still %d dBm", dbm, was);
            else
                UART_LOG("bletx %d -> %d dBm (radio rounds to its own ladder)",
                         was, NimBLEDevice::getPower());
        } else if (inp == "blerate" || inp.startsWith("blerate ")) {
            String v(inp.substring(7));
            v.trim();
            if (v.length() == 0) {
                UART_LOG("blerate %lu ms", telemetryGetIntervalUs() / 1000);
                break;
            }
            const long ms = v.toInt();
            if (!telemetrySetIntervalUs((unsigned long) ms * 1000))
                /* The floor moves with the negotiated connection interval, so report the
                 * number that actually applies right now rather than a constant the
                 * reader would then have to go and look up. */
                UART_LOG("blerate: %ld ms rejected, still %lu ms (allowed %lu..700 ms; the "
                         "floor is 2x the negotiated connection interval -- one indication "
                         "costs ~1.6 of them, and 1.03x measured 64%% delivery)",
                         ms, telemetryGetIntervalUs() / 1000, telemetryMinIntervalUs() / 1000);
            else
                UART_LOG("blerate %lu ms", telemetryGetIntervalUs() / 1000);
        } else if (inp.startsWith("bleconn ")) {
            /* bleconn <itvlMin> <itvlMax> <latency>, in the LL's own 1.25 ms / event
             * units, so what is typed is what goes on the air. */
            String a(inp.substring(8));
            a.trim();
            const int s1 = a.indexOf(' '), s2 = a.indexOf(' ', s1 + 1);
            if (s1 < 0 || s2 < 0) {
                UART_LOG("bleconn <itvlMin> <itvlMax> <latency>   (1.25ms units)");
                break;
            }
            const long mn = a.substring(0, s1).toInt(), mx = a.substring(s1 + 1, s2).toInt(),
                       lat = a.substring(s2 + 1).toInt();
            if (mn < 6 || mx < mn || mx > 3200 || lat < 0 || lat > 30) {
                UART_LOG("bleconn: itvl 6..3200 (min<=max), latency 0..30");
                break;
            }
            /* Supervision timeout is fixed at 4000 ms, so the skip window must stay
             * well inside it or a quiet stretch drops the link:
             * (1 + latency) * itvlMax * 1.25 ms, doubled for margin. Refusing here
             * beats discovering it as a reconnect loop on a board across the bench. */
            const float skipMs = (1 + lat) * mx * 1.25f;
            if (skipMs * 2 >= 4000.f) {
                UART_LOG("bleconn: (1+%ld)*%ld*1.25 = %.0f ms skip window needs a supervision "
                         "timeout > %.0f ms, but it is fixed at 4000 -- refused",
                         lat, mx, skipMs, skipMs * 2);
                break;
            }
            ble.requestConnParams((uint16_t) mn, (uint16_t) mx, (uint16_t) lat);
            UART_LOG("bleconn requested %.1f-%.1f ms lat %ld (skip window %.0f ms); the central "
                     "may refuse -- watch the pwr: line for what landed",
                     mn * 1.25f, mx * 1.25f, lat, skipMs);
        }
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
        } else if (inp == "i2c" || inp == "i2c recover") {
            const bool recover = inp != "i2c";
            bool any = false;
            for (uint8_t port = 0; port < 2; ++port) {
                I2cBus &b = i2c_bus(port);
                if (b.sda < 0) continue; // never started on this board
                any = true;
                if (recover) {
                    UART_LOG("i2c bus %u: recovery %s", port,
                             i2c_recover_bus(port, "console") ? "ok" : "FAILED (see log)");
                }
                UART_LOG("i2c bus %u SDA %d SCL %d %lu kHz%s: errors %lu (streak %lu), "
                         "recovered %lu, failed %lu, generation %lu",
                         port, b.sda, b.scl, (unsigned long) (b.freq / 1000),
                         b.running ? "" : " NOT RUNNING", (unsigned long) b.errors,
                         (unsigned long) b.policy.streak, (unsigned long) b.recoveries,
                         (unsigned long) b.failedRecoveries, (unsigned long) b.generation);
            }
            // Never an empty reply: over BLE that reads exactly like success.
            if (!any) UART_LOG("i2c: no bus started on this board");
        }
#endif
#ifndef TARGET_STM32H5
        else if (inp == "reboot") {
            /* Deferred so the reply and its CON DONE terminator reach a remote caller
             * before the link drops. */
            static esp_timer_handle_t rebootTimer = nullptr;
            if (!rebootTimer) {
                esp_timer_create_args_t args = {};
                args.callback = [](void *) { Serial.flush(); esp_restart(); };
                args.name = "reboot";
                esp_timer_create(&args, &rebootTimer);
            }
            if (rebootTimer && esp_timer_is_active(rebootTimer)) {
                UART_LOG("reboot already pending");
            } else if (rebootTimer && esp_timer_start_once(rebootTimer, 800 * 1000) == ESP_OK) {
                UART_LOG("rebooting in 0.8 s");
            } else {
                UART_LOG("reboot: could not arm the timer");
            }
        }
        else if (inp == "download") {
            /* Reboot straight into the ROM downloader, so a board whose USB-CDC
             * has stopped serving esptool can still be reflashed without someone
             * physically holding BOOT and tapping RESET. That is not hypothetical:
             * the xiao_nest riser prints nothing and refuses every upload while
             * happily clicking relays and streaming telemetry over BLE.
             *
             * THE BIT SURVIVES A CPU RESET -- it is in the RTC domain -- and an
             * earlier version of this comment claimed setup() would therefore
             * always disarm it. THAT IS FALSE, and review caught it: while the
             * chip sits in the ROM downloader the app never runs, so setup()
             * never executes and cannot clear anything. Pressing RESET returns
             * you to the downloader.
             *
             * Only two things actually clear it: esptool zeroes it on the reset
             * it performs after a flash (tool-esptoolpy esp32s3.py,
             * RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK) -- though it ignores failure to
             * do so -- and a real POWER CYCLE, which resets the RTC domain to the
             * register's documented default of zero. setup()'s REG_CLR_BIT is
             * still worth having, but it only covers the case where the app did
             * boot; it is not the safety net it was described as.
             *
             * COSTS: the app stops. No sampling, no telemetry, no BLE. The relays
             * release, because nothing drives ARM/REQ and the mux board's 100k
             * pull-downs park every relay off. */
            UART_LOG("rebooting into ROM download mode -- sampling, telemetry and BLE stop, "
                     "and the relays release. Recover by FLASHING it, or by a POWER CYCLE; "
                     "pressing reset just returns to the downloader.");
            Serial.flush();
            delay(100);
            REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
            esp_restart();
        }
#endif
        else if (inp == "help") {
            /* THE LIST IS THE INTERFACE. This is now the only discovery mechanism a remote
             * caller has -- there is no tab completion over BLE and no man page -- so a
             * command missing from here is a command nobody else knows exists. Two of them
             * were (`relaymux settle`, the whole ble group) before the remote console made
             * that cost visible. Keep it in the order the commands are handled above, and
             * add the line in the same commit as the command. */
            UART_LOG("aux [on|off|toggle]           get/set the load switch (persisted)");
            UART_LOG("reset-counters                zero the energy counters");
            UART_LOG("calibrate <sampler> <U|I> [*]<factor>");
            UART_LOG("                              PREFIXED sampler name, e.g. ftr_DCCT");
            UART_LOG("ina22x-resistor-range[2|3] <ohm> <max A>");
            UART_LOG("board-prefix [<name>]         series-name prefix (NVS, needs reboot)");
#ifndef TARGET_STM32H5
            UART_LOG("wifi on|off                   WiFi + the firmware's own InfluxDB writer");
            UART_LOG("bletx [dBm]                   radio power, -27..20 (rounds to a ladder)");
            UART_LOG("blerate [ms]                  telemetry interval");
            UART_LOG("bleconn <min> <max> <latency> ask for link params, 1.25ms units");
            UART_LOG("cpufreq [80|160|240]          CPU clock + die temp (not persisted)");
            UART_LOG("bootinfo                      running slot, OTA verify state, uptime");
            UART_LOG("i2c [recover]                 bus error/recovery counters; force a recovery");
            UART_LOG("reboot                        restart the app (applies board-prefix)");
            UART_LOG("download                      reboot into ROM download mode");
#ifdef WITH_RELAY_MUX
            UART_LOG("relaymux                      mux state, settle stats, fault counts");
            UART_LOG("relaymux a|b|off              park a channel (STOPS PUBLISHING)");
            UART_LOG("relaymux hold a|b             park it and KEEP PUBLISHING (no ratio)");
            UART_LOG("relaymux auto                 resume the A/B round-robin");
            UART_LOG("relaymux settle [ms]          settle time (not persisted)");
#endif
            /* Only true over BLE, and cheap to print unconditionally: on the serial console
             * it is a harmless note about the other transport, while a remote caller who
             * cannot see this line has no way to learn the framing its client depends on. */
            UART_LOG("--");
            UART_LOG("over BLE: write 'cmd <line>' to the OTA control characteristic;");
            UART_LOG("  replies come back as 'CON <text>' and end with 'CON DONE ok|err <n>'.");
            UART_LOG("  'download' reboots before the reply can be sent, so it has no");
            UART_LOG("  terminator -- send the bare 'download' verb, which notifies first.");
#endif
        } else {
            UART_LOG("Unknown command '%s'. enter 'help' for help", inp.c_str());
        }
        break;
    }
}
