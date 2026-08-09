#include <Arduino.h>
#include <Wire.h>

#include <InfluxDbClient.h>
#include <map>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "adc/ina226.h"
#include "adc/ina228.h"
#include "adc/ads1220.h"
#include "adc/ads1262.h"
#include "adc/ads131.h"

#ifdef WITH_LCD
#include "lcd.h"
#endif

#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include <WiFi.h>
//#include "adc/adc_esp_dma.h"
//#if ARDUINO_USB_MODE == 1
#include "USB.h"
//#endif

#include "ble.h"
#include "energy_counter.h"
#include "util.h"
#include "adc/tmp117.h"


InfluxDBClient client;

PowerSampler_ADS ads;
//PowerSampler_ADS1220 ads1220;
//PowerSampler_ADS1262 ads1262;
PowerSampler_ADS131 ads131;

PowerSampler_INA226 ina226;
PowerSampler_INA228 ina228_40{0x40};
PowerSampler_INA228 ina228_41{0x41};
PowerSampler_INA228 ina228_42{0x42};
// PowerSampler_ESP32 esp_adc;

// ADD0 -> GND / V+.  A part that is not fitted simply fails the DEVICE_ID check in
// init() and is never registered, so listing an address costs nothing but one log
// line at boot.  0x4A (ADD0->SDA) and 0x4B (ADD0->SCL) can be added the same way.
PowerSampler_TMP117 tmp117{0x48};
PowerSampler_TMP117 tmp117_49{0x49};

PowerSampler_Dummy dummy{};

unsigned long LastTimeOut = 0;
unsigned long LastTimePrint = 0;

//std::map
//std::array<std::pair<std::string, PowerSampler *>, 2> samplers{
//        std::pair<std::string, PowerSampler *>{"ESP32_ADS", &ads},
//        {"ESP32_INA226", &ina226},
//};

std::map<std::string, PowerSampler *> samplers{
    //{"ESP32_ADS", &ads},
    //{"ESP32_ADS1220",  &ads1220},
    //{"ESP32_ADS1262",  &ads1262},
    //{"ESP32_ADS131M02", &ads131},

    //{"ESP32_INA226", &ina226},

    {"ESP32_INA228", &ina228_40},
    {"ESP32_INA228_2", &ina228_41},
    {"ESP32_INA228_3", &ina228_42},
    {"TMP117", &tmp117},
    {"TMP117_2", &tmp117_49},
    //{"dummy", &dummy},
};

std::vector<EnergyCounter> energyCounters;

/// Declared in ble.h, set by the OTA quiesce hook, read by realTimeTask.
volatile bool g_samplingHalted = false;

BleSrv bleSrv;

bool disableWifi = true;
bool wifiTimeSyncOnly = true;

#ifdef WITH_LCD
LCD lcd;
#endif

// Idle shutdown.  Deliberately on the 64-bit esp_timer clock, not micros(): micros()
// wraps every 71.6 min, which is uncomfortably close to the 1 h threshold below.
int64_t timeLastWakeEvent = 0;

constexpr int64_t IDLE_SLEEP_AFTER_US = 3600ll * 1000000ll; // idle this long -> sleep
constexpr uint64_t IDLE_SLEEP_WAKE_US = 900ull * 1000000ull; // ...and wake up to re-check

/// Idle shutdown must only ever trigger on *evidence* that there is nothing to measure.
/// It used to trigger on the absence of evidence instead, and that took the logger off
/// the air for good:
///   - P is NaN whenever the shunt channel is disabled (vbus-only boards, see
///     PowerSampler_INA228::init) or an I2C read failed.  The old test was
///     `std::abs(P) >= 0.0005f`, and !(|NaN| >= x) is false, so a device that could not
///     measure power at all concluded there was none -- every single window, forever.
///   - ESP.deepSleep(0) configures NO wake source, so "going to sleep" meant "off until
///     someone power-cycles it".
/// Result: every board with a vbus-only INA228 went dark at exactly 60 min of uptime.
/// So: an unmeasurable power counts as activity, and the sleep is always recoverable.
static void noteWakeEvent() { timeLastWakeEvent = esp_timer_get_time(); }

static bool looksActive(float meanPower) {
    if (!std::isfinite(meanPower)) return true; // cannot judge -> stay awake
    return std::abs(meanPower) >= 0.0005f;
}

[[noreturn]] void realTimeTask(void *arg);

[[noreturn]] void nonRealTimeTask(void *arg);

#define RT_CORE 1
constexpr auto RT_PRIO = 20; // highest priority is 24

void vTaskGetRunTimeStats();

// ---- OTA safety net ---------------------------------------------------------------------------
//
// The bootloader shipped by pioarduino has CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y, so a freshly
// OTA'd image boots as PENDING_VERIFY and the *next reset* reverts it unless the app confirms it.
// That gives us a real safety net, but only if two things exist: something that confirms on evidence
// the firmware actually works, and something that guarantees a reset when it doesn't. Without the
// second, an image that hangs in setup() just sits there -- no reset, so no rollback, so a brick that
// needs a wire and a serial adapter. That is not hypothetical; it is how the fugu units were lost.

static constexpr uint32_t OTA_VALIDATE_UPTIME_MS = 60000;
static constexpr uint64_t BOOT_WATCHDOG_TIMEOUT_US = 30ull * 1000000ull;

/// Overrides the weak definition in the Arduino core (esp32-hal-misc.c). The core's default returns
/// false, which makes initArduino() call esp_ota_mark_app_valid_cancel_rollback() immediately --
/// BEFORE setup() runs, on nothing but the fact that the image reached init.
///
/// That silently defeats the entire rollback safety net: by the time markOtaValidIfHealthy() looks,
/// the image is already VALID and there is nothing left to confirm. It was caught on hardware, where
/// the first real OTA came up `state=VALID` at 43s uptime against a 60s confirm gate that had not
/// run. Returning true defers the decision to us, so the image stays PENDING_VERIFY until it has
/// shown it can actually sample -- and a reset before that rolls back, which is the whole point.
extern "C" bool verifyRollbackLater() { return true; }

static esp_timer_handle_t bootWatchdog = nullptr;

static void bootWatchdogFire(void *) {
    // No logging that could itself block -- the whole point is that we got here because something is
    // stuck. A reset with PENDING_VERIFY set is what hands control back to the bootloader.
    esp_restart();
}

static void bootWatchdogArm() {
    esp_timer_create_args_t args = {};
    args.callback = &bootWatchdogFire;
    args.name = "bootwd";
    args.dispatch_method = ESP_TIMER_TASK;
    if (esp_timer_create(&args, &bootWatchdog) != ESP_OK) {
        ESP_LOGE("main", "boot watchdog could not be created -- a bad OTA will brick this device");
        bootWatchdog = nullptr;
        return;
    }
    esp_timer_start_once(bootWatchdog, BOOT_WATCHDOG_TIMEOUT_US);
}

static void bootWatchdogDisarm() {
    if (!bootWatchdog) return;
    esp_timer_stop(bootWatchdog);
    esp_timer_delete(bootWatchdog);
    bootWatchdog = nullptr;
}

/// Confirm a PENDING_VERIFY image, but only on evidence that the device is doing its job. "setup()
/// returned" is not that evidence -- the failure worth catching is exactly a build that boots and
/// then does not measure.
static void markOtaValidIfHealthy() {
    static bool done = false;
    if (done) return;

    unsigned long total = 0;
    for (auto &ec: energyCounters) total += ec.numSamples();

    // Two observations, not one. NumSamples is cumulative, so a single non-zero reading would also
    // be satisfied by a build that sampled once and then wedged -- which is the shape of failure
    // this is here to catch. Snapshot at the halfway mark, then require the count to have MOVED.
    static unsigned long snapshot = 0;
    static bool haveSnapshot = false;
    if (!haveSnapshot) {
        if (millis() < OTA_VALIDATE_UPTIME_MS / 2) return;
        snapshot = total;
        haveSnapshot = true;
        return;
    }
    if (millis() < OTA_VALIDATE_UPTIME_MS) return;
    if (total <= snapshot) {
        // Not sampling. Stay unconfirmed: a reset from here rolls back, which is the right default
        // when we cannot show the image works.
        return;
    }

    done = true;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
            ESP_LOGI("main", "OTA image confirmed valid (rollback cancelled)");
        else
            ESP_LOGE("main", "failed to confirm OTA image valid");
    }
}

void setup(void) {
    bootWatchdogArm();
    Serial.begin(115200);
    // Early, before the long init below: coming out of the idle deep sleep the pad hold has been
    // carrying the load across the reset, and it should be handed back to the firmware promptly.
    auxBegin();
    //#if CONFIG_IDF_TARGET_ESP32S3
    // for unknown reason need to initialize uart0 for serial reading (see loop below)
    // Serial.available() works under Arduino IDE (for both ESP32,ESP32S3), but always returns 0 under platformio
    // so we access the uart port directly. on ESP32 the Serial.begin() is sufficient (since it uses the uart0)
    uartInit(0);
    //#endif

    //USBSerial.begin();
    // USB.begin();

    delay(500);

    ESP_LOGI("main", "SmartShunt started");


    Wire.begin(
        settings.Pin_I2C_SDA,
        settings.Pin_I2C_SCL,
        400000UL
        //1000000UL
    );

    /*if (!lcd.init()) {
        ESP_LOGW("main", "Failed to initialize LCD");
        //scan_i2c();
    }*/


    if (!disableWifi) {
        connect_wifi_async();
        wait_for_wifi();
        timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");
        if (wifiTimeSyncOnly) {
            WiFi.disconnect(true);
            WiFiClass::mode(WIFI_OFF);
            disableWifi = true;
        }
    }

    bleSrv.begin();
    if (sizeof(WireSample) != 56) {
        assert(false);
    }
    if (sizeof(Sample) != 28) {
        assert(false);
    }


    for (auto p: samplers) {
        if (!p.second->init()) {
            ESP_LOGI("main", "%s: Failed to initialize sampler.", p.first.c_str());
        } else {
            energyCounters.emplace_back(EnergyCounter{p.second, p.first, p.second->getStorageId()});
            ESP_LOGI("main", "Initialized energy counter for %s", p.first.c_str());
        }
    }

    if (energyCounters.empty()) {
        scan_i2c();
    }


    if (!disableWifi) {
        client.setConnectionParamsV1("http://homeassistant.local:8086", /*db*/ "open_pe", "home_assistant", "h0me");
        client.setWriteOptions(WriteOptions()
            .writePrecision(WritePrecision::MS)
            .batchSize(200)
            .bufferSize(400)
            .flushInterval(1) // uint16! min is 1
            .retryInterval(0) // 0=disable retry
        );
    }
    // when multiplexing channels, TI recommnads single-shot mode
    for (auto &ec: energyCounters) {
        ec.sampler->startReading();
    }

    xTaskCreatePinnedToCore(realTimeTask, "loopRt", 4096 * 4, NULL, RT_PRIO, NULL, RT_CORE);
    xTaskCreatePinnedToCore(nonRealTimeTask, "loopy", 4096 * 4, NULL, 1, NULL, RT_CORE - 1);

    // Past every blocking call in setup(); from here the tasks carry the liveness argument, and
    // markOtaValidIfHealthy() is what decides whether this image gets to keep running.
    bootWatchdogDisarm();

    //TaskStatus_t *pxTaskStatusArray[20];
    //uxTaskGetSystemState(pxTaskStatusArray, 20);
    //vTaskGetRunTimeStats();
}

std::vector<WireSample> wire_sample_buf;

[[noreturn]] void realTimeTask(void *arg) {
    assert(xPortGetCoreID() == RT_CORE);
    vTaskPrioritySet(nullptr, RT_PRIO);

#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define PRINT_MACRO_AT_COMPILE_TIME(var) #var "=`" VALUE(var) "`"

#if CONFIG_ARDUINO_RUNNING_CORE == RT_CORE or CONFIG_ARDUINO_EVENT_RUNNING_CORE == RT_CORE or \
CONFIG_ARDUINO_UDP_RUNNING_CORE == RT_CORE or CONFIG_ARDUINO_SERIAL_EVENT_TASK_RUNNING_CORE == RT_CORE
    //#pragma message PRINT_MACRO_AT_COMPILE_TIME(CONFIG_ARDUINO_RUNNING_CORE)
    //#error "arduino runtime is configured to run on RT_CORE CONFIG_ARDUINO_RUNNING_CORE="
#endif

    // CONFIG_ESP_TIMER_ISR_AFFINITY != RT_CORE or CONFIG_ESP_TIMER_TASK_AFFINITY == RT_CORE or
#if CONFIG_LWIP_TCPIP_TASK_AFFINITY == RT_CORE \
 or CONFIG_PTHREAD_TASK_CORE_DEFAULT == RT_CORE or CONFIG_FMB_PORT_TASK_AFFINITY == RT_CORE or CONFIG_MDNS_TASK_AFFINITY == RT_CORE
    //#error "esp runtime is configured to run on RT_CORE"
#endif

    while (true) {
        if (g_samplingHalted) {
            // An OTA is writing flash. Erase/write disables the CPU cache and stalls this core, so a
            // tight sampling loop here does nothing but starve the transfer and generate I2C timeouts.
            // Yielding also lets the idle task run, which the 5s panic-on-timeout task watchdog wants.
            vTaskDelay(10);
            continue;
        }
        for (auto &ec: energyCounters) {
            ec.update();
        }
    }
}

void loop() {
    vTaskDelay(1000);
}

void handleConsoleInput(const String &buf) {
    while (1) {
        String inp(buf);
        inp.trim();

        if (inp.isEmpty()) break;

        if (inp == "reset") {
            //Serial0.println("Reset, delay 1s");
            ESP_LOGW("main", "Reset in 1s!");
            delay(1000);
            for (auto &ec: energyCounters) {
                ec.reset();
            }
        } else if (inp.startsWith("calibrate ")) {
            // calibrate ESP32_INA228 I 1.0028870
            // calibrate ESP32_INA228 I 1.00223895 // rsn20-50 with ref=3458a (2025-10) 1.1A
            // calibrate ESP32_INA228_3 U 1.00// 60v
            // calibrate ESP32_INA228 U 1.00
            // calibrate ESP32_INA228 I *0.9997247927345282
            // calibrate ESP32_INA228 U *1.000983433436737
            // calibrate ESP32_ADS U *1.0003957914179227
            // calibrate ESP32_ADS I *1.0017
            // calibrate ESP32_INA228_3 U 1
            // calibrate ESP32_INA228_2 U 1
            // calibrate ESP32_INA228 U 1
            std::string samplerName = inp.substring(10, inp.indexOf(' ', 10)).c_str();
            size_t i = 10 + samplerName.size() + 1;

            EnergyCounter *ec = nullptr;
            for (auto &ec_: energyCounters) {
                if (ec_.name == samplerName) {
                    ec = &ec_;
                    break;
                }
            }

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

            // TODO send factors to influxDB

#if !APPLY_U_GAIN_CAL
            // Reject before logging success: storing a factor that update() never applies
            // would report "set" and change nothing.
            if (dim == "U") {
                ESP_LOGW("main", "U gain calibration is disabled (APPLY_U_GAIN_CAL=0), "
                         "factor %.9f rejected -- calibrate voltage host-side instead", factor);
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
            // todo add offset
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

            if (ina226_instance) {
                UART_LOG("INA226 setResistorRange(%.6f, %.6f)", res, range);
                ina226_instance->setResistorRange(res, range);
            } else if (ina228_instance[devIdx]) {
                UART_LOG("INA228[%i] setResistorRange(%.3fmΩ, %.3fA)", devIdx, res * 1e3f, range);
                ina228_instance[devIdx]->setResistorRange(res, range);
            } else {
                ESP_LOGW("main", "No INA22x instance!");
            }
        } else if (inp == "aux" || inp.startsWith("aux ")) {
            // Runs on the app task, so it may write NVS directly. Also the only way to work the
            // switch with no BLE client attached.
            String arg = inp.substring(3);
            arg.trim();
            if (arg.isEmpty()) {
                UART_LOG("aux GPIO%d = %s", (int) AUX_PIN, auxGet() ? "ON" : "off");
            } else {
                bool want;
                if (!auxParseCommand((const uint8_t *) arg.c_str(), arg.length(), want)) {
                    ESP_LOGW("aux", "expected: aux [on|off|toggle]");
                } else {
                    auxSet(want);
                    UART_LOG("aux GPIO%d = %s", (int) AUX_PIN, auxGet() ? "ON" : "off");
                }
            }
        } else if (inp == "bootinfo") {
            // The only way to see whether a rollback happened: after a bad OTA the device comes back
            // looking perfectly healthy, just running the previous slot.
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
        } else if (inp == "wifi on") {
            disableWifi = false;
            connect_wifi_async();
        } else if (inp == "wifi off") {
            disableWifi = true;
            WiFi.disconnect(true);
        } else if (inp == "help") {
            UART_LOG("ina22x-resistor-range <resistance> <max expected current>");
            UART_LOG("calibrate <sampler> <U|I> [*]<factor>");
            UART_LOG("aux [on|off|toggle]   aux switch on GPIO%d", (int) AUX_PIN);
            UART_LOG("bootinfo              running slot + OTA verify state");
            UART_LOG("reset                 zero the energy counters");
            UART_LOG("wifi on|off");
        } else {
            UART_LOG("Unknown command '%s'. enter 'help' for help", inp.c_str());
        }

        break;
    }
}



void update() {
    constexpr bool hfWrites = false;

    unsigned long nowTime = micros();

    assert(xPortGetCoreID() != RT_CORE);

    //ESP_LOGI("main", "Loop!");

    // OTA runs entirely on this task -- esp_ota_begin/write/end block on flash for far longer than a
    // BLE host callback may. The BLE callbacks only latch a command or copy bytes into the ring; this
    // is where the flash actually gets written. bleSrv.tick() goes after it so the status lines it
    // just produced are notified in the same pass.
    otaBleTick(millis());
    bleSrv.tick();
    if (otaBleActive()) {
        // Deep-sleeping out from under a firmware transfer would be a spectacular way to brick the
        // device: the OTA slot is half-written and the boot partition has not moved. isConnected()
        // covers this incidentally, but not by intent.
        noteWakeEvent();
    }
    markOtaValidIfHealthy();

    for (auto &ec: energyCounters) {
        ec.consumeQueue();
    }

    /*    if (hfWrites) {
          Point point("smart_shunt");
          samplePoint(point, s, "ESP8266_proto1");
          pointFrame[i] = point;
        }

    if (hfWrites) influxWritePointsUDP(&pointFrame[0], pointFrame.size()); */

    if (nowTime - LastTimeOut > 400e3) {
        // every 200 ms => ble missing samples
        // every 19 ms TODO why?
        auto print = nowTime - LastTimePrint > 2000e3;


        uint16_t bleLenPos = 0;
        // Idle-sleep vote, tallied over the whole window rather than decided per
        // counter: a counter that cannot vote must not be able to *cause* sleep by
        // staying silent, and a counter that has no business voting (temperature)
        // must not be able to prevent it.  See the resolution below the loop.
        bool anyPowerVote = false, anyActive = false;
        for (auto &ec: energyCounters) {
            if (ec.newSamplesSinceLastSummary()) {
                if (ec.sampler->measuresPower()) {
                    anyPowerVote = true;
                    if (looksActive(ec.winPrint.P.getMean())) anyActive = true;
                }

                bool newSample;
                auto ws = ec.summary((nowTime - LastTimeOut), print, newSample);
                if (newSample) {
                    if (!disableWifi) wire_sample_buf.push_back(ws);
                    bleSrv.send((uint8_t*)&ws, sizeof(ws));
                }

                if (print) {
                    // lcd.updateValues(ec.printSample);
                }
            }
        }

        /// Sleep only on positive evidence that there IS a power channel and it reads
        /// idle.  Both other outcomes stay awake:
        ///   - anyActive: something is drawing power.
        ///   - !anyPowerVote: no power channel reported this window at all -- every
        ///     sampler failed init, or stalled, or the only samplers registered are
        ///     temperature-only.  That is "cannot judge", not "nothing to measure",
        ///     and it is exactly the confusion that once put vbus-only boards to
        ///     sleep forever at 60 min (see the comment on looksActive above).
        const bool active = anyActive || !anyPowerVote;
        if (active) noteWakeEvent();

        // Make the verdict observable: a guard nobody ever sees fire is not a guard.
        // Logs the first evaluation after boot, then only on a change, so it is
        // silent in steady state.
        {
            static int8_t wasActive = -1;
            if (wasActive != (int8_t) active) {
                UART_LOG("idle-sleep vote: %s (powerVote=%d anyActive=%d) after %.0fs idle",
                         active ? "ACTIVE" : "IDLE", (int) anyPowerVote, (int) anyActive,
                         (double) (esp_timer_get_time() - timeLastWakeEvent) * 1e-6);
                wasActive = (int8_t) active;
            }
        }

        bleSrv.flush();

        if (print) {
            if (energyCounters.size() > 1)
                UART_LOG("");
            LastTimePrint = nowTime;
        }

        if (!disableWifi) {
            for (auto &ws: wire_sample_buf)
                influxWritePointUDP(ws.getInfluxDbPoint());
        }
        wire_sample_buf.clear();


        LastTimeOut = nowTime;
    }


    // for some reason Serial.available() doesn't work under platformio
    // so access the uart port directly

    const uart_port_t uart_num = UART_NUM_0; // Arduino Serial is on port 0
    static char buf[128];
    static int buf_pos = 0;
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
    if (length > 0) {
        length = uart_read_bytes(uart_num, buf + buf_pos, min(length, 127 - buf_pos), 20);
        uart_write_bytes(uart_num, buf + buf_pos, length); // echo
        buf_pos += length;
    }

    auto eol = strchr(buf, '\n');
    while (eol) {
        // enable using break below
        *eol = 0; // terminate string at line break
        buf_pos = 0; // reset buf
        handleConsoleInput(buf);
        noteWakeEvent();
        break;
    }

    static String serialBuf;
    if (Serial.available()) {
        auto r = Serial.readString();
        serialBuf += r;
        Serial.write(r.c_str()); // echo
        Serial.flush();
        int lb;
        while ((lb = serialBuf.indexOf('\n')) != -1) {
            String line = serialBuf.substring(0, lb);
            handleConsoleInput(line);
            serialBuf = serialBuf.substring(lb + 1);
        }
        noteWakeEvent();
    }

    if (esp_timer_get_time() - timeLastWakeEvent > IDLE_SLEEP_AFTER_US) {
        // A subscribed client is the strongest possible evidence that we are not idle.
        // Sleeping out from under it is what made the old failure invisible: the samples
        // just stopped and the peripheral never advertised again.
        if (bleSrv.isConnected()) {
            noteWakeEvent();
        } else {
            UART_LOG("Zero power for %llds, sleeping for %llds (aux=%s)",
                     (long long) (IDLE_SLEEP_AFTER_US / 1000000),
                     (long long) (IDLE_SLEEP_WAKE_US / 1000000), auxGet() ? "ON" : "off");
            Serial.flush();
            // Latch the aux pad so the switch survives the sleep and the wake reset. Without it the
            // load would drop for the whole boot interval every 15 minutes, and nothing would say so.
            auxArmDeepSleepHold();
            ESP.deepSleep(IDLE_SLEEP_WAKE_US); // wakes and re-checks; never a one-way trip
        }
    }
}

void nonRealTimeTask(void *arg) {
    while (1) {
        update();
        vTaskDelay(10);
    }
}


const int BUF_SIZE = 1024;
QueueHandle_t uart_queue;

void uartInit(int port_num) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // UART_HW_FLOWCTRL_CTS_RTS
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    // tx=34, rx=33, stack=2048


#if CONFIG_IDF_TARGET_ESP32S3
    //const int PIN_TX = 34, PIN_RX = 33;
    const int PIN_TX = 43, PIN_RX = 44;
#else
    const int PIN_TX = 1, PIN_RX = 3;
#endif

    const auto port = (uart_port_t) port_num;
    ESP_ERROR_CHECK(uart_set_pin(port, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(port, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, intr_alloc_flags));


    /* uart_intr_config_t uart_intr = {
         .intr_enable_mask = (0x1 << 0) | (0x8 << 0),  // UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
         .rx_timeout_thresh = 1,
         .txfifo_empty_intr_thresh = 10,
         .rxfifo_full_thresh = 112,
    };
    uart_intr_config((uart_port_t) 0, &uart_intr);  // Zero is the UART number for Arduino Serial
    */
}


// This example demonstrates how a human readable table of run time stats
// information is generated from raw data provided by uxTaskGetSystemState().
// The human readable table is written to pcWriteBuffer
void vTaskGetRunTimeStats() {
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    // Make sure the write buffer does not contain a string.
    //*pcWriteBuffer = 0x00;

    // Take a snapshot of the number of tasks in case it changes while this
    // function is executing.
    uxArraySize = uxTaskGetNumberOfTasks();

    // Allocate a TaskStatus_t structure for each task.  An array could be
    // allocated statically at compile time.
    pxTaskStatusArray = (TaskStatus_t *) pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        // Generate raw status information about each task.
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
        // For percentage calculations.
        ulTotalRunTime /= 100UL;

        // Avoid divide by zero errors.
        if (ulTotalRunTime > 0) {
            // For each populated position in the pxTaskStatusArray array,
            // format the raw data as human readable ASCII data
            for (x = 0; x < uxArraySize; x++)
                // What percentage of the total run time has the task used?
                // This will always be rounded down to the nearest integer.
                // ulTotalRunTimeDiv100 has already been divided by 100.
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

            auto aff = xTaskGetAffinity(pxTaskStatusArray[x].xHandle);

            if (ulStatsAsPercentage > 0UL) {
                printf("%s (core#%i)\t\t%lu\t\t%lu%%\r\n", pxTaskStatusArray[x].pcTaskName, aff,
                       pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage);
            } else {
                // If the percentage is zero here then the task has
                // consumed less than 1% of the total run time.
                printf("%s (core#%i)\t\t%lu\t\t1%%\r\n", pxTaskStatusArray[x].pcTaskName, aff,
                       pxTaskStatusArray[x].ulRunTimeCounter);
            }

            //pcWriteBuffer += strlen((char *) pcWriteBuffer);
        }
    }

    // The array is no longer needed, free the memory it consumes.
    vPortFree(pxTaskStatusArray);
}
