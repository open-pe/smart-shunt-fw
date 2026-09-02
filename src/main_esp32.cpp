#include <Arduino.h>
#include <Wire.h>

#include <InfluxDbClient.h>
#include <map>
#include <vector>

#include "adc/adc_ads.h"
#include "adc/adc_esp.h"
#include "adc/ina226.h"
#include "adc/ina228.h"
#include "adc/ina228_mux.h"
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
#include "USB.h"

#include "ble.h"
#include "platform.h"
#include "sampler_registry.h"
#include "console.h"
#include "telemetry.h"
#include "energy_counter.h"
#include "util.h"
#include "adc/tmp117.h"

InfluxDBClient client;

PowerSampler_ADS ads;
PowerSampler_ADS131 ads131;
PowerSampler_INA226 ina226;
PowerSampler_INA228 ina228_40{0x40};

/* Second INA228 on the XIAO harness, strapped to 0x43 (ADD0 -> SCL). The
 * derived storage slots do not work at this address -- 2 + 3 = 5 is TMP117
 * 0x48 and 8 + 3 = 11 is mux channel B -- so both are passed explicitly from
 * the free high range (see CALIB_SLOT_FIRST_HIGH in settings.h). It has no
 * ALERT pin (Pin_INA22x_ALERT4 = 255) and samples on the 50 ms register-read
 * fallback in hasData(). */
PowerSampler_INA228 ina228_43{0x43, /*calibSlot=*/19, /*resistorSlot=*/20};

#ifdef DUAL_INA228
/* The 0x41 part on the two-INA228 variant is a PLAIN INA228, not a mux backend --
 * there is no TMUX8612 on that board. Its derived slots (3 and 9) are exactly the
 * ones INA228MuxBackend would have used for channel A and its resistor/range, so
 * the two builds never both need them; a board reflashed between variants simply
 * re-reads its own calibration into the same slots. */
PowerSampler_INA228 ina228_41{0x41};
#endif

INA228MuxBackend muxBackend{0x41, settings.Pin_INA22x_ALERT2,
                            settings.Pin_Mux_S1, settings.Pin_Mux_S2, settings.Pin_Mux_Zero};
PowerSampler_MuxChannel mux_chA{muxBackend, INA228MuxBackend::Target::CH_A, 3};
PowerSampler_MuxChannel mux_chB{muxBackend, INA228MuxBackend::Target::CH_B, 11};

/* One per ADD0 strap (GND/V+/SDA/SCL).  All four are registered unconditionally
 * and SamplerRegistry::initAll() drops whichever do not answer -- init() reads
 * DEVICE_ID first and returns false on no response, so an absent part costs one
 * failed transaction at boot and logs "init failed".  That beats hardcoding the
 * populated addresses: rebuilding the firmware is not the right response to
 * moving a strap. */
PowerSampler_TMP117 tmp117{0x48};
PowerSampler_TMP117 tmp117_49{0x49};
PowerSampler_TMP117 tmp117_4a{0x4A};
PowerSampler_TMP117 tmp117_4b{0x4B};

#ifdef SHUNT_ADC_ONLY
/* A separate controller lets another default-address part coexist with bus 0;
 * storage slot 9 is unused by every sampler registered in this build. */
static constexpr uint8_t TMP117_I2C2_SDA = 3;
static constexpr uint8_t TMP117_I2C2_SCL = 8;
static constexpr uint8_t TMP117_I2C2_STORAGE_ID = 9;
PowerSampler_TMP117 tmp117_i2c2{0x48, 1, TMP117_I2C2_STORAGE_ID};
#endif

/* pwr-metering shunt-adc board (ADS1262) on J2. Pins are the wiring, not
 * settings.h, because settings.h has no SPI entries for this board.
 *
 * WARNING: on the XIAO these five collide with the INA228 alerts and the mux
 * (GPIO4=ALERT2, 5=ALERT3, 6=Mux_S1, 7=Mux_S2). This wiring is the N16R8 clone
 * bring-up map; pick one or the other per board before enabling both. */
static constexpr int8_t SHUNTADC_SCLK = 4, SHUNTADC_DIN = 5, SHUNTADC_DOUT = 6,
                        SHUNTADC_NCS = 7, SHUNTADC_START = 16;

Ads1262ShuntAdc shuntAdc;
/* Shunt current on J3 (IIN, PAIR_CH0). uPair = PAIR_NONE means no voltage
 * channel: u carries the raw shunt volts and p is pinned to 0. Give it a uPair
 * (J5 = PAIR_CH2, the old wiring) once a divider is actually fitted there, and
 * set dividerRatio at the same time.
 *
 * NO LONGER PARKED: with shuntAdcDcct below also registered, the device scans
 * CH0 <-> CH1. Every pair switch restarts the conversion cycle (td(STDR)
 * doubled to ~104 ms by chop), so EACH channel updates at ~4.8 SPS -- not the
 * 19.15 SPS the mux-parked configuration delivered. Still well above the
 * ~2.5 SPS on-air target.
 *
 * Each published sample is no noisier: it is still ONE conversion at the same
 * gain and filter. What degrades is the uncertainty of any FIXED-DURATION
 * average, by ~2x -- four times fewer CH0 conversions arrive per second, and
 * averaging error falls as sqrt(N). Comment out the DCCT registration below and
 * this channel parks again automatically (the device parks whenever exactly one
 * pair is registered).
 *
 * shuntOhm 2 mOhm: sized for 14 A => 28 mV, 80% of the 35 mV input window note N8
 * derives from G=32, leaving headroom for transients. Nominal, NOT calibrated --
 * gain error is whatever the resistor's tolerance is until it is measured against
 * a reference.
 *
 * storageId 12 continues past the existing samplers. */
PowerSampler_ShuntAdc shuntAdcIn{shuntAdc,
                                 Ads1262ShuntAdc::PAIR_NONE, Ads1262ShuntAdc::PAIR_CH0,
                                 /* shuntOhm = 2 mOhm nominal; dividerRatio 0 = no voltage
                                  * channel, so u stays raw ADC volts. */
                                 0.002f, 0.0f,
                                 /* storageId is a namespace shared by EVERY registered
                                  * sampler -- it indexes the EEPROM calibration slot
                                  * (settings.h: 16 + id*8). Taken: 0 (ADS/ADS131),
                                  * 2 (INA228 0x40), 3/11 (mux A/B), 5/6 (TMP117 0x48/0x49,
                                  * see STORAGE_ID_BASE in tmp117.h), 15 (Dummy). 5 aliased
                                  * TMP117 and would have silently shared its calibration. */
                                 12,
                                 SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                 SHUNTADC_NCS, SHUNTADC_START,
                                 /* reportDieTemp: DCCT below shares this ADS1262 but
                                  * deliberately opts out, so this remains the one
                                  * channel carrying the single die sensor -- and it is the
                                  * x-axis for the offset drift the shunt sizing
                                  * depends on. T read nan before this was set. */
                                 true};

/* Offset-drift characterisation: the ADC's INTERNAL short with chop OFF,
 * streamed through the normal sampler path. MUTUALLY EXCLUSIVE with shuntAdcIn
 * -- it parks the ADC in a different configuration -- so exactly one of the two
 * samplers.add() lines below may be active. 16 conversions per point at sinc4
 * 10 SPS, so ~1.6 s per point plus a one-time 400 ms settle (was ~0.8 s on the
 * production FIR @ 20 SPS; see ZERO_MODE1_VALUE in adc/ads1262.h). */
/* storageId 13: the ID indexes the shared EEPROM calibration slot
 * (settings.h: 16 + id*8). 6 aliased tmp117_49 -- tmp117.h computes
 * STORAGE_ID_BASE(5) + (0x49 - ADDR_MIN(0x48)) = 6 -- so both samplers would have
 * silently overwritten each other's calibration. Taken: 0, 2, 3, 5, 6, 11, 12, 15. */
/* avgN = 4: at ZERO_DATA_RATE_CODE (10 SPS) this is 400 ms per published
 * sample, i.e. the 2.5 SPS target. avgN = 16 was 1.6 s -> 0.625 SPS, which is
 * exactly the rate measured on the bench. Averaging deeper here also duplicates
 * work EnergyCounter/MeanWindow already does downstream; what the device-level
 * average buys is a sigma estimate over conversions taken in ONE undisturbed
 * configuration, and 4 is enough for that. The mean's uncertainty goes from
 * sd/4 to sd/2 -- about 3 nV to 7 nV at the measured 13 nV per-conversion
 * sigma, against a budget of 350 nV (10 ppm of 35 mV). */
/* DCCT primary current on J4 (PAIR_CH1): a 500:N current transformer into a
 * 5 Ohm burden resistor across AIN2(+)/AIN3(-). V_burden = I_primary * N/100,
 * so the envelope (2..40 A primary, N = 1..6 turns) spans 20 mV .. 2.4 V. At
 * G=1 the PGA's own output swing (Equation 12) still limits each input pin to
 * roughly -2.2..+2.2 V on these +-2.5 V rails, which a ground-referenced 2.4 V
 * peak clears by ~0.2 V, so this pair runs GAIN 1 with the PGA BYPASSED:
 * rail-to-rail input, +-VREF = +-2.5 V digital full scale. 2.4 V is 96% of FS;
 * prefer a turns
 * count with I*N <= ~200 A-turns when the setup allows headroom. Over-range is
 * ALSO policed digitally (DIAG_PGA_RANGE): the PGA monitors watch the PGA
 * output, which a bypassed signal never passes through, so they cannot be relied
 * on here -- though they are demonstrably not dead in bypass either, a floating
 * J4 having raised PGAH continuously on 2026-08-25.
 * G=1 noise (~1 uV/conversion) is ~50 ppm of the worst-case 20 mV
 * signal -- the DCCT ratio error and the burden's tolerance/tempco dominate.
 * Burden dissipation reaches (40*6/500)^2 * 5 = 1.15 W: fit a >= 3 W low-tempco
 * part, it is the metrology element.
 *
 * TIE THE BURDEN COLD END TO BOARD GND -- not optional. Without a DC path to the
 * board's reference the pair floats out of the +-2.6 V absolute input window,
 * and a CH1 driven HARD out of range corrupts its NEIGHBOUR: on 2026-08-25 it
 * put a 7.2 mV offset on CH0 (3.6 A of phantom shunt current) and made the
 * internal die-temperature read return 1503 degC, because the mux now visits
 * both pairs and the front end had not recovered by the next conversion. A
 * merely FLOATING CH1 does not do this -- only a driven one.
 *
 * FIELD MAPPING (same convention as the other overloaded channels): u is the
 * raw burden voltage in VOLTS at the ADC input (dividerRatio 0 = unscaled,
 * doubles as a range readout), i is the PRIMARY current in AMPERES, p = 0.
 *
 * shuntOhm 0.01 is the N=1 effective transresistance (5 Ohm * 1/500). The
 * turns ratio is applied at runtime through the EXISTING per-sampler
 * calibration: after re-rigging, `calibrate DCCT I <1/N>` on the console
 * (0.5 for N=2 ... 0.1667 for N=6; EEPROM-persisted). i is only correct for
 * the rigged N -- and note a trim calibration lives in the SAME factor, so
 * changing N overwrites trim.
 *
 * storageId 10: taken are 0 (ADS/ADS131), 1 (INA226), 2 (INA228 0x40),
 * 3/11 (mux A/B), 5/6/7 (TMP117 0x48-0x4A), 16 (TMP117 0x4B), 9 (SHUNT_ADC_ONLY
 * TMP117), 12/13/14 (shunt/zero/health), 15 (Dummy), 19/20 (INA228 0x43
 * calib + resistor/range, assigned explicitly -- see ina228_43).
 *
 * Slot 8 is NOT free despite no getStorageId() returning it: PowerSampler_INA228
 * is a SECOND consumer of this same EEPROM namespace, storing resistor/range at
 * 8 + (addr - I2C_A0) (ina228.h:186) independently of its own storageId of 2.
 * Grepping getStorageId() alone therefore understates what is taken -- 10 was
 * checked against both. */
PowerSampler_ShuntAdc shuntAdcDcct{shuntAdc,
                                   Ads1262ShuntAdc::PAIR_NONE, Ads1262ShuntAdc::PAIR_CH1,
                                   0.01f, 0.0f,
                                   10,
                                   SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                   SHUNTADC_NCS, SHUNTADC_START,
                                   /* reportDieTemp: CH0 already carries it */ false,
                                   /* pgaGainCode */ 0, /* pgaBypass */ true};

PowerSampler_ShuntAdcZero shuntAdcZero{shuntAdc, 4, 13,
                                       SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                       SHUNTADC_NCS, SHUNTADC_START};

/* Front-end health as a recorded series: u = fCLK (Hz), i = AVDD-AVSS (V).
 * Publishes cached values only -- it drives no conversions of its own, so it is
 * free to register alongside whichever measurement channel is active. 30 s is
 * far faster than either quantity can meaningfully move, and the failure this
 * exists to catch ran for 35 minutes before the board died.
 * storageId 14: taken are 0, 2, 3, 5, 6, 11, 12, 13, 15. */
PowerSampler_ShuntAdcHealth shuntAdcHealth{shuntAdc, 30000, 14,
                                           SHUNTADC_SCLK, SHUNTADC_DIN, SHUNTADC_DOUT,
                                           SHUNTADC_NCS, SHUNTADC_START};

SamplerRegistry samplers;
BleSrv bleSrv;

volatile bool g_samplingHalted = false;
bool disableWifi = true;
bool wifiTimeSyncOnly = true;

class EspTelemetry : public Telemetry {
public:
    using Telemetry::Telemetry;

private:
    std::vector<WireSample> wireSampleBuf;

public:
    void onIdleSleep() override {
        UART_LOG("Zero power for %llds, sleeping for %llds (aux=%s)",
                 (long long)(IDLE_SLEEP_AFTER_US / 1000000),
                 (long long)(IDLE_SLEEP_WAKE_US / 1000000), auxGet() ? "ON" : "off");
        Serial.flush();
        auxArmDeepSleepHold();
        ESP.deepSleep(IDLE_SLEEP_WAKE_US);
    }

    void onSummary(const WireSample &ws) override {
        if (!disableWifi) wireSampleBuf.push_back(ws);
    }

    void onTelemetryFlush() override {
        if (!disableWifi) {
            for (auto &ws : wireSampleBuf)
                influxWritePointUDP(ws.getInfluxDbPoint());
        }
        wireSampleBuf.clear();
    }

    void processConsole() override {
        const uart_port_t uart_num = UART_NUM_0;
        static char buf[128];
        static int buf_pos = 0;
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *) &length));
        if (length > 0) {
            length = uart_read_bytes(uart_num, buf + buf_pos, min(length, 127 - buf_pos), 20);
            uart_write_bytes(uart_num, buf + buf_pos, length);
            buf_pos += length;
        }
        auto eol = strchr(buf, '\n');
        while (eol) {
            *eol = 0;
            buf_pos = 0;
            handleConsoleInput(String(buf), registry, ble);
            noteWakeEvent();
            break;
        }

        static String serialBuf;
        if (Serial.available()) {
            auto r = Serial.readString();
            serialBuf += r;
            Serial.print(r);
            Serial.flush();
            int lb;
            while ((lb = serialBuf.indexOf('\n')) != -1) {
                String line = serialBuf.substring(0, lb);
                handleConsoleInput(line, registry, ble);
                serialBuf = serialBuf.substring(lb + 1);
            }
            noteWakeEvent();
        }
    }
};

EspTelemetry *telemetry = nullptr;

#ifdef WITH_LCD
LCD lcd;
#endif

static constexpr uint32_t OTA_VALIDATE_UPTIME_MS = 60000;
static constexpr uint64_t BOOT_WATCHDOG_TIMEOUT_US = 30ull * 1000000ull;

extern "C" bool verifyRollbackLater() { return true; }

static esp_timer_handle_t bootWatchdog = nullptr;

static void bootWatchdogFire(void *) { esp_restart(); }

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

static void markOtaValidIfHealthy() {
    static bool done = false;
    if (done) return;
    unsigned long total = 0;
    for (auto &ec: samplers.counters) total += ec.numSamples();
    static unsigned long snapshot = 0;
    static bool haveSnapshot = false;
    if (!haveSnapshot) {
        if (millis() < OTA_VALIDATE_UPTIME_MS / 2) return;
        snapshot = total;
        haveSnapshot = true;
        return;
    }
    if (millis() < OTA_VALIDATE_UPTIME_MS) return;
    if (total <= snapshot) return;
    done = true;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
            ESP_LOGI("main", "OTA image confirmed valid (rollback cancelled)");
        else
            ESP_LOGE("main", "failed to confirm OTA image valid -- a reset will roll back");
    }
}

#define RT_CORE 1
constexpr auto RT_PRIO = 20;

[[noreturn]] void realTimeTask(void *arg);
[[noreturn]] void appTask(void *arg);

void uartInit(int port_num);

void setup(void) {
    bootWatchdogArm();
    Serial.begin(115200);
    auxBegin();
    uartInit(0);
    delay(500);

    ESP_LOGI("main", "SmartShunt ESP32-S3 started");

    // Before initAll(): the prefix is baked into each EnergyCounter's name there.
    boardPrefixLoad();
    boardPrefixLogState();

    /* SHUNT_ADC_ONLY: the ADS1262 bring-up board on an OTRONIC ESP32-S3 N16R8 clone.
     * settings.h hardcodes `#define XIAO_ESP32S3 1`, so settings_t always carries
     * the XIAO pin map -- where I2C is GPIO3/2 and the INA228 alerts and mux sit
     * on GPIO4/5/6/7, exactly the pins the ADS1262 SPI uses here. There are no
     * INA228s on the clone, so the mux/INA side stays skipped -- but there ARE
     * TMP117s, so settings.h now carries a SHUNT_ADC_ONLY pin branch (I2C on
     * GPIO11/12, alerts and mux at 255) and the bus is brought up on both builds. */

    /* Physical-layer check while the pins are still plain GPIOs. Not fatal: on the
     * shunt-adc board the ADS1262 is on SPI and must come up even with the I2C
     * harness unplugged. See i2c_check_pins() for what it can and cannot see. */
    i2c_check_pins(settings.Pin_I2C_SDA, settings.Pin_I2C_SCL);

    Wire.begin(settings.Pin_I2C_SDA, settings.Pin_I2C_SCL, settings.I2C_Freq);

#ifdef SHUNT_ADC_ONLY
    i2c_check_pins(TMP117_I2C2_SDA, TMP117_I2C2_SCL);
    Wire1.begin(TMP117_I2C2_SDA, TMP117_I2C2_SCL, settings.I2C_Freq);
#endif

    if (!disableWifi) {
        connect_wifi_async_once();
        wait_for_wifi();
        timeSync("CET-1CEST,M3.5.0,M10.5.0/3", "de.pool.ntp.org", "time.nis.gov");
        if (wifiTimeSyncOnly) {
            WiFi.disconnect(true);
            WiFiClass::mode(WIFI_OFF);
            disableWifi = true;
            g_wifiRequest = 0;
        } else {
            g_wifiRequest = 1;
        }
    }

    bleSrv.begin();

    if (sizeof(WireSample) != 64) assert(false);
    if (sizeof(Sample) != 32) assert(false);

#ifdef DUAL_INA228
    /* Two plain parts, no mux, no 0x40. The names keep the series this harness has
     * always published: 0x41 is _2, and 0x43 takes _4 (the retired _3 was the 0x42
     * strap). Registering the mux channels here would be worse than useless -- they
     * would drive D8/D9, which carry this board's vbus-only jumper. */
    samplers.add("ESP32_INA228_2", &ina228_41);
    samplers.add("ESP32_INA228_4", &ina228_43);
#elif !defined(SHUNT_ADC_ONLY)
    samplers.add("ESP32_INA228", &ina228_40);
    samplers.add("ESP32_INA228_4", &ina228_43);
    samplers.add("ESP32_INA228_2A", &mux_chA);
    samplers.add("ESP32_INA228_2B", &mux_chB);
#endif
    /* Outside the SHUNT_ADC_ONLY guard: the N16R8 clone carries TMP117s even though it
     * carries no INA228s.
     *
     * The name is the InfluxDB series key (the collector tags points with
     * "BLE_" + this name), so it has to be unique across every board reporting into
     * the same database -- not just within one board. The XIAO already publishes
     * BLE_TMP117 and BLE_TMP117_2, so the clone's parts take an SA_ (shunt-adc)
     * prefix; the two boards can then never collide even if they populate the same
     * addresses. The XIAO keeps its historical names, so no existing series forks.
     *
     * Address-suffixed rather than ordinal: with parts appearing and disappearing by
     * strap, "TMP117_3" is not stable across boots, whereas the address is the one
     * name that always identifies the same physical part.
     *
     * The board half of the identity is NOT here: SamplerRegistry prepends the
     * NVS-stored board prefix (board_prefix.h), so the same binary gives each unit
     * distinct series without a per-unit build.
     *
     * LENGTH: WireSample::dev is 16 bytes and name.copy(ws.dev, 16) TRUNCATES, so
     * two names differing only past character 16 become one series. "t17" rather
     * than "TMP117" buys the room for a prefix: "ftr_t17_48" is 10. Registration
     * checks the composed names for exactly this, so a bad prefix is reported at
     * boot rather than discovered in the database. */
    samplers.add("t17_48", &tmp117);
    samplers.add("t17_49", &tmp117_49);
    samplers.add("t17_4A", &tmp117_4a);
    samplers.add("t17_4B", &tmp117_4b);
#ifdef SHUNT_ADC_ONLY
    samplers.add("t17b_48", &tmp117_i2c2);
#endif
    /* ONE of these two, never both: they share the ADS1262 and configure it
     * differently. SHUNT_ADC_ZERO is the offset-drift characterisation run
     * (internal short, chop ON since 2026-08-15); SHUNT_ADC is normal measurement. */
    /* The bridge prefixes the sampler name with "BLE_", so these appear as
     * BLE_SHUNT_ADC and BLE_SHUNT_ADC_ZERO. Extra channels: SHUNT_ADC_ch1, ... */
    samplers.add("SHUNT_ADC", &shuntAdcIn);
    /* dev[16] truncation: "ftr_DCCT" fits whole; a SHUNT_ADC_-prefixed name
     * would collide with SHUNT_ADC's truncation in InfluxDB. */
    samplers.add("DCCT", &shuntAdcDcct);
    //samplers.add("SHUNT_ADC_ZERO", &shuntAdcZero);
    //samplers.add("SHUNT_ADC_HEALTH", &shuntAdcHealth);  // fCLK/AVDD diagnostic series; disabled on request

/* DUAL_INA228 must be excluded here, not just from samplers.add(): the backend
 * talks to 0x41, which on that variant is ina228_41's OWN device. Leaving this
 * call in gave the part two drivers -- the backend rewrote its DIAG_ALRT and
 * SHUNT_CAL and attached a second ISR to Pin_INA22x_ALERT2, the same GPIO4 the
 * plain sampler uses -- and drove pinMode(255) three times over, which the HAL
 * logs as "Invalid IO 255 selected" on every boot. */
#if !defined(SHUNT_ADC_ONLY) && !defined(DUAL_INA228)
    if (!muxBackend.init()) {
        ESP_LOGW("main", "INA228 mux backend (0x41) init failed");
    }
#endif

    samplers.initAll();
    samplers.startAll();

    if (!disableWifi) {
        client.setConnectionParamsV1("http://homeassistant.local:8086", "open_pe", "home_assistant", "h0me");
        client.setWriteOptions(WriteOptions()
            .writePrecision(WritePrecision::MS)
            .batchSize(200).bufferSize(400)
            .flushInterval(1).retryInterval(0));
    }

    telemetry = new EspTelemetry(samplers, bleSrv);
    telemetry->noteWakeEvent();

    xTaskCreatePinnedToCore(realTimeTask, "loopRt", 4096 * 4, NULL, RT_PRIO, NULL, RT_CORE);
    xTaskCreatePinnedToCore(appTask, "loopy", 4096 * 4, NULL, 1, NULL, RT_CORE - 1);
    if (xTaskCreatePinnedToCore(wifiTask, "wifi", 4096, NULL, 0, NULL, RT_CORE - 1) != pdPASS) {
        ESP_LOGE("main", "wifiTask creation failed");
    }

    bootWatchdogDisarm();
}

[[noreturn]] void realTimeTask(void *arg) {
    (void)arg;
    while (true) {
        if (g_samplingHalted) {
            vTaskDelay(10);
            continue;
        }
        samplers.updateAll();
    }
}

void loop() { vTaskDelay(1000); }

[[noreturn]] void appTask(void *arg) {
    (void)arg;
    while (1) {
        wifi_tick();
        otaBleTick(millis());
        telemetry->update(false);
        if (otaBleActive()) {
            telemetry->noteWakeEvent();
        } else {
            telemetry->checkIdleSleep();
        }
        markOtaValidIfHealthy();
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
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
#if CONFIG_IDF_TARGET_ESP32S3
    const int PIN_TX = 43, PIN_RX = 44;
#else
    const int PIN_TX = 1, PIN_RX = 3;
#endif
    const auto port = (uart_port_t) port_num;
    ESP_ERROR_CHECK(uart_set_pin(port, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(port, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, 0));
}
