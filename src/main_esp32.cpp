#include <Arduino.h>
#include <Wire.h>

#include <InfluxDbClient.h>
#include <map>

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

INA228MuxBackend muxBackend{0x41, settings.Pin_INA22x_ALERT2,
                            settings.Pin_Mux_S1, settings.Pin_Mux_S2, settings.Pin_Mux_Zero};
PowerSampler_MuxChannel mux_chA{muxBackend, INA228MuxBackend::Target::CH_A, 3};
PowerSampler_MuxChannel mux_chB{muxBackend, INA228MuxBackend::Target::CH_B, 11};

PowerSampler_TMP117 tmp117{0x48};
PowerSampler_TMP117 tmp117_49{0x49};

SamplerRegistry samplers;
BleSrv bleSrv;

class EspTelemetry : public Telemetry {
public:
    using Telemetry::Telemetry;
    void onIdleSleep() override {
        UART_LOG("Zero power for %llds, sleeping for %llds (aux=%s)",
                 (long long)(IDLE_SLEEP_AFTER_US / 1000000),
                 (long long)(IDLE_SLEEP_WAKE_US / 1000000), auxGet() ? "ON" : "off");
        Serial.flush();
        auxArmDeepSleepHold();
        ESP.deepSleep(IDLE_SLEEP_WAKE_US);
    }
};

EspTelemetry *telemetry = nullptr;

volatile bool g_samplingHalted = false;
bool disableWifi = true;
bool wifiTimeSyncOnly = true;

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

    Wire.begin(settings.Pin_I2C_SDA, settings.Pin_I2C_SCL, 400000UL);

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

    if (sizeof(WireSample) != 64) assert(false);
    if (sizeof(Sample) != 32) assert(false);

    samplers.add("ESP32_INA228", &ina228_40);
    samplers.add("ESP32_INA228_2A", &mux_chA);
    samplers.add("ESP32_INA228_2B", &mux_chB);
    samplers.add("TMP117", &tmp117);
    samplers.add("TMP117_2", &tmp117_49);

    if (!muxBackend.init()) {
        ESP_LOGW("main", "INA228 mux backend (0x41) init failed");
    }

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
        otaBleTick(millis());
        telemetry->update();
        if (otaBleActive()) telemetry->noteWakeEvent();
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
