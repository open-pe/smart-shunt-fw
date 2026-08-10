#include <Arduino.h>
#include <Wire.h>

#include "platform.h"
#include "sampler_registry.h"
#include "console.h"
#include "telemetry.h"
#include "ble_transport.h"
#include "settings.h"
#include "i2c.h"
#include "aux_switch.h"
#include "util.h"
#include "energy_counter.h"
#include "adc/ina228.h"
#include "adc/ina228_mux.h"
#include "adc/tmp117.h"

PowerSampler_INA228 ina228_40{0x40};
INA228MuxBackend muxBackend{0x41, settings.Pin_INA22x_ALERT2,
                            settings.Pin_Mux_S1, settings.Pin_Mux_S2, settings.Pin_Mux_Zero};
PowerSampler_MuxChannel mux_chA{muxBackend, INA228MuxBackend::Target::CH_A, 3};
PowerSampler_MuxChannel mux_chB{muxBackend, INA228MuxBackend::Target::CH_B, 11};
PowerSampler_TMP117 tmp117{0x48};
PowerSampler_TMP117 tmp117_49{0x49};

SamplerRegistry samplers;
StubBleTransport bleTransport;
Telemetry telemetry(samplers, bleTransport);

[[noreturn]] void realTimeTask(void *arg);
[[noreturn]] void appTask(void *arg);

constexpr auto RT_PRIO = 20;

void setup(void) {
    Serial.begin(115200);
    delay(500);
    auxBegin();

    ESP_LOGI("main", "SmartShunt STM32H5 started");

    Wire.begin();
    Wire.setClock(400000);

    bleTransport.begin();

    if (sizeof(WireSample) != 64) assert(false);
    if (sizeof(Sample) != 32) assert(false);

    samplers.add("INA228", &ina228_40);
    samplers.add("INA228_2A", &mux_chA);
    samplers.add("INA228_2B", &mux_chB);
    samplers.add("TMP117", &tmp117);
    samplers.add("TMP117_2", &tmp117_49);

    if (!muxBackend.init()) {
        ESP_LOGW("main", "INA228 mux backend (0x41) init failed");
    }

    samplers.initAll();
    samplers.startAll();

    telemetry.noteWakeEvent();

    platform::createRealTimeTask(realTimeTask, "rt", 1024, RT_PRIO);
    platform::createAppTask(appTask, "nrt", 1024, 1);
    vTaskStartScheduler();
}

[[noreturn]] void realTimeTask(void *arg) {
    (void)arg;
    while (true) {
        samplers.updateAll();
        vTaskDelay(1);
    }
}

void loop() { vTaskDelay(1000); }

[[noreturn]] void appTask(void *arg) {
    (void)arg;
    while (1) {
        telemetry.update();
        vTaskDelay(10);
    }
}
