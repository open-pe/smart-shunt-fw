#pragma once

#include <Arduino.h>
#include "sampler_registry.h"
#include "ble_transport.h"
#include "aux_switch.h"
#include "settings.h"
#include "util.h"
#include "adc/ina228.h"
#include "adc/ina228_mux.h"

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
        } else if (inp == "help") {
            UART_LOG("ina22x-resistor-range <resistance> <max expected current>");
            UART_LOG("calibrate <sampler> <U|I> [*]<factor>");
            UART_LOG("aux [on|off|toggle]");
            UART_LOG("reset   zero the energy counters");
        } else {
            UART_LOG("Unknown command '%s'. enter 'help' for help", inp.c_str());
        }
        break;
    }
}
