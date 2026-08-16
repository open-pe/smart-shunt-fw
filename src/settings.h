#pragma once

#include <EEPROM.h>

#ifdef TARGET_STM32H5
#include "esp_compat.h"
#include <cassert>

struct settings_t {
    int8_t Pin_I2C_SDA = PB_4;
    int8_t Pin_I2C_SCL = PB_5;
    uint8_t Pin_INA22x_ALERT = PA10;
    uint8_t Pin_INA22x_ALERT2 = PC7;
    uint8_t Pin_INA22x_ALERT3 = PC6;
    uint8_t Pin_Mux_S1 = PC9;
    uint8_t Pin_Mux_S2 = PC8;
    uint8_t Pin_Mux_Zero = PB10;

    uint8_t Pin_ADS1220_CS = 0;
    uint8_t Pin_ADS1220_DRDY = 0;
    uint8_t Pin_ADS1262_START = 0;
    uint8_t Pin_ADS1262_PWDN = 0;
    uint8_t Pin_ADS131_Clk = 0;
    uint8_t Pin_ADS131_Miso = 0;
    uint8_t Pin_ADS131_Mosi = 0;
    uint8_t Pin_ADS131_Cs = 0;
    uint8_t Pin_ADS131_CsClk = 0;
    uint8_t Pin_ADS131_Drdy = 0;
    uint8_t Pin_ADS131_Rst = 0;
};

static settings_t settings;

static bool readCalibrationFactors(size_t ecIndex, float &u, float &i);

static void storeCalibrationFactors(uint8_t ecIndex, float u, float i) {
    assert(ecIndex <= 16);
    EEPROM.put(16 + ecIndex * 4 * 2 + 0, u);
    EEPROM.put(16 + ecIndex * 4 * 2 + 4, i);
}

static bool checkCalibrationFactorBounds(float f) {
    if (!std::isnormal(f))
        return false;
    if (std::abs(f) < (1 / 10e4)) {
        return false;
    }
    return f > -10e3 and f < 10e3;
}

static bool readCalibrationFactors(size_t ecIndex, float &u, float &i) {
    assert(ecIndex <= 16);
    EEPROM.get(16 + ecIndex * 4 * 2 + 0, u);
    EEPROM.get(16 + ecIndex * 4 * 2 + 4, i);
    return checkCalibrationFactorBounds(u) and checkCalibrationFactorBounds(i);
}

#else

#define XIAO_ESP32S3 1
#define WAVESHARE_MINI 0
#define GREEN_DAQ_BREAD_BOARD 0

struct settings_t {
#ifdef SHUNT_ADC_ONLY
    /* OTRONIC ESP32-S3 N16R8 clone carrying the pwr-metering shunt-adc board. This
     * branch exists because `#define XIAO_ESP32S3 1` above is unconditional, so
     * without it this clone build silently inherits the XIAO map -- I2C on GPIO3/2
     * and the alerts/mux on GPIO4/5/6/7, which are the ADS1262 SPI pins here.
     *
     * I2C is GPIO11/12. NOT 9/10: GPIO9 is AUX_PIN
     * (aux_switch.h), driven push-pull by auxBegin() before Wire.begin() and held
     * through deep sleep, which would sit on SDA.  Clear of the ADS1262 wiring
     * (SHUNTADC_* in main_esp32.cpp: 4/5/6/7/16) as well.
     *
     * The INA228/mux pins are 255 rather than a plausible GPIO: there are no
     * INA228s on this board, PowerSampler_INA228/INA226 are never registered under
     * SHUNT_ADC_ONLY so their init() never runs pinMode on these, and a real number
     * here would be a loaded gun pointed at the ADS1262 SPI bus if one ever did.
     *
     * GPIO11/12 do collide with Pin_ADS131_Mosi/Miso below -- those live outside the
     * board branch, so they apply here too.  Inert today only because main_esp32.cpp
     * never registers `ads131` (or `ads`), so their init() never runs pinMode.  If
     * either is ever added to samplers, repin it or repin I2C. */
    int8_t Pin_I2C_SDA = 11, Pin_I2C_SCL = 12;
    uint8_t Pin_INA22x_ALERT = 255;
    uint8_t Pin_INA22x_ALERT2 = 255;
    uint8_t Pin_INA22x_ALERT3 = 255;
    uint8_t Pin_Mux_S1 = 255;
    uint8_t Pin_Mux_S2 = 255;
    uint8_t Pin_Mux_Zero = 255;
#elif defined(FMETAL)
    uint8_t Pin_I2C_SDA = 42, Pin_I2C_SCL = 2, Pin_INA22x_ALERT = 41; // fugu2 (fmetal)
#elif XIAO_ESP32S3
    int8_t Pin_I2C_SDA = 3, Pin_I2C_SCL = 2;
    uint8_t Pin_INA22x_ALERT = 1;
    uint8_t Pin_INA22x_ALERT2 = 4;
    uint8_t Pin_INA22x_ALERT3 = 5;
    uint8_t Pin_Mux_S1 = 6;
    uint8_t Pin_Mux_S2 = 7;
    uint8_t Pin_Mux_Zero = 8;
#elif WAVESHARE_MINI
    int8_t Pin_I2C_SDA = 3, Pin_I2C_SCL = 2;
    uint8_t Pin_INA22x_ALERT = 1;
    uint8_t Pin_INA22x_ALERT2 = 4;
    uint8_t Pin_INA22x_ALERT3 = 5;
    uint8_t Pin_Mux_S1 = 5;
    uint8_t Pin_Mux_S2 = 6;
    uint8_t Pin_Mux_Zero = 9;
#elif GREEN_DAQ_BREAD_BOARD
    int8_t Pin_I2C_SDA = 42, Pin_I2C_SCL = 38;
    uint8_t Pin_INA22x_ALERT = 40;
    uint8_t Pin_INA22x_ALERT2 = 0;
    uint8_t Pin_INA22x_ALERT3 = 0;
#else
    int8_t Pin_I2C_SDA = 40, Pin_I2C_SCL = 41;
    uint8_t Pin_INA22x_ALERT = 42;
    uint8_t Pin_INA22x_ALERT2 = 21;
    uint8_t Pin_INA22x_ALERT3 = 47;
#endif

    /* Bus clock. 400 kHz needs real pull-ups; the boards with INA228s have them.
     *
     * The shunt-adc harness has NONE -- it leans on the ESP32-S3's internal ones,
     * which arduino-esp32 enables unconditionally (esp32-hal-i2c-ng.c:131) but which
     * are only ~45 kOhm. Against the ~50 pF of a short harness that is a ~2 us rise
     * time, against a 300 ns budget at 400 kHz: SDA never reaches a valid high, and
     * the IDF driver reports the bus as stuck (ESP_ERR_INVALID_STATE, 259) rather
     * than as a NACK -- which is exactly how this first presented.
     *
     * 100 kHz allows 1000 ns, so this is still outside spec on paper and works only
     * because the real harness capacitance is well under the 400 pF the spec budgets
     * for. It is a bench expedient, not a fix: two 4.7 kOhm resistors to 3V3 would
     * make the bus compliant and allow 400 kHz back. If reads are flaky, suspect
     * this before suspecting the sensors. */
#ifdef SHUNT_ADC_ONLY
    uint32_t I2C_Freq = 100000;
#else
    uint32_t I2C_Freq = 400000;
#endif

    uint8_t Pin_ADS1220_CS = 7;
    uint8_t Pin_ADS1220_DRDY = 6;
    uint8_t Pin_ADS1262_START = 8;
    uint8_t Pin_ADS1262_PWDN = 9;

    uint8_t Pin_ADS131_Clk = 13;
    uint8_t Pin_ADS131_Miso = 12;
    uint8_t Pin_ADS131_Mosi = 11;
    uint8_t Pin_ADS131_Cs = 7;
    uint8_t Pin_ADS131_CsClk = 5;
    uint8_t Pin_ADS131_Drdy = 6;
    uint8_t Pin_ADS131_Rst = 9;
};

static settings_t settings;

static bool readCalibrationFactors(size_t ecIndex, float &u, float &i);

static void storeCalibrationFactors(uint8_t ecIndex, float u, float i) {
    assert(ecIndex <= 16);

    float oldU, oldI;
    if (readCalibrationFactors(ecIndex, oldU, oldI)) {
        ESP_LOGI("store", "Writing calib.factors at %hhu: %.6f/%.6f (old: %.6f/%.6f)", ecIndex, u, i,
                 oldU, oldI);
    }

    EEPROM.begin(256);
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 0, u);
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 4, i);
    EEPROM.commit();
    EEPROM.end();

    float ru, ri;
    assert(readCalibrationFactors(ecIndex, ru, ri));
    assert(abs(ru - u)/u < 1e-9);
    assert(abs(ri - i)/i < 1e-9);
}

static bool checkCalibrationFactorBounds(float f) {
    if (!std::isnormal(f))
        return false;

    if (std::abs(f) < (1 / 10e4)) {
        return false;
    }
    return f > -10e3 and f < 10e3;
}

static bool readCalibrationFactors(size_t ecIndex, float &u, float &i) {
    assert(ecIndex <= 16);
    EEPROM.begin(256);
    auto u_ = EEPROM.readFloat(16 + ecIndex * 4 * 2 + 0);
    auto i_ = EEPROM.readFloat(16 + ecIndex * 4 * 2 + 4);
    EEPROM.end();

    if ((checkCalibrationFactorBounds(u_) and checkCalibrationFactorBounds(i_))) {
        u = u_;
        i = i_;
        return true;
    }
    return false;
}

#endif
