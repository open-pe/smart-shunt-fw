#pragma once

#include <EEPROM.h>

#ifdef TARGET_STM32H5
#include "esp_compat.h"
#include <cassert>

struct settings_t {
    int8_t Pin_I2C_SDA = PB_9;
    int8_t Pin_I2C_SCL = PB_8;
    uint8_t Pin_INA22x_ALERT = PA10;
    uint8_t Pin_INA22x_ALERT2 = PB3;
    uint8_t Pin_INA22x_ALERT3 = PC7;
    uint8_t Pin_Mux_S1 = PB5;
    uint8_t Pin_Mux_S2 = PB4;
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

#if CONFIG_IDF_TARGET_ESP32S3

#define XIAO_ESP32S3 1
#define WAVESHARE_MINI 0
#define GREEN_DAQ_BREAD_BOARD 0

struct settings_t {
#ifdef FMETAL
    uint8_t Pin_I2C_SDA = 42, Pin_I2C_SCL = 2, Pin_INA22x_ALERT = 41;
#elif XIAO_ESP32S3
    int8_t Pin_I2C_SDA = 5, Pin_I2C_SCL = 6;
    uint8_t Pin_INA22x_ALERT = 1;
    uint8_t Pin_INA22x_ALERT2 = 2;
    uint8_t Pin_INA22x_ALERT3 = 3;
    uint8_t Pin_Mux_S1 = 4;
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
#else
struct settings_t_01 {
    uint8_t Pin_I2C_SDA = 21, Pin_I2C_SCL = 22;
    uint8_t Pin_INA22x_ALERT = 19;
};
#endif

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
