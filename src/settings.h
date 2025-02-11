#pragma once

#include <EEPROM.h>

#if CONFIG_IDF_TARGET_ESP32S3
struct settings_t_01 {
    // uint8_t Pin_I2C_SDA = 42, Pin_I2C_SCL = 2; // fugu
    //uint8_t Pin_INA22x_ALERT = 40;
    //uint8_t Pin_INA22x_ALERT = 41; // fugu2

    uint8_t Pin_I2C_SDA = 15, Pin_I2C_SCL = 16;
    uint8_t Pin_INA22x_ALERT = 7;
    uint8_t Pin_INA22x_ALERT2 = 6;
    uint8_t Pin_INA22x_ALERT3 = 5;
};

struct settings_t {
    uint8_t Pin_I2C_SDA = 3, Pin_I2C_SCL = 2;
    uint8_t Pin_INA22x_ALERT = 1;
    uint8_t Pin_INA22x_ALERT2 = 4;
    uint8_t Pin_INA22x_ALERT3 = 5;

    uint8_t Pin_ADS1220_CS = 7;
    uint8_t Pin_ADS1220_DRDY = 6;


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
        ESP_LOGI("store", "Writing calib.facWriting calib.factors at %hhu: %.6f/%.6f (old: %.6f/%.6f)", ecIndex, u, i, oldU, oldI);
    }

    EEPROM.begin(256);
    // 16 bytes name
    // 32 bytes calibration data (4*2*4)
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 0, u);
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 4, i);
    EEPROM.commit();
    EEPROM.end();

    float ru, ri;
    assert(readCalibrationFactors(ecIndex, ru, ri));
    assert( abs(ru - u)/u < 1e-9);
    assert( abs(ri - i)/i < 1e-9);
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
    // 16 bytes name
    // 32 bytes calibration data (4*2*4)
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
