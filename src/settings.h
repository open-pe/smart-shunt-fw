#pragma once

#include <EEPROM.h>

#if CONFIG_IDF_TARGET_ESP32S3
struct settings_t {
    // uint8_t Pin_I2C_SDA = 42, Pin_I2C_SCL = 2; // fugu
    //uint8_t Pin_INA22x_ALERT = 40;
    //uint8_t Pin_INA22x_ALERT = 41; // fugu2

    uint8_t Pin_I2C_SDA = 15, Pin_I2C_SCL = 16;
    uint8_t Pin_INA22x_ALERT = 7;
};
#else
struct settings_t {
    uint8_t Pin_I2C_SDA = 21, Pin_I2C_SCL = 22;
    uint8_t Pin_INA22x_ALERT = 19;

};
#endif
settings_t settings;

bool readCalibrationFactors(size_t ecIndex, float &u, float &i);

void storeCalibrationFactors(uint8_t ecIndex, float u, float i) {
    assert(ecIndex <= 4);

    float oldU, oldI;
    if (readCalibrationFactors(ecIndex, oldU, oldI)) {
        ESP_LOGI("store", "Writing calib.factors at %hhu: %.6f/%.6f (old: %.6f/%.6f)", ecIndex, u, i, oldU, oldI);
    }

    EEPROM.begin(256);
    // 16 bytes name
    // 32 bytes calibration data (4*2*4)
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 0, u);
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 4, i);
    EEPROM.commit();
    EEPROM.end();
}

bool checkCalibrationFactorBounds(float f) {
    if(!std::isnormal(f))
        return false;

    if (std::abs(f) < (1 / 10e4)) {
        return false;
    }
    return f > -10e3 and f < 10e3;
}

bool readCalibrationFactors(size_t ecIndex, float &u, float &i) {
    assert(ecIndex <= 4);
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
