#pragma once

#include <EEPROM.h>

struct settings_t {
    uint8_t Pin_I2C_SDA = 42, Pin_I2C_SCL = 2;
    uint8_t Pin_INA226_ALERT = 40; // ESP32: 19
    float CalibFactor_INA226_U = 1, CalibFactor_INA226_I = 1;
};

settings_t settings;


void storeCalibrationFactors(uint8_t ecIndex, float u, float i) {
    assert(ecIndex <= 4);
    EEPROM.begin(256);
    // 16 bytes name
    // 32 bytes calibration data (4*2*4)
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 0, u);
    EEPROM.writeFloat(16 + ecIndex * 4 * 2 + 4, i);
    EEPROM.commit();
    EEPROM.end();
}

bool checkCalibrationFactorBounds(float f) {
    if(std::abs(f) < (1/10e4)) {
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

    if((checkCalibrationFactorBounds(u_) and checkCalibrationFactorBounds(i_))) {
        u = u_;
        i = i_;
        return true;
    }
    return false;
}
