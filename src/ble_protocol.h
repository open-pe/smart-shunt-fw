#pragma once

#include <stdint.h>
#include <string.h>

#define BLE_PROTO_FRAME_SOF0 0xAA
#define BLE_PROTO_FRAME_SOF1 0x55

enum BleMsgType : uint8_t {
    BLE_MSG_TELEMETRY     = 0x01,
    BLE_MSG_AUX_SET       = 0x02,
    BLE_MSG_AUX_STATE     = 0x03,
    BLE_MSG_CALIB_SET     = 0x04,
    BLE_MSG_RESET_CMD     = 0x05,
    BLE_MSG_RESISTOR_RANGE= 0x06,
    BLE_MSG_BOOTINFO      = 0x07,
    BLE_MSG_LINK_STATUS   = 0x08,
    BLE_MSG_CONSOLE_LOG   = 0x09,
};

#pragma pack(push, 1)
struct BleFrameHeader {
    uint8_t sof0;
    uint8_t sof1;
    uint8_t type;
    uint16_t len;
};

#pragma pack(pop)

static inline uint16_t ble_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static inline void ble_crc16_update(uint16_t &crc, uint8_t b) {
    crc ^= (uint16_t)b;
    for (int j = 0; j < 8; j++) {
        if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
        else { crc >>= 1; }
    }
}
