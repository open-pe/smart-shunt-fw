#pragma once

#ifdef TARGET_STM32H5

#include <Arduino.h>
#include "esp_compat.h"
#include "ble_protocol.h"
#include "aux_switch.h"

#define BLE_UART_BAUD 115200
#define BLE_FRAME_MAX_PAYLOAD 255

extern bool g_bleLinkConnected;

class BleBridge {
    HardwareSerial *uart;
    uint8_t rxBuf[BLE_FRAME_MAX_PAYLOAD + 16];
    size_t rxLen = 0;

    enum class RxState : uint8_t { WAIT_SOF0, WAIT_SOF1, WAIT_TYPE, WAIT_LEN_HI, WAIT_LEN_LO, WAIT_PAYLOAD, WAIT_CRC_HI, WAIT_CRC_LO };
    RxState rxState = RxState::WAIT_SOF0;
    uint8_t rxType = 0;
    uint16_t rxPayloadLen = 0;
    uint16_t rxCrc = 0;

    uint8_t txBuf[BLE_FRAME_MAX_PAYLOAD + 16];

    void processFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
        switch (type) {
            case BLE_MSG_AUX_SET:
                if (len >= 1) {
                    bool want = payload[0] != 0;
                    auxSet(want);
                }
                break;
            case BLE_MSG_LINK_STATUS:
                if (len >= 1) {
                    g_bleLinkConnected = payload[0] != 0;
                    ESP_LOGI("ble", "link status: %s", g_bleLinkConnected ? "connected" : "disconnected");
                }
                break;
            default:
                break;
        }
    }

public:
    BleBridge(HardwareSerial &port) : uart(&port) {}

    void begin() {
        uart->begin(BLE_UART_BAUD);
        ESP_LOGI("ble", "BLE bridge started on UART @ %d baud", BLE_UART_BAUD);
    }

    void sendFrame(uint8_t type, const uint8_t *data, uint16_t len) {
        if (len > BLE_FRAME_MAX_PAYLOAD) return;

        size_t pos = 0;
        txBuf[pos++] = BLE_PROTO_FRAME_SOF0;
        txBuf[pos++] = BLE_PROTO_FRAME_SOF1;
        txBuf[pos++] = type;
        txBuf[pos++] = (uint8_t)(len >> 8);
        txBuf[pos++] = (uint8_t)(len & 0xFF);

        uint16_t crc = 0xFFFF;
        ble_crc16_update(crc, type);
        ble_crc16_update(crc, (uint8_t)(len >> 8));
        ble_crc16_update(crc, (uint8_t)(len & 0xFF));
        for (uint16_t i = 0; i < len; i++) {
            txBuf[pos++] = data[i];
            ble_crc16_update(crc, data[i]);
        }

        txBuf[pos++] = (uint8_t)(crc >> 8);
        txBuf[pos++] = (uint8_t)(crc & 0xFF);

        uart->write(txBuf, pos);
    }

    void send(const uint8_t *data, size_t len) {
        sendFrame(BLE_MSG_TELEMETRY, data, (uint16_t)len);
    }

    void flush() {
        uart->flush();
    }

    void publishAux(bool on) {
        uint8_t v = on ? 1 : 0;
        sendFrame(BLE_MSG_AUX_STATE, &v, 1);
    }

    bool isConnected() const {
        return g_bleLinkConnected;
    }

    void tick() {
        while (uart->available()) {
            uint8_t b = uart->read();

            switch (rxState) {
                case RxState::WAIT_SOF0:
                    if (b == BLE_PROTO_FRAME_SOF0) rxState = RxState::WAIT_SOF1;
                    break;
                case RxState::WAIT_SOF1:
                    if (b == BLE_PROTO_FRAME_SOF1) rxState = RxState::WAIT_TYPE;
                    else if (b == BLE_PROTO_FRAME_SOF0) { /* stay */ }
                    else rxState = RxState::WAIT_SOF0;
                    break;
                case RxState::WAIT_TYPE:
                    rxType = b;
                    rxState = RxState::WAIT_LEN_HI;
                    break;
                case RxState::WAIT_LEN_HI:
                    rxPayloadLen = (uint16_t)b << 8;
                    rxState = RxState::WAIT_LEN_LO;
                    break;
                case RxState::WAIT_LEN_LO:
                    rxPayloadLen |= b;
                    if (rxPayloadLen > BLE_FRAME_MAX_PAYLOAD) {
                        rxState = RxState::WAIT_SOF0;
                    } else if (rxPayloadLen == 0) {
                        rxState = RxState::WAIT_CRC_HI;
                    } else {
                        rxLen = 0;
                        rxState = RxState::WAIT_PAYLOAD;
                    }
                    break;
                case RxState::WAIT_PAYLOAD:
                    rxBuf[rxLen++] = b;
                    if (rxLen >= rxPayloadLen) rxState = RxState::WAIT_CRC_HI;
                    break;
                case RxState::WAIT_CRC_HI:
                    rxCrc = (uint16_t)b << 8;
                    rxState = RxState::WAIT_CRC_LO;
                    break;
                case RxState::WAIT_CRC_LO: {
                    rxCrc |= b;
                    uint16_t calc = 0xFFFF;
                    ble_crc16_update(calc, rxType);
                    ble_crc16_update(calc, (uint8_t)(rxPayloadLen >> 8));
                    ble_crc16_update(calc, (uint8_t)(rxPayloadLen & 0xFF));
                    for (uint16_t i = 0; i < rxPayloadLen; i++)
                        ble_crc16_update(calc, rxBuf[i]);
                    if (calc == rxCrc) {
                        processFrame(rxType, rxBuf, rxPayloadLen);
                    } else {
                        ESP_LOGW("ble", "CRC mismatch: got 0x%04X, expected 0x%04X", rxCrc, calc);
                    }
                    rxState = RxState::WAIT_SOF0;
                    break;
                }
            }
        }
    }
};

bool g_bleLinkConnected = false;

#else
#include "ble.h"
using BleBridge = BleSrv;
#endif
