#include <Arduino.h>

#include <esp_log.h>
#include <NimBLEDevice.h>

#include "ble_protocol.h"

#define SERVICE_UUID        "e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb"
#define CHARACTERISTIC_UUID "df51a73d-0b60-43a5-bc86-a043f3841152"
#define AUX_CHAR_UUID       "b952dad5-9541-4852-bab6-96b9cbc9131a"

#define UART_BAUD 115200
#define UART_RX_BUF 1024

static NimBLEServer *pServer = nullptr;
static NimBLECharacteristic *pTelemetryChar = nullptr;
static NimBLECharacteristic *pAuxChar = nullptr;

static volatile bool bleConnected = false;
static volatile bool bleSubscribed = false;

#define MAX_FRAME_PAYLOAD 255
static uint8_t rxBuf[MAX_FRAME_PAYLOAD + 16];
static size_t rxLen = 0;

enum class RxState : uint8_t { WAIT_SOF0, WAIT_SOF1, WAIT_TYPE, WAIT_LEN_HI, WAIT_LEN_LO, WAIT_PAYLOAD, WAIT_CRC_HI, WAIT_CRC_LO };
static RxState rxState = RxState::WAIT_SOF0;
static uint8_t rxType = 0;
static uint16_t rxPayloadLen = 0;
static uint16_t rxCrc = 0;

static void ble_crc16_update(uint16_t &crc, uint8_t b) {
    crc ^= (uint16_t)b;
    for (int j = 0; j < 8; j++) {
        if (crc & 1) { crc >>= 1; crc ^= 0xA001; }
        else { crc >>= 1; }
    }
}

static void sendFrameToSTM32(uint8_t type, const uint8_t *data, uint16_t len) {
    uint8_t txBuf[256];
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
    Serial1.write(txBuf, pos);
}

static bool parseAuxCommand(const uint8_t *data, size_t len, uint8_t &out) {
    if (!data || !len) return false;
    if (len == 1 && (data[0] == 0 || data[0] == 1)) { out = data[0]; return true; }

    String s;
    for (size_t i = 0; i < len; ++i) s += (char)data[i];
    s.trim();
    s.toLowerCase();

    if (s == "1" || s == "on" || s == "true") { out = 1; return true; }
    if (s == "0" || s == "off" || s == "false") { out = 0; return true; }
    if (s == "toggle") { out = 0xFF; return true; }
    return false;
}

static void processFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
    switch (type) {
        case BLE_MSG_TELEMETRY:
            if (pTelemetryChar && bleSubscribed) {
                pTelemetryChar->setValue((uint8_t *)payload, len);
                pTelemetryChar->indicate();
            }
            break;
        case BLE_MSG_AUX_STATE:
            if (pAuxChar) {
                if (len >= 1) {
                    uint8_t v = payload[0] ? '1' : '0';
                    pAuxChar->setValue(&v, 1);
                    if (bleConnected) pAuxChar->notify(&v, 1);
                }
            }
            break;
        default:
            break;
    }
}

static void processUartRx() {
    while (Serial1.available()) {
        uint8_t b = Serial1.read();

        switch (rxState) {
            case RxState::WAIT_SOF0:
                if (b == BLE_PROTO_FRAME_SOF0) rxState = RxState::WAIT_SOF1;
                break;
            case RxState::WAIT_SOF1:
                if (b == BLE_PROTO_FRAME_SOF1) rxState = RxState::WAIT_TYPE;
                else if (b == BLE_PROTO_FRAME_SOF0) { }
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
                if (rxPayloadLen > MAX_FRAME_PAYLOAD) {
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
                    ESP_LOGW("bridge", "CRC mismatch: got 0x%04X, expected 0x%04X", rxCrc, calc);
                }
                rxState = RxState::WAIT_SOF0;
                break;
            }
        }
    }
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        bleConnected = true;
    }
    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        bleConnected = false;
        bleSubscribed = false;
        sendFrameToSTM32(BLE_MSG_LINK_STATUS, (uint8_t[]){0}, 1);
        NimBLEDevice::startAdvertising();
    }
};

class TelemetryCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, uint16_t subValue) override {
        if (subValue >= 2) {
            bleSubscribed = true;
            sendFrameToSTM32(BLE_MSG_LINK_STATUS, (uint8_t[]){1}, 1);
        } else if (subValue == 0) {
            bleSubscribed = false;
            sendFrameToSTM32(BLE_MSG_LINK_STATUS, (uint8_t[]){0}, 1);
        }
    }
};

class AuxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo) override {
        std::string val = chr->getValue();
        uint8_t cmd;
        if (parseAuxCommand((const uint8_t *)val.data(), val.length(), cmd)) {
            sendFrameToSTM32(BLE_MSG_AUX_SET, &cmd, 1);
        }
    }
};

static ServerCallbacks serverCallbacks;
static TelemetryCallbacks telemetryCallbacks;
static AuxCallbacks auxCallbacks;

void setup() {
    Serial.begin(115200);

    Serial1.setRxBufferSize(UART_RX_BUF);
    Serial1.begin(UART_BAUD, SERIAL_8N1, 18, 17);

    delay(500);
    ESP_LOGI("bridge", "BLE relay module starting");

    NimBLEDevice::init("smart-shunt");
    NimBLEDevice::setMTU(512);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    pTelemetryChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::INDICATE);
    pTelemetryChar->setCallbacks(&telemetryCallbacks);
    pTelemetryChar->setValue("\0\0\0\0");

    pAuxChar = pService->createCharacteristic(
        AUX_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pAuxChar->setCallbacks(&auxCallbacks);
    pAuxChar->setValue((uint8_t[]){'0'}, 1);

    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->enableScanResponse(true);
    pAdvertising->setAdvertisingInterval(800);
    pAdvertising->start();

    ESP_LOGI("bridge", "BLE server started, advertising");
}

void loop() {
    processUartRx();
    delay(1);
}
