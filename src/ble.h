#pragma once

/*
  Complete Getting Started Guide: https://RandomNerdTutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
  Ported to Arduino ESP32 by Evandro Copercini


indicuations:
https://community.silabs.com/s/article/x-deprecated-kba-bt-0102-ble-basics-master-slave-gatt-client-server-data-rx-x?language=en_US#:~:text=Notify%20and%20indicate%20operations%20are,therefore%20faster%2C%20but%20less%20reliable.
"It will be followed up by the gecko_evt_gatt_server_characteristic_status event with status_flags = 0x2 when the acknowledgement comes back from the GATT client."
NimBLECharacteristicCallbacks::onStatus
*/

//#define USE_ARDUINO_BLE 0

#ifdef USE_ARDUINO_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#else
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLELocalValueAttribute.h> // NIMBLE_PROPERTY
#endif

#include <esp_log.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb"
#define CHARACTERISTIC_UUID "df51a73d-0b60-43a5-bc86-a043f3841152"

#ifdef USE_ARDUINO_BLE
class BleSrv {
    BLECharacteristic *pCharacteristic = nullptr;

public:
    static constexpr uint16_t MTU = 256;
    static constexpr uint16_t MAX_PAYLOAD_LEN = MTU - 3;

    void begin() {
        BLEDevice::init("smart-shunt");
        BLEServer *pServer = BLEDevice::createServer();


        BLEService *pService = pServer->createService(SERVICE_UUID);
        pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY
        );

        pCharacteristic->setValue("\0\0\0\0");
        pService->start();
        // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        //pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
        pAdvertising->setMinPreferred(0x12);
        // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_gap_ble.html
        if (BLEDevice::setMTU(MTU) != ESP_OK) {
            ESP_LOGW("ble", "BLE server set MTU failed");
        }
        BLEDevice::startAdvertising();

        ESP_LOGI("ble", "BLE server started");
    }

    void setVal(const uint8_t *buf, size_t len) {
        if (pCharacteristic) {
            pCharacteristic->setValue(const_cast<uint8_t *>(buf), len);
            pCharacteristic->notify();
        }
    }
};
#else
class BleSrv {
    BLEServer *pServer = nullptr;
    BLECharacteristic *pCharacteristic = nullptr;

public:
    static constexpr uint16_t MTU = 512; // lower than BLE_ATT_MTU_MAX
    static constexpr uint16_t MAX_PAYLOAD_LEN = MTU - 3;

    static constexpr auto bleBufLen = BleSrv::MAX_PAYLOAD_LEN * 1;
    uint8_t bleBuf[bleBufLen];
    uint16_t bleBufPos = 0;

    bool subscribed = false; // whether the client subscribed for indications
    uint8_t waitingForAck = 0; // in-transit
    uint32_t tLastIndicate = 0;

    // An indication that is never acknowledged must not park the buffer forever.
    static constexpr uint32_t ACK_TIMEOUT_MS = 5000;

    bool inTransmission() const { return waitingForAck > 0; }

    bool isConnected() const { return pServer && pServer->getConnectedCount() > 0; }

    BleSrv() : chrCallbacks{this} {
    }

    void begin() {
        assert(pServer == nullptr);
        BLEDevice::init("smart-shunt");
        if (!NimBLEDevice::setMTU(MTU)) {
            ESP_LOGW("ble", "BLE server set MTU failed");
        }
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(&serverCallbacks);
        BLEService *pService = pServer->createService(SERVICE_UUID);
        pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::INDICATE);
        pCharacteristic->setCallbacks(&chrCallbacks);
        pCharacteristic->setValue("\0\0\0\0");
        pService->start();
        // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->enableScanResponse(true); //setScanResponse(true);
        //pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
        //pAdvertising->setMinPreferred(0x0C80);
        pAdvertising->setAdvertisingInterval(800); // 3200
        //pAdvertising->setM

        pAdvertising->start(); // BLEDevice::startAdvertising();
        // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_gap_ble.html
        //BLEDevice::startAdvertising();
        ESP_LOGI("ble", "BLE server started");
    }

    /*
    void setVal(const uint8_t *buf, size_t len) {
        if (pCharacteristic) {
            //pCharacteristic->setValue(const_cast<uint8_t *>(buf), len);
            bool res = pCharacteristic->indicate(const_cast<uint8_t *>(buf), len);
            if (!res) {
                ESP_LOGW("ble", "BLE server indicate failed");
            }
        }
    }
    */

    void send(const uint8_t *buf, size_t len) {
        if (bleBufPos + len > bleBufLen) {
            ESP_LOGW("main", "ble buf overflow, dropping sample");
        } else {
            memcpy(&bleBuf[bleBufPos], buf, len);
            bleBufPos += len;
        }
        if (!inTransmission()) {
            flush();
        }
    }

    void flush() {
        if (inTransmission()) {
            // Without this the client stops receiving samples while the link stays up, so
            // nothing on either side looks wrong -- the worst kind of stall.  bleBufPos is
            // untouched by a lost ack, so this re-sends the same payload.
            if (millis() - tLastIndicate < ACK_TIMEOUT_MS) return;
            ESP_LOGW("ble", "no ack for %ums with %hhu indication(s) in flight, re-arming",
                     (unsigned) (millis() - tLastIndicate), waitingForAck);
            waitingForAck = 0;
        }
        if (!isConnected()) {
            return;
        }

        if (pCharacteristic && bleBufPos && subscribed) {
            ESP_LOGI("ble", "sending %hu", bleBufPos);
            //pCharacteristic->setValue(const_cast<uint8_t *>(buf), len);
            bool res = pCharacteristic->indicate(bleBuf, bleBufPos);
            if (!res) {
                ESP_LOGW("ble", "BLE server indicate failed");
            } else {
                ++waitingForAck;
                tLastIndicate = millis();
            }
        }
    }

    class ServerCallbacks : public NimBLEServerCallbacks {
        void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
            Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

            /**
             *  We can use the connection handle here to ask for different connection parameters.
             *  Args: connection handle, min connection interval, max connection interval
             *  latency, supervision timeout.
             *  Units; Min/Max Intervals: 1.25 millisecond increments.
             *  Latency: number of intervals allowed to skip.
             *  Timeout: 10 millisecond increments.
             */
            pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
        }

        void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
            Serial.printf("Client disconnected - start advertising\n");
            NimBLEDevice::startAdvertising();
        }

        void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override {
            Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
        }

        /********************* Security handled here *********************/
        uint32_t onPassKeyDisplay() override {
            Serial.printf("Server Passkey Display\n");
            /**
             * This should return a random 6 digit number for security
             *  or make your own static passkey as done here.
             */
            return 123456;
        }

        void onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t pass_key) override {
            Serial.printf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
            /** Inject false if passkeys don't match. */
            NimBLEDevice::injectConfirmPasskey(connInfo, true);
        }

        void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
            /** Check that encryption was successful, if not we disconnect the client */
            if (!connInfo.isEncrypted()) {
                NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
                Serial.printf("Encrypt connection failed - disconnecting client\n");
                return;
            }

            Serial.printf("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
        }
    } serverCallbacks;

    void onAck(bool done) {
        if (waitingForAck) {
            --waitingForAck;
        } else {
            // A status callback with nothing in flight is reachable: onCharSub() zeroes the
            // counter, and an indication outstanding at that moment still reports later.
            // Decrementing here would underflow the uint8_t to 255, so inTransmission()
            // would be true forever and flush() would never send another sample again --
            // with the connection still up and healthy.
            ESP_LOGW("ble", "onAck (done=%i) with no indication in flight", (int) done);
        }
        ESP_LOGI("ble", "onAck done=%i waiting=%hhu", (int)done, waitingForAck);
        if (done) {
            bleBufPos = 0;
        }
    }

    void onCharSub(bool sub) {
        waitingForAck = 0;
        subscribed = sub;
    }

    /** Handler class for characteristic actions */
    class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
        BleSrv *srv;

    public:
        explicit CharacteristicCallbacks(BleSrv *srv) : srv{srv} {
        }

        void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
            Serial.printf("%s : onRead(), value: %s\n",
                          pCharacteristic->getUUID().toString().c_str(),
                          pCharacteristic->getValue().c_str());
        }

        void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
            Serial.printf("%s : onWrite(), value: %s\n",
                          pCharacteristic->getUUID().toString().c_str(),
                          pCharacteristic->getValue().c_str());
        }

        /**
         *  The value returned in code is the NimBLE host return code.
         */
        void onStatus(NimBLECharacteristic *pCharacteristic, int code) override {
            srv->onAck(code == BLE_HS_EDONE);
            if (code != BLE_HS_EDONE) {
                Serial.printf("Notification/Indication return code: %d, %s\n", code,
                              NimBLEUtils::returnCodeToString(code));
            }
        }

        /** Peer subscribed to notifications/indications */
        void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override {
            std::string str = "Client ID: ";
            str += connInfo.getConnHandle();
            str += " Address: ";
            str += connInfo.getAddress().toString();
            if (subValue == 0) {
                str += " Unsubscribed to ";
                srv->onCharSub(false);
            } else if (subValue == 1) {
                str += " Subscribed to notifications for ";
            } else if (subValue == 2) {
                str += " Subscribed to indications for ";
                srv->onCharSub(true);
            } else if (subValue == 3) {
                str += " Subscribed to notifications and indications for ";
            }
            str += std::string(pCharacteristic->getUUID());

            Serial.printf("%s\n", str.c_str());
        }
    } chrCallbacks;
};


#endif
