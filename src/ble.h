#pragma once

/*
  Complete Getting Started Guide: https://RandomNerdTutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
  Ported to Arduino ESP32 by Evandro Copercini
*/

#define USE_ARDUINO_BLE 1

#ifdef USE_ARDUINO_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#else
#include <NimBLEDevice.h>
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
    void begin() {

        BLEDevice::init("MyESP32");
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
        pAdvertising->setMinPreferred(0x0C80); // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_gap_ble.html
        BLEDevice::startAdvertising();
        ESP_LOGI("ble", "BLE server started");
    }

    void setVal(const uint8_t *buf, size_t len) {
        if (pCharacteristic) {
            pCharacteristic->setValue(const_cast<uint8_t*>(buf), len);
            pCharacteristic->notify();
        }
    }
};
#else
class BleSrv {
    NimBLEServer* NimBLEServer = nullptr;

public:
    void begin() {

        BLEDevice::init("MyESP32");
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
        pAdvertising->setMinPreferred(0x0C80); // https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_gap_ble.html
        BLEDevice::startAdvertising();
        ESP_LOGI("ble", "BLE server started");
    }

    void setVal(const uint8_t *buf, size_t len) {
        if (pCharacteristic) {
            pCharacteristic->setValue(const_cast<uint8_t*>(buf), len);
            pCharacteristic->notify();
        }
    }
};
#endif