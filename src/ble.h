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

#include <mutex>
#include <string>

#include <ota_ble.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb"
#define CHARACTERISTIC_UUID "df51a73d-0b60-43a5-bc86-a043f3841152"

// OTA firmware push, on the same service as the telemetry characteristic. Control and status share
// one characteristic (fugu splits them only because it is reusing a console); firmware bytes get
// their own write-no-response characteristic because that is the only high-throughput path.
#define OTA_CTRL_CHAR_UUID  "b0e0d1a4-7f52-4a3e-9c61-2d8f5b3ae741"
#define OTA_DATA_CHAR_UUID  "b0e0d1a5-7f52-4a3e-9c61-2d8f5b3ae741"

/// Set by the OTA quiesce hook, read by realTimeTask. Flash erase/write disables the CPU cache and
/// stalls the other core, so the sampling loop has to stand down for the duration of a transfer.
extern volatile bool g_samplingHalted;

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
    // How much of bleBuf the in-flight indication actually covers. send() keeps appending
    // while an indication is unacked, so the ack must only consume THIS many bytes -- an ack
    // that clears the whole buffer destroys every sample queued since the indication was cut
    // (that bug made the first energy counter's samples the only ones that ever arrived).
    //
    // Single-writer rule: the app task (send/flush) is the ONLY writer of bleBuf, bleBufPos,
    // inFlightLen and waitingForAck. The NimBLE host-task callbacks write nothing but the
    // single-byte flags below, which the next send()/flush() consumes -- so no lock is needed
    // across the two tasks.
    uint16_t inFlightLen = 0;
    volatile bool acked = false;      // onAck(done): the in-flight indication was confirmed
    volatile bool ackFailed = false;  // onAck(!done): it terminally failed; retire it unconsumed
    // The subscription changed (unsubscribe, resubscribe or disconnect). Consumed at the top of
    // send()/flush(), which discard the buffer: it belongs to the previous session. A flag (not
    // an app-task check of `subscribed`) because a quick unsub->resub or disconnect->reconnect
    // can complete WITHIN one 400ms send cadence, so the app task would never see the gap and
    // would ship the dead session's bytes to the new client.
    volatile bool sessionChanged = false;

    volatile bool subscribed = false; // whether the client subscribed for indications
    volatile uint8_t waitingForAck = 0; // in-transit; written by app task, read by onAck
    uint32_t tLastIndicate = 0;

    uint32_t nDroppedNoPeer = 0;   // discarded because nothing was subscribed
    uint32_t nDroppedBacklog = 0;  // discarded with a subscriber attached -- the real overflow

    // An indication that is never acknowledged must not park the buffer forever.
    static constexpr uint32_t ACK_TIMEOUT_MS = 5000;

    // ---- OTA over BLE -------------------------------------------------------------------------
    BLECharacteristic *pOtaCtrl = nullptr;
    BLECharacteristic *pOtaData = nullptr;

    /// Negotiated ATT MTU. onMTUChange used to only print it; notifications have to be chunked to
    /// it, and CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256 in the prebuilt libs caps the 512 we ask for,
    /// so the requested value is not the value we get.
    volatile uint16_t peerMtu = 23;
    volatile uint16_t connHandle = 0;
    volatile bool haveConn = false;

    /// Pending connection-parameter change, applied from the app task. NOT from onConnect: issuing
    /// a device-initiated LL update synchronously inside the connect callback trips a controller
    /// assert (lld_con.c:3275) -- fugu paid for that one.
    enum class ConnParams : uint8_t { None, Default, Fast };
    volatile ConnParams connParamsWanted = ConnParams::None;
    ConnParams connParamsApplied = ConnParams::None;

    /// OTA status stream (device -> host). The status hook appends here from the app task and the
    /// control characteristic's write callback appends from the NimBLE host task, so unlike the
    /// telemetry path this one genuinely needs a lock.
    std::string otaTx;
    std::mutex otaTxMutex;
    static constexpr size_t OTA_TX_CAP = 4096;
    uint32_t nOtaTxDropped = 0;

    inline static BleSrv *s_self = nullptr;

    void otaStatusAppend(const char *line) {
        std::lock_guard<std::mutex> lk(otaTxMutex);
        if (otaTx.size() + strlen(line) + 1 > OTA_TX_CAP) {
            // Never silently: a dropped status line is a host that waits out a timeout instead of
            // learning what went wrong.
            ++nOtaTxDropped;
            ESP_LOGW("ble", "OTA status buffer full, dropped a line (%lu total)",
                     (unsigned long) nOtaTxDropped);
            return;
        }
        otaTx += line;
        otaTx += '\n';
    }

    static void otaStatusHook(OtaBleLevel level, const char *line) {
        // Mirrored to the UART log as well as the client -- during a bench OTA the serial console is
        // often the only view when the BLE link is the thing misbehaving.
        switch (level) {
            case OtaBleLevel::Info:  ESP_LOGI("otab", "%s", line); break;
            case OtaBleLevel::Warn:  ESP_LOGW("otab", "%s", line); break;
            case OtaBleLevel::Error: ESP_LOGE("otab", "%s", line); break;
        }
        if (s_self) s_self->otaStatusAppend(line);
    }

    static void otaQuiesceHook(bool halt) { g_samplingHalted = halt; }

    static void otaRestartHook() {
        Serial.flush();
        ESP.restart();
    }

    bool inTransmission() const { return waitingForAck > 0; }

    bool isConnected() const { return pServer && pServer->getConnectedCount() > 0; }

    BleSrv() : serverCallbacks{this}, chrCallbacks{this}, otaCtrlCallbacks{this} {
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

        pOtaCtrl = pService->createCharacteristic(
            OTA_CTRL_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
        pOtaCtrl->setCallbacks(&otaCtrlCallbacks);
        // Write-no-response only: an acked write per chunk would cap throughput at one packet per
        // connection interval, which is the difference between a one-minute push and a ten-minute one.
        pOtaData = pService->createCharacteristic(OTA_DATA_CHAR_UUID, NIMBLE_PROPERTY::WRITE_NR);
        pOtaData->setCallbacks(&otaDataCallbacks);

        pService->start();

        s_self = this;
        OtaBleHooks hooks;
        hooks.status = &BleSrv::otaStatusHook;
        hooks.quiesce = &BleSrv::otaQuiesceHook;
        hooks.restart = &BleSrv::otaRestartHook;
        otaBleInit(hooks);
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

    /// App task only. Discard all transfer state; the buffer content died with its session.
    void consumeSessionChange() {
        if (!sessionChanged) return;
        sessionChanged = false;
        bleBufPos = 0;
        inFlightLen = 0;
        waitingForAck = 0;
        acked = false;
        ackFailed = false;
    }

    void send(const uint8_t *buf, size_t len) {
        consumeSessionChange();
        if (otaBleActive()) {
            // Telemetry stands down for the duration of a firmware push: the sampler is halted
            // anyway, and indications would only compete with the OTA status notifies for the same
            // small mbuf pool. Dropped here rather than queued -- the buffer holds one payload.
            bleBufPos = 0;
            inFlightLen = 0;
            return;
        }
        if (!subscribed || !isConnected()) {
            // Nobody can receive this sample, so there is no point buffering it. The old
            // code queued it anyway: bleBuf filled up within a few seconds of running
            // unconnected and then EVERY subsequent sample logged "ble buf overflow",
            // once per sample, forever. That noise is not just untidy -- it is the same
            // message a genuine overflow *with a peer attached* produces, so the one
            // warning that matters was buried in thousands that did not.
            bleBufPos = 0;
            inFlightLen = 0;
            ++nDroppedNoPeer;
            return;
        }

        if (bleBufPos + len > bleBufLen) {
            // This one is real: a peer is subscribed and we still cannot keep up.
            ++nDroppedBacklog;
            ESP_LOGW("ble", "buffer full with a subscriber attached, dropping sample "
                     "(%hu queued, %lu dropped)", bleBufPos, (unsigned long) nDroppedBacklog);
        } else {
            memcpy(&bleBuf[bleBufPos], buf, len);
            bleBufPos += len;
        }
        if (!inTransmission()) {
            flush();
        }
    }

    /// Drop every trace of the previous connection. Without this, a stale bleBufPos and a
    /// stale waitingForAck survive into the next connection, where they either ship a
    /// fragment of the old session or wedge flush() before it ever sends.
    void onDisconnected() {
        // Host-task context: flags only. The app task tears the transfer state down when it
        // consumes sessionChanged.
        subscribed = false;
        sessionChanged = true;
    }

    /// App-task pump for everything that must not run on the NimBLE host task: the deferred
    /// connection-parameter change and the OTA status stream. Call once per app-task pass, after
    /// otaBleTick() so the lines it produced go out in the same pass.
    void tick() {
        // Self-reconciling rather than hook-driven: an OTA wants a 7.5-15ms interval (the default
        // 30-60ms would stretch a 1.4MB image over minutes), and the link goes back to the relaxed
        // interval as soon as the transfer ends, however it ended.
        if (haveConn) {
            ConnParams want = otaBleActive() ? ConnParams::Fast : ConnParams::Default;
            if (connParamsWanted != ConnParams::None) want = connParamsWanted;
            if (want != connParamsApplied) {
                if (want == ConnParams::Fast)
                    pServer->updateConnParams(connHandle, 6, 12, 0, 400);
                else
                    pServer->updateConnParams(connHandle, 24, 48, 0, 180);
                connParamsApplied = want;
                connParamsWanted = ConnParams::None;
            }
        }

        drainOtaTx();
    }

    /// Push queued OTA status bytes out with real backpressure. A failed notify means NimBLE's mbuf
    /// pool is empty; looping on it there exhausts the pool and silently drops packets, so we stop
    /// and pick up on the next tick when it has refilled.
    void drainOtaTx() {
        if (!pOtaCtrl || !isConnected()) return;
        const size_t chunk = peerMtu > 3 ? (size_t) (peerMtu - 3) : 20;
        for (;;) {
            std::string out;
            {
                std::lock_guard<std::mutex> lk(otaTxMutex);
                if (otaTx.empty()) return;
                size_t n = otaTx.size() < chunk ? otaTx.size() : chunk;
                out.assign(otaTx, 0, n);
                otaTx.erase(0, n);
            }
            if (!pOtaCtrl->notify((const uint8_t *) out.data(), out.size())) {
                // Put it back at the front; it never reached the peer.
                std::lock_guard<std::mutex> lk(otaTxMutex);
                otaTx.insert(0, out);
                return;
            }
        }
    }

    void flush() {
        consumeSessionChange();
        if (otaBleActive()) return; // see send()
        if (acked || ackFailed) {
            // Consume the ack here, in the app task, instead of inside onAck (host task):
            // only the bytes the acked indication covered leave the buffer, the tail that
            // send() queued since moves to the front and goes out with the next indication.
            // A failed indication retires without consuming -- the payload goes out again.
            const bool consume = acked;
            acked = false;
            ackFailed = false;
            waitingForAck = 0;
            if (consume) {
                if (inFlightLen > bleBufPos) {
                    // No path is known to produce this; if it ever fires the bookkeeping is
                    // wrong and silently clamping would hide it.
                    ESP_LOGE("ble", "inFlightLen %hu > bleBufPos %hu", inFlightLen, bleBufPos);
                    inFlightLen = bleBufPos;
                }
                memmove(bleBuf, bleBuf + inFlightLen, bleBufPos - inFlightLen);
                bleBufPos -= inFlightLen;
            }
            inFlightLen = 0;
        }
        if (inTransmission()) {
            // Without this the client stops receiving samples while the link stays up, so
            // nothing on either side looks wrong -- the worst kind of stall.  bleBufPos is
            // untouched by a lost ack, so the next indicate re-sends the same payload (the
            // client dedups by idx). A late ack for the ABANDONED indication is ignored by
            // onAck's waitingForAck guard only until we re-indicate; if it arrives after
            // that, it consumes the re-sent range early -- acceptable, since that range was
            // transmitted and the true ack for it then hits the guard and is dropped.
            if (millis() - tLastIndicate < ACK_TIMEOUT_MS) return;
            ESP_LOGW("ble", "no ack for %ums with %hhu indication(s) in flight, re-arming",
                     (unsigned) (millis() - tLastIndicate), waitingForAck);
            waitingForAck = 0;
            inFlightLen = 0;
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
                inFlightLen = bleBufPos;
                ++waitingForAck;
                tLastIndicate = millis();
            }
        }
    }

    class ServerCallbacks : public NimBLEServerCallbacks {
        BleSrv *srv;

    public:
        explicit ServerCallbacks(BleSrv *srv) : srv{srv} {
        }

    private:
        void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
            Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

            /**
             *  Connection parameters -- min/max interval in 1.25ms units, latency in intervals,
             *  supervision timeout in 10ms units -- are requested from the app task, not here.
             *  Issuing a device-initiated LL connection update synchronously from inside the connect
             *  callback trips a controller assert (lld_con.c:3275); BleSrv::tick() applies it a pass
             *  later, which is soon enough and cannot assert.
             */
            srv->connHandle = connInfo.getConnHandle();
            srv->haveConn = true;
            srv->connParamsApplied = ConnParams::None;
            srv->connParamsWanted = ConnParams::Default;
        }

        void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
            Serial.printf("Client disconnected (reason %d) - start advertising\n", reason);
            srv->onDisconnected();
            srv->haveConn = false;
            srv->connParamsApplied = ConnParams::None;
            srv->peerMtu = 23;
            // Harmless when no transfer is in flight -- otaBleRequestAbort() ignores it -- which
            // matters, because this fires for every ordinary telemetry disconnect too.
            otaBleRequestAbort();
            // Unchecked, this is the difference between "the peer went away" and "this
            // device is never reachable again", with nothing in the log to tell them
            // apart. That is exactly the symptom we spent a day chasing from the client
            // side, so it says so if it fails.
            if (!NimBLEDevice::startAdvertising()) {
                ESP_LOGE("ble", "startAdvertising() FAILED after disconnect -- unreachable");
            }
        }

        void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override {
            Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
            srv->peerMtu = MTU; // notifications are chunked to this, so it has to be kept, not just printed
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
            // Host-task context: report only. flush() clears waitingForAck and consumes the
            // buffer in the app task.
            if (done) acked = true;
            else ackFailed = true;
        } else {
            // A status callback with nothing in flight is reachable: a session change zeroes
            // the counter while an indication is still outstanding, and the abandoned
            // indication reports later. Attributing it to anything would consume bytes it
            // never covered, so it is only logged.
            ESP_LOGW("ble", "onAck (done=%i) with no indication in flight, ignored", (int) done);
        }
        ESP_LOGI("ble", "onAck done=%i waiting=%hhu", (int)done, waitingForAck);
    }

    void onCharSub(bool sub) {
        // Host-task context: flags only, see the single-writer rule at the top.
        subscribed = sub;
        sessionChanged = true;
        if (sub && nDroppedNoPeer) {
            ESP_LOGI("ble", "subscriber attached, %lu samples were discarded unsent",
                     (unsigned long) nDroppedNoPeer);
            nDroppedNoPeer = 0;
        }
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

    /// OTA control plane. Runs on the NimBLE host task, so it only parses and latches -- otaBleTick()
    /// on the app task does the flash work, which blocks for far longer than a host callback may.
    class OtaCtrlCallbacks : public NimBLECharacteristicCallbacks {
        BleSrv *srv;

    public:
        explicit OtaCtrlCallbacks(BleSrv *srv) : srv{srv} {
        }

        void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
            std::string v = pCharacteristic->getValue();
            // Tolerate a client that sends the newline the protocol is written with.
            while (!v.empty() && (v.back() == '\n' || v.back() == '\r')) v.pop_back();
            if (v.empty()) return;
            // A rejection is announced by the module itself, on the same OTAB status channel as
            // every other failure -- nothing to add here beyond echoing what was actually sent.
            if (otaBleSubmitCommand(v.c_str()) == OtaBleSubmit::Rejected)
                ESP_LOGW("otab", "rejected command: %s", v.c_str());
        }
    } otaCtrlCallbacks;

    /// OTA data plane: copy into the staging ring and return. Nothing here may touch flash.
    class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
    public:
        void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
            std::string v = pCharacteristic->getValue();
            otaBleStageBytes((const uint8_t *) v.data(), v.size());
        }
    } otaDataCallbacks;
};


#endif
