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

#include "aux_switch.h"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb"
#define CHARACTERISTIC_UUID "df51a73d-0b60-43a5-bc86-a043f3841152"

// OTA firmware push, on the same service as the telemetry characteristic. Control and status share
// one characteristic (fugu splits them only because it is reusing a console); firmware bytes get
// their own write-no-response characteristic because that is the only high-throughput path.
#define OTA_CTRL_CHAR_UUID  "b0e0d1a4-7f52-4a3e-9c61-2d8f5b3ae741"
#define OTA_DATA_CHAR_UUID  "b0e0d1a5-7f52-4a3e-9c61-2d8f5b3ae741"

// Aux switch (see aux_switch.h). Value is a single ASCII '0'/'1' so a read or a notification is
// trivially parseable; writes additionally accept raw 0/1 and on/off/toggle.
#define AUX_CHAR_UUID       "b952dad5-9541-4852-bab6-96b9cbc9131a"

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
#include "ble_transport.h"
class BleSrv : public BleTransport {
    BLEServer *pServer = nullptr;
    BLECharacteristic *pCharacteristic = nullptr;

public:
    static constexpr uint16_t MTU = 512; // lower than BLE_ATT_MTU_MAX
    static constexpr uint16_t MAX_PAYLOAD_LEN = MTU - 3;

    /// Queue depth, in units of one indication payload.
    ///
    /// Was *1, i.e. exactly one payload (509 B = 7 WireSamples). That gave the
    /// link no elasticity at all: one indication round trip measured ~600 ms on
    /// the bench, so any tick whose samples outran a single in-flight payload
    /// dropped the remainder -- 45 "buffer full" warnings in 50 s with a
    /// subscriber attached, and the dropped samples never reach InfluxDB, so the
    /// stored series is silently decimated rather than merely late.
    ///
    /// *4 holds ~31 samples, several ticks' worth, so a slow ack costs LATENCY
    /// instead of DATA. flush() clamps each indication to MAX_PAYLOAD_LEN and the
    /// existing inFlightLen/memmove bookkeeping carries the remainder, so a
    /// buffer larger than one payload is drained across successive indications.
    static constexpr auto bleBufLen = BleSrv::MAX_PAYLOAD_LEN * 4;
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

#ifdef BLE_STATUS_NEOPIXEL_PIN
    // Minimum nonzero WS2812 channel value: visible status with the least light.
    static constexpr uint8_t BLE_STATUS_BLUE_BRIGHTNESS = 1;
    /// -1 forces begin() to transmit black once; afterward the WS2812 only needs
    /// another frame when the displayed connection state changes.
    int8_t connectionLedState = -1;

    void updateConnectionLed(bool connected) {
        const int8_t next = connected ? 1 : 0;
        if (connectionLedState == next) return;
        rgbLedWrite(BLE_STATUS_NEOPIXEL_PIN, 0, 0,
                    connected ? BLE_STATUS_BLUE_BRIGHTNESS : 0);
        connectionLedState = next;
    }
#endif

    /// Pending connection-parameter change, applied from the app task. NOT from onConnect: issuing
    /// a device-initiated LL update synchronously inside the connect callback trips a controller
    /// assert (lld_con.c:3275) -- fugu paid for that one.
    enum class ConnParams : uint8_t { None, Default, Fast };
    /// App task (tick()) is the ONLY writer -- see the single-writer rule above. There is
    /// deliberately no "wanted" companion field: nothing outside tick() gets to request a
    /// parameter change, because the steady-state answer is "ask for nothing".
    ConnParams connParamsApplied = ConnParams::None;

    /// OTA status stream (device -> host). The status hook appends here from the app task and the
    /// control characteristic's write callback appends from the NimBLE host task, so unlike the
    /// telemetry path this one genuinely needs a lock.
    std::string otaTx;
    std::mutex otaTxMutex;
    static constexpr size_t OTA_TX_CAP = 4096;
    uint32_t nOtaTxDropped = 0;

    inline static BleSrv *s_self = nullptr;

    // ---- aux switch --------------------------------------------------------------------------
    BLECharacteristic *pAuxChar = nullptr;
    /// Latched by the write callback on the NimBLE host task; applied by tick() on the app task,
    /// because setting the switch persists to NVS and that writes flash.
    volatile bool auxReqPending = false;
    volatile bool auxReqValue = false;
    /// Last value published to the characteristic, so tick() only notifies on an actual change.
    int8_t auxPublished = -1;

    void publishAux(bool on) override {
        if (!pAuxChar) return;
        const char v = on ? '1' : '0';
        pAuxChar->setValue((uint8_t *) &v, 1); // keep a plain read truthful, connected or not
        if (isConnected()) pAuxChar->notify((uint8_t *) &v, 1);
        auxPublished = on ? 1 : 0;
    }

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

    bool isConnected() const override { return pServer && pServer->getConnectedCount() > 0; }

    BleSrv() : serverCallbacks{this}, chrCallbacks{this}, otaCtrlCallbacks{this}, auxCallbacks{this} {
    }

    void begin() override {
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

        pAuxChar = pService->createCharacteristic(
            AUX_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
        pAuxChar->setCallbacks(&auxCallbacks);
        publishAux(auxGet()); // the pin already has its restored state; make a read reflect it

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

#ifdef BLE_STATUS_NEOPIXEL_PIN
        updateConnectionLed(false);
#endif
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

    void send(const uint8_t *buf, size_t len) override {
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
        /* DELIBERATELY NO flush() HERE. Telemetry calls send() once per sampler
         * and then flush() once at the end of the tick (telemetry.h), so an
         * eager flush on the FIRST send shipped a single 64-byte sample and then
         * blocked every later send in that tick behind the in-flight ack --
         * visible in the log as a stream of "sending 64" against a 448-byte
         * capacity. At a ~600 ms round trip that caps throughput near 1.7
         * samples/s no matter how much buffer exists, which is what made a
         * 7-sample queue overflow.
         *
         * Letting the tick's own flush() send the accumulated batch puts up to
         * MAX_PAYLOAD_LEN into each round trip instead of one sample. Nothing
         * waits on an eager flush: telemetry.h:97 runs flush() unconditionally
         * every tick, so a partial batch is never stranded. */
    }

    /// Drop every trace of the previous connection. Without this, a stale bleBufPos and a
    /// stale waitingForAck survive into the next connection, where they either ship a
    /// fragment of the old session or wedge flush() before it ever sends.
    void onDisconnected() {
        // Host-task context: flags only. The app task tears the transfer state down when it
        // consumes sessionChanged.
        subscribed = false;
        sessionChanged = true;
        // Undelivered OTA status belongs to the session that just ended. Carrying it over would
        // hand the next client the previous one's READY/FAIL lines -- the same trap the telemetry
        // buffer avoids via sessionChanged, and it would desynchronise a host's line parser.
        {
            std::lock_guard<std::mutex> lk(otaTxMutex);
            otaTx.clear();
        }
    }

    /// App-task pump for everything that must not run on the NimBLE host task: the deferred
    /// connection-parameter change and the OTA status stream. Call once per app-task pass, after
    /// otaBleTick() so the lines it produced go out in the same pass.
    void tick() override {
        // Self-reconciling rather than hook-driven: an OTA wants a 7.5-15ms interval (the default
        // 30-60ms would stretch a 1.4MB image over minutes), and the link goes back to the relaxed
        // interval as soon as the transfer ends, however it ended.
        if (haveConn) {
            // Steady state: request NOTHING. Whatever the central negotiated is the central's
            // business, and here it is a deliberate choice -- the bench rpi widens its interval to
            // 100-200ms with a 4s supervision timeout because BLE and WiFi share one radio
            // (ble-conn-params.service). The old code overrode that with 30-60ms / 1.8s on every
            // connect, and the link died with an LL timeout (reason 546) roughly once a minute,
            // forever, which looked like flaky radio rather than the device arguing with its peer.
            //
            // The only time we ask for anything is during an OTA, where the transfer genuinely
            // needs the throughput -- and then we hand the link back when it is done.
            ConnParams want = otaBleActive() ? ConnParams::Fast
                                             : (connParamsApplied == ConnParams::Fast
                                                    ? ConnParams::Default
                                                    : connParamsApplied);
            if (want != connParamsApplied) {
                // 15-30ms during OTA, NOT the 7.5-15ms fugu uses. The bench rpi deliberately widens
                // its connection interval to 100-200ms (ble-conn-params.service) because BLE and
                // WiFi share one radio there, and asking for 7.5ms fought that mitigation: the link
                // died with an LL timeout after ~35s of transfer, reproducibly, twice. The pressure
                // for a tiny interval came from 20-byte writes anyway; with the MTU actually
                // negotiated (509-byte chunks) this is far more headroom than the transfer needs.
                if (want == ConnParams::Fast)
                    pServer->updateConnParams(connHandle, 12, 24, 0, 400);
                else
                    // Post-OTA: relax to the coex-friendly end, matching what the bench central
                    // asks for anyway. NOT the old 24/48/180, which is what caused the drops.
                    pServer->updateConnParams(connHandle, 80, 160, 0, 400);
                connParamsApplied = want;
            }
        }

        // Apply a latched aux request here rather than in the write callback: auxSet() persists to
        // NVS, and an NVS commit is a flash write -- the one thing the BLE host task must never do.
        if (auxReqPending) {
            auxReqPending = false;
            auxSet(auxReqValue);
        }
        // Aux state-change notification is handled centrally in Telemetry::update(),
        // which calls publishAux() for both BLE-originated and console-originated changes.
        // The one exception: on a reconnect (auxPublished == -1, set by onConnect), push
        // the current state so the new client sees it immediately.
        if (auxPublished == -1) {
            publishAux(auxGet());
        }

        drainOtaTx();

#ifdef BLE_STATUS_NEOPIXEL_PIN
        updateConnectionLed(haveConn);
#endif
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

    void flush() override {
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
            /* One indication carries at most one ATT payload. bleBufLen is now
             * several payloads deep, so this MUST be clamped. The tail stays
             * queued and the ack path's memmove brings it forward for the next
             * indication.
             *
             * CLAMP TO THE NEGOTIATED MTU, NOT THE REQUESTED ONE. MAX_PAYLOAD_LEN
             * is MTU-3 = 509 for the 512 we ASK for, but the comment on peerMtu
             * says plainly that we do not always get it --
             * CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256 in the prebuilt libs caps it.
             * NimBLE truncates an oversized indication to the negotiated MTU and
             * still returns success, so with peerMtu=256 only 253 of 509 bytes
             * would reach the central while the ack retired all 509: samples
             * silently lost, and the collector fed a half frame that fails CRC
             * and desynchronises its parser.
             *
             * This was latent before batching -- send() flushed eagerly, so
             * bleBufPos was usually one 64-byte sample and never approached the
             * cap. Filling the buffer is what made it reachable, which is why the
             * bound belongs here and not only in drainOtaTx() (ble.h:462), where
             * it was already being done correctly. */
            const uint16_t attPayload = peerMtu > 3 ? (uint16_t) (peerMtu - 3) : 20;
            const uint16_t cap = attPayload < MAX_PAYLOAD_LEN ? attPayload : MAX_PAYLOAD_LEN;
            const uint16_t sendLen = bleBufPos > cap ? cap : bleBufPos;
            /* DEBUG, not INFO: one line per indication is ~2/s forever and it
             * buried the sampler summaries. Nothing is lost by hiding it --
             * every case worth acting on has its own WARNING (indicate failed,
             * no-ack re-arm, buffer full with a subscriber attached), and drops
             * are counted in nDroppedBacklog rather than inferred from this.
             * Rebuild with -DCORE_DEBUG_LEVEL=4 to watch batching behaviour. */
            ESP_LOGD("ble", "sending %hu of %hu queued", sendLen, bleBufPos);
            //pCharacteristic->setValue(const_cast<uint8_t *>(buf), len);
            bool res = pCharacteristic->indicate(bleBuf, sendLen);
            if (!res) {
                ESP_LOGW("ble", "BLE server indicate failed");
            } else {
                inFlightLen = sendLen;
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
            // Accept whatever the central negotiated; tick() requests nothing in steady state.
            srv->connHandle = connInfo.getConnHandle();
            srv->haveConn = true; // set last: it is what gates tick()
            srv->auxPublished = -1; // force tick() to re-publish for the new client
        }

        void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
            Serial.printf("Client disconnected (reason %d) - start advertising\n", reason);
            srv->onDisconnected();
            srv->haveConn = false;
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
        /* DEBUG for the same reason as flush()'s send line: this fires once per
         * indication, pairing with it at ~2/s. A FAILED ack is the interesting
         * case and it is handled below on its own path, not inferred from here. */
        ESP_LOGD("ble", "onAck done=%i waiting=%hhu", (int)done, waitingForAck);
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

    /// Aux switch control. Host-task context: parse and latch only -- applying it writes NVS.
    class AuxCallbacks : public NimBLECharacteristicCallbacks {
        BleSrv *srv;

    public:
        explicit AuxCallbacks(BleSrv *srv) : srv{srv} {
        }

        void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
            std::string v = pCharacteristic->getValue();
            bool want;
            if (!auxParseCommand((const uint8_t *) v.data(), v.size(), want)) {
                ESP_LOGW("aux", "unparseable aux write (%u bytes), ignored", (unsigned) v.size());
                // Re-publish so the characteristic value never reflects the garbage that was
                // written into it, and the client can see the switch did not move.
                srv->publishAux(auxGet());
                return;
            }
            srv->auxReqValue = want;
            srv->auxReqPending = true;
        }
    } auxCallbacks;

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
