#pragma once

#include <cstdint>
#include <cstddef>

class BleTransport {
public:
    virtual ~BleTransport() = default;
    virtual void begin() = 0;
    virtual void send(const uint8_t *data, size_t len) = 0;
    virtual void flush() = 0;
    virtual void tick() = 0;
    virtual bool isConnected() const = 0;
    virtual void publishAux(bool on) = 0;

    /// Sweep hook: ask the central for these steady-state parameters (LL units). A
    /// transport with no negotiable link ignores it.
    virtual void requestConnParams(uint16_t itvlMin, uint16_t itvlMax, uint16_t latency) {
        (void) itvlMin, (void) itvlMax, (void) latency;
    }

    /// Negotiated link parameters, for status reporting. Not pure: a transport with no
    /// real link (the STM32 stub, the UART bridge) has nothing to report and says so by
    /// returning false, rather than every such transport having to stub it out.
    /// intervalMs/timeoutMs are in milliseconds; latency counts skippable events.
    virtual bool connSnapshot(float &intervalMs, uint16_t &latency, uint16_t &timeoutMs) const {
        (void) intervalMs, (void) latency, (void) timeoutMs;
        return false;
    }
};

class StubBleTransport : public BleTransport {
public:
    void begin() override {}
    void send(const uint8_t *, size_t) override {}
    void flush() override {}
    void tick() override {}
    bool isConnected() const override { return false; }
    void publishAux(bool) override {}
};
