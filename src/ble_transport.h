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
