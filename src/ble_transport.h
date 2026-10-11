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

    /// Number of telemetry subscriptions accepted since boot. A change means a (new)
    /// collector is listening, so per-session state such as the T-offset audit frames
    /// (temp_offset.h) must be sent again. 0 forever on a transport with no subscriptions.
    virtual uint32_t telemetrySessions() const { return 0; }

    /// Sweep hook: ask the central for these steady-state parameters (LL units). A
    /// transport with no negotiable link ignores it.
    virtual void requestConnParams(uint16_t itvlMin, uint16_t itvlMax, uint16_t latency) {
        (void) itvlMin, (void) itvlMax, (void) latency;
    }

    /* REMOTE CONSOLE.
     *
     * A transport that can carry console commands latches an inbound line on
     * whatever task received it and hands it over here, on the app task, where
     * handleConsoleInput() may safely touch NVS, WiFi and the relay state machine.
     * Default: no such channel, so nothing is ever taken and the caller's loop
     * costs one virtual call per pass.
     *
     * consoleWrite() receives the reply as it is produced (installed as the
     * UART_LOG sink for the duration of the command), and consoleReplyDone()
     * closes the exchange -- the host needs an explicit end marker, because a
     * command that legitimately prints nothing is otherwise indistinguishable
     * from one that hung.
     */
    virtual bool takeConsoleCommand(char *out, size_t cap) {
        (void) out, (void) cap;
        return false;
    }

    virtual void consoleWrite(const char *text, size_t len) { (void) text, (void) len; }

    /// `ok` is "the line was dispatched", not "the command succeeded" --
    /// handleConsoleInput() returns void and reports its own outcome in the text.
    virtual void consoleReplyDone(bool ok) { (void) ok; }

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
