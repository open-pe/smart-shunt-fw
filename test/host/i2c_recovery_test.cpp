// Host test for the I2C recovery trigger (src/i2c_recovery.h). No Arduino, no PlatformIO:
//   c++ -std=c++17 -Wall -I src test/host/i2c_recovery_test.cpp -o /tmp/i2c_recovery_test && /tmp/i2c_recovery_test
#include "i2c_recovery.h"

#include <cstdio>
#include <cstdlib>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

using P = I2cRecoveryPolicy;

// Feed one failure every `stepUs` from `t`; return the time of the first trigger, or -1.
static int64_t firstTrigger(P &p, int64_t &t, int64_t stepUs, int maxFails) {
    for (int i = 0; i < maxFails; ++i, t += stepUs)
        if (p.onFailure(t)) return t;
    return -1;
}

int main() {
    {   // Boot probes of absent TMP117s, as TMP117::init() really spaces them: DEVICE_ID
        // read, delay(50), retry, ~2 ms to the next strap. Eight failures over ~210 ms.
        auto probeAbsentTmp117s = [](P &p) {
            int64_t t = 1000000;
            bool tripped = false;
            for (int addr = 0; addr < 4; ++addr) {
                tripped |= p.onFailure(t);
                tripped |= p.onFailure(t += 51000);
                t += 2000;
            }
            return tripped;
        };
        P unguarded;
        CHECK(probeAbsentTmp117s(unguarded)); // why `probing` exists: this IS a valid streak
        P p;
        p.probing = true;
        CHECK(!probeAbsentTmp117s(p));
        CHECK(p.streak == 0);
        p.probing = false;                    // a real upset after setup still trips
        int64_t t0 = 2000000, t = t0;
        CHECK(firstTrigger(p, t, 25000, 100) - t0 == P::MIN_SPAN_US);
    }
    {   // Two pollers at 50 ms: sustained failure trips after MIN_SPAN_US, not before.
        P p;
        int64_t t0 = 1000000, t = t0;
        const int64_t at = firstTrigger(p, t, 25000, 100);
        CHECK(at - t0 == P::MIN_SPAN_US);
    }
    {   // Few failures spread out: needs MIN_FAILS as well as the span.
        P p;
        int64_t t = 0;
        CHECK(!p.onFailure(t));
        CHECK(!p.onFailure(t += 300000));
        CHECK(!p.onFailure(t += 300000));
        CHECK(p.onFailure(t += 300000));
    }
    {   // A success anywhere in the streak restarts it.
        P p;
        int64_t t = 0;
        for (int i = 0; i < 3; ++i) p.onFailure(t += 100000);
        p.onSuccess(t += 1000);
        CHECK(!p.onFailure(t += 100000));
        CHECK(p.streak == 1);
    }
    {   // Dead bus: attempts are spaced by a doubling backoff, capped, never a spin.
        P p;
        int64_t t = 0, last = -1;
        int64_t gaps[12];
        for (int n = 0; n < 12; ++n) {
            const int64_t at = firstTrigger(p, t, 10000, 100000);
            CHECK(at >= 0);
            if (last >= 0) gaps[n - 1] = at - last;
            last = at;
            p.onRecoveryAttempt(at);
        }
        CHECK(gaps[0] >= P::BACKOFF_MIN_US);
        for (int n = 1; n < 11; ++n) {
            CHECK(gaps[n] >= gaps[n - 1]);                          // monotonic
            CHECK(gaps[n] <= P::BACKOFF_MAX_US + P::MIN_SPAN_US + 10000); // capped
        }
        CHECK(gaps[10] >= P::BACKOFF_MAX_US);
    }
    {   // Dead bus, then recovered and healthy: an upset 5 s later is recovered promptly,
        // not after the 30 s deadline the dead stretch left behind.
        P p;
        int64_t t = 0, at = 0;
        for (int n = 0; n < 12; ++n) {                  // dead long enough to max the backoff
            at = firstTrigger(p, t, 25000, 100000);
            p.onRecoveryAttempt(at);
        }
        CHECK(p.backoffUs == P::BACKOFF_MAX_US);
        CHECK(p.notBeforeUs >= at + P::BACKOFF_MAX_US / 2);
        for (t = at + 25000; t < at + 5000000; t += 25000) p.onSuccess(t); // healthy 5 s
        const int64_t t0 = t;
        const int64_t again = firstTrigger(p, t, 25000, 100000);
        CHECK(again - t0 == P::MIN_SPAN_US);
        CHECK(p.backoffUs == P::BACKOFF_MIN_US);
    }
    {   // Intermittent upsets: 200 ms of failures, 500 ms healthy, repeating. One success in
        // between must not reset the backoff -- attempts have to spread out, not recur every
        // 0.7 s forever.
        P p;
        int64_t t = 0;
        int64_t attempts[16];
        int n = 0;
        for (int cycle = 0; cycle < 200 && n < 16; ++cycle) {
            for (int i = 0; i < 9; ++i, t += 25000)
                if (p.onFailure(t)) { p.onRecoveryAttempt(t); attempts[n++] = t; break; }
            for (int i = 0; i < 20; ++i) p.onSuccess(t += 25000);
        }
        CHECK(n >= 6);
        for (int k = 2; k < n; ++k)
            CHECK(attempts[k] - attempts[k - 1] >= attempts[k - 1] - attempts[k - 2]); // monotonic
        CHECK(attempts[n - 1] - attempts[n - 2] >= 8 * 700000);                      // backed off
    }
    {   // A long healthy stretch resets the backoff entirely.
        P p;
        int64_t t = 0;
        for (int n = 0; n < 6; ++n) p.onRecoveryAttempt(t += 1);
        CHECK(p.backoffUs == P::BACKOFF_MAX_US);
        p.onSuccess(t += P::HEALTHY_MAX_US - 10);
        CHECK(p.backoffUs == P::BACKOFF_MAX_US); // not yet
        p.onSuccess(t += P::HEALTHY_MAX_US);
        CHECK(p.backoffUs == P::BACKOFF_MIN_US);
        CHECK(p.notBeforeUs == 0);
    }
    std::printf(failures ? "%d FAILED\n" : "ok\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
