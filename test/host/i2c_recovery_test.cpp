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
    {   // Boot probes of absent parts: many NACKs within a few ms are not a bus fault.
        P p;
        int64_t t = 1000000;
        CHECK(firstTrigger(p, t, 500, 8) == -1);
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
        p.onSuccess();
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
    {   // Success resets the backoff, so the next upset is handled promptly again.
        P p;
        int64_t t = 0;
        for (int n = 0; n < 6; ++n) p.onRecoveryAttempt(t += 1);
        CHECK(p.backoffUs == P::BACKOFF_MAX_US);
        p.onSuccess();
        CHECK(p.backoffUs == P::BACKOFF_MIN_US);
    }
    std::printf(failures ? "%d FAILED\n" : "ok\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
