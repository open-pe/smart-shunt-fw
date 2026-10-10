#pragma once

#include <cstdint>

/// When to run an I2C bus recovery. Pure logic, no Arduino: test/host/ runs it on the host.
///
/// A recovery is due once the bus has failed at least MIN_FAILS times in a row AND the
/// streak has lasted MIN_SPAN_US. Both, because each alone misfires:
///  - count only: boot probes of absent parts (four TMP117 straps, two reads each) are
///    eight NACKs in a few ms, and are not a bus fault;
///  - time only: one stale failure followed by a second one much later is not a streak.
/// The two INA228 pollers hit the bus every 50 ms, so a real upset trips this in ~200 ms.
///
/// After every attempt the next one waits a backoff that doubles up to BACKOFF_MAX_US,
/// so a bus that stays dead costs one recovery (and one log line) per 30 s, not a spin.
/// Any successful transfer resets the streak and the backoff.
struct I2cRecoveryPolicy {
    static constexpr uint32_t MIN_FAILS = 4;
    static constexpr int64_t MIN_SPAN_US = 200000;
    static constexpr int64_t BACKOFF_MIN_US = 500000;
    static constexpr int64_t BACKOFF_MAX_US = 30000000;

    uint32_t streak = 0;
    int64_t firstFailUs = 0;
    int64_t notBeforeUs = 0;
    int64_t backoffUs = BACKOFF_MIN_US;

    void onSuccess() {
        streak = 0;
        backoffUs = BACKOFF_MIN_US;
    }

    /// Records a failed transfer; true when a recovery should run now.
    bool onFailure(int64_t nowUs) {
        if (streak++ == 0) firstFailUs = nowUs;
        return streak >= MIN_FAILS && nowUs - firstFailUs >= MIN_SPAN_US && nowUs >= notBeforeUs;
    }

    /// Call after every attempt, successful or not.
    void onRecoveryAttempt(int64_t nowUs) {
        streak = 0;
        notBeforeUs = nowUs + backoffUs;
        backoffUs = backoffUs * 2 > BACKOFF_MAX_US ? BACKOFF_MAX_US : backoffUs * 2;
    }
};
