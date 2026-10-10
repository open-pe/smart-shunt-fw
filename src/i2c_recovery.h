#pragma once

#include <cstdint>

/// When to run an I2C bus recovery. Pure logic, no Arduino: test/host/ runs it on the host.
///
/// A recovery is due once the bus has failed at least MIN_FAILS times in a row AND the
/// streak has lasted MIN_SPAN_US. Both, because each alone misfires:
///  - count only: a burst of NACKs from one absent part is not a bus fault;
///  - time only: one stale failure followed by a second one much later is not a streak.
/// The two INA228 pollers hit the bus every 50 ms, so a real upset trips this in ~200 ms.
///
/// Neither rule separates boot probing: TMP117::init() reads DEVICE_ID, waits 50 ms and
/// retries, so four absent straps are eight failures over ~210 ms -- a valid trigger. So
/// failures are not counted at all while `probing` is set (setup, around sampler init).
///
/// After every attempt the next one waits a backoff that doubles up to BACKOFF_MAX_US,
/// so a bus that stays dead costs one recovery (and one log line) per 30 s, not a spin.
/// A success ends the streak, but resets the backoff and the deadline only once the bus
/// has gone failure-free for min(backoff, HEALTHY_MAX_US): an upset that keeps coming
/// back within that keeps backing off, while one after a healthy stretch -- even after a
/// long dead one -- is recovered as promptly as the first.
struct I2cRecoveryPolicy {
    static constexpr uint32_t MIN_FAILS = 4;
    static constexpr int64_t MIN_SPAN_US = 200000;
    static constexpr int64_t BACKOFF_MIN_US = 500000;
    static constexpr int64_t BACKOFF_MAX_US = 30000000;
    static constexpr int64_t HEALTHY_MAX_US = 2000000;

    bool probing = false;
    uint32_t streak = 0;
    int64_t firstFailUs = 0;
    int64_t lastFailUs = 0;
    int64_t notBeforeUs = 0;
    int64_t backoffUs = BACKOFF_MIN_US;

    void onSuccess(int64_t nowUs) {
        streak = 0;
        const int64_t healthyUs = backoffUs < HEALTHY_MAX_US ? backoffUs : HEALTHY_MAX_US;
        if (nowUs - lastFailUs >= healthyUs) {
            backoffUs = BACKOFF_MIN_US;
            notBeforeUs = 0;
        }
    }

    /// Records a failed transfer; true when a recovery should run now.
    bool onFailure(int64_t nowUs) {
        if (probing) return false;
        lastFailUs = nowUs;
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
