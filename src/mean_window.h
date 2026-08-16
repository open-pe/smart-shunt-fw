#pragma once
#include <cmath>
#include <limits>

struct MeanWindow {
  // TODO trapezoidal sum?
  float sum;
  float max;
  uint32_t num;

  float getMean() const {
    return sum / num;
  }
  /// NAN when no finite sample was ever added, NOT the lowest() sentinel.
  ///
  /// add() compares with `x > max`, which is false for NaN, so a window fed only
  /// NaN keeps the sentinel. Returning it raw published I_max/U_max as
  /// -3.4028235e38 for every temperature-only sampler: finite, so
  /// Point::addField() did not drop it the way it drops the NaN mean, and InfluxDB
  /// got -3.4e38 as a real reading -- enough to wreck any auto-scaled dashboard
  /// sharing the measurement. The sentinel means "no data", so say NAN and let the
  /// existing NaN handling downstream drop the field.
  float getMax() const {
      return max == std::numeric_limits<float>::lowest() ? NAN : max;
  }

  void clear() {
    sum = 0.f;
    num = 0.f;
    max = std::numeric_limits<float>::lowest();
  }
  /// Non-finite input is IGNORED, not accumulated.
  ///
  /// `sum += NAN` poisons the entire window: one failed read out of eight made the
  /// whole 400 ms mean NaN and discarded seven good samples with it. That is how a
  /// TMP117 that missed a single I2C read published a summary with no usable field
  /// in it at all -- the collector then emitted line protocol with an empty field
  /// section, which InfluxDB rejects outright.
  ///
  /// A dropout is missing evidence about ONE sample. It is not evidence about the
  /// other samples in the window, and it must not delete them.
  ///
  /// A window that saw no finite sample at all still reports NAN, through 0/0 in
  /// getMean() -- "nothing was measured" must never come out as 0. Samplers that
  /// legitimately have no value for a channel (a temperature-only part has no I or
  /// U) keep reporting NAN for it exactly as before, since every sample they add is
  /// non-finite and num stays 0.
  void add(float x) {
    if (!std::isfinite(x)) return;
    sum += x;
    if(x > max) max = x;
    ++num;
  }

  float pop() {
    float m = getMean();
    clear();
    return m;
  }

  MeanWindow() {
    clear();
  }
};