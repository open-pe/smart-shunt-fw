#pragma once
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
  void add(float x) {
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