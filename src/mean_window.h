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
  float getMax() const {
      return max;
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