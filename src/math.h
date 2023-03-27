#pragma once

struct MeanWindow {
  // TODO trapezoidal sum?
  float sum;
  uint32_t num;

  float getMean() const {
    return sum / num;
  }
  void clear() {
    sum = 0.f;
    num = 0.f;
  }
  void add(float x) {
    sum += x;
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