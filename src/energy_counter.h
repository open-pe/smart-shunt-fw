#include <Arduino.h>

#include "adc/sampling.h"
#include "math.h"
#include "util.h"

class EnergyCounter
{
public:
  PowerSampler *sampler;
  std::string name;

  unsigned long NumSamples = 0;
  unsigned long NSamplesLastSummary = 0;

  double Energy = 0;
  float LastP = 0.0f;

  unsigned long startTime = 0;
  unsigned long lastTime = 0;
  unsigned long maxDt = 0;
  unsigned long numTimeouts = 0;

  struct MeanWindow winI
  {
  };
  struct MeanWindow winU
  {
  };
  struct MeanWindow winP
  {
  };

  unsigned long long windowTimestamp = 0;

  bool hfWrites = false;

public:
  EnergyCounter(PowerSampler *sampler, const std::string &name) : sampler(sampler), name(name) {}
  EnergyCounter(const EnergyCounter &) = delete; // no copy
  EnergyCounter(EnergyCounter &&) = default;

  void update()
  {
    unsigned long nowTime = micros();

    PowerSampler &ps(*sampler);
    if (ps.hasData())
    {
      Sample s = ps.getSample();

      unsigned long nowTime = micros();
      auto P = s.p();
      if (lastTime != 0)
      {
        // we use simple trapezoidal rule here
        unsigned long dt_us = nowTime - lastTime;
        Energy += (double)((LastP + P) * 0.5f * (dt_us * (1e-6f / 3600.f)));
        s.e = Energy;

        if (dt_us > maxDt)
          maxDt = dt_us;
      }
      else
      {
        s.e = 0.0f;
        startTime = nowTime;
      }

      ++NumSamples;
      lastTime = nowTime;
      LastP = P;

      winI.add(s.i);
      winU.add(s.u);
      winP.add(s.p());
      windowTimestamp = s.t;
    }
    else
    {
      auto lt = lastTime; // capture
      if (nowTime > lt && (nowTime - lt) > 4e6)
      {
        ++numTimeouts;
        Serial.println("");
        Serial.println("Timeout waiting for new sample!");
        Serial.println((nowTime - lt) * 1e-6);
        Serial.println(lt);
        Serial.println(nowTime);
        Serial.println("");
        ps.startReading();
      }
    }
  }

  void summary(unsigned long dt_us)
  {

    // capture
    auto nSamples = NumSamples;
    auto energy = Energy;
    float i_mean = winI.pop(), u_mean = winU.pop(), p_mean = winP.pop();

    // compute
    float sps = (nSamples - NSamplesLastSummary) / (dt_us * 1e-6);

    if (!hfWrites)
    {
      Point point("smart_shunt");
      point.addTag("device", name.c_str());
      point.addField("I", i_mean, 4);
      point.addField("U", u_mean, 4);
      point.addField("P", p_mean, 4);
      point.addField("E", energy, 4);
      point.setTime(windowTimestamp);
      influxWritePointsUDP(&point, 1);
      // client.writePoint(point);
    }

    printTime();
    Serial.print(name.c_str());
    Serial.print(": U=");
    Serial.print(u_mean, 4);
    Serial.print("V, I=");
    Serial.print(i_mean, 3);
    Serial.print("A, P=");
    Serial.print(p_mean, 3);
    Serial.print("W, E=");
    Serial.print(energy, 3);
    Serial.print("Wh, N=");
    Serial.print(nSamples);
    Serial.print(", SPS=");
    Serial.print(sps, 1);
    // Serial.print(", T=");
    // Serial.print((nowTime - startTime) * 1e-6, 1);
    // Serial.print("s");
    Serial.print(", maxDt=");
    Serial.print(maxDt);
    // Serial.print(", numDropped=");
    // Serial.print(numDropped);
    Serial.println();

    NSamplesLastSummary = nSamples;
  }
};
