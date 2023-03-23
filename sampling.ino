
// minimize memory footprint
// - use raw ADC samples
// - remove p, e
// - instead of timeval store dt to the prev sample (uint_16)
struct Sample {
  float u, i, e;
  unsigned long long t;  // 8byte

  inline float p() const {
    return u * i;
  }

  void setTimeNow() {
      struct timeval u_time;
      gettimeofday(&u_time, NULL);
      t = getTimeStamp(&u_time, 3);    
  }
};

class PowerSampler {

};



struct EnergyCounter {
  PowerSampler *sampler;

  unsigned long NumSamples = 0;
  unsigned long NSamplesLastSummary = 0;

  unsigned long lastTime = 0;

  struct MeanWindow winI {};
  struct MeanWindow winU {};
  struct MeanWindow winP {};

  unsigned long long windowTimestamp = 0;

  void update() {
    unsigned long nowTime = micros();

    PowerSampler &ps(*sampler);
    if (ps.hasData()) {
      Sample s = ps.getSample();

      unsigned long nowTime = micros();
      auto P = s.p();
      if (lastTime != 0) {
        // we use simple trapezoidal rule here
        unsigned long dt_us = nowTime - lastTime;
        Energy += (double)((LastP + P) * 0.5f * (dt_us * (1e-6f / 3600.f)));
        s.e = Energy;
      } else {
        s.e = 0.0f;
      }

      ++NumSamples;
      lastTime = nowTime;
      LastP = P;

      winI.add(s.i);
      winU.add(s.u);
      winP.add(s.p());
      windowTimestamp = s.t;
    } else {
      auto lastTime = lastTime;
      if (nowTime > lastTime && (nowTime - lastTime) > 4e6) {
        Serial.println("");
        Serial.println("Timeout waiting for new sample!");
        Serial.println((nowTime - lastTime) * 1e-6);
        Serial.println(lastTime);
        Serial.println(nowTime);
        Serial.println("");
        startReading();  // TODO this is for some reason not necessary
      }
    }
  }

  void summary(unsigned long dt) {

    // capture
    auto nSamples = NumSamples;
    auto energy = Energy;

    // compute
    float sps = (nSamples - NSamplesLastSummary) / (dt * 1e-6);
    float i_mean = MeanI.pop(), u_mean = MeanU.pop(), p_mean = MeanP.pop();


    if (!hfWrites) {
      Point point("smart_shunt");
      point.addTag("device", "ESP8266_proto1");
      point.addField("I", i_mean, 4);
      point.addField("U", u_mean, 4);
      point.addField("P", p_mean, 4);
      point.addField("E", energy, 4);
      point.setTime(windowTimestamp);
      influxWritePointsUDP(&point, 1);
      //client.writePoint(point);
    }

    printTime();
    Serial.print("U=");
    Serial.print(u_mean, 4);
    Serial.print("V, I=");
    Serial.print(i_mean, 3);
    Serial.print("A, P=");
    Serial.print(p_mean, 3);
    Serial.print("W, E=");
    Serial.print(energy, 3);
    Serial.print("Wh, N=");
    Serial.print(NumSamples);
    Serial.print(", SPS=");
    Serial.print(sps, 1);
    Serial.print(", T=");
    Serial.print((nowTime - StartTime) * 1e-6, 1);
    Serial.print("s");
    Serial.print(", maxBufSize=");
    Serial.print(maxBufSize);
    Serial.print(", numDropped=");
    Serial.print(numDropped);
    //Serial.print(adc2, BIN);
    Serial.println();

    NSamplesLastSummary = nSamples;
  }
};
