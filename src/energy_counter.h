#include <Arduino.h>

#include <utility>

#include "adc/sampling.h"
#include <cmath>
#include "util.h"
#include "math.h"

#include "readerwriterqueue.h"

template<typename T>
struct UIP {
    T U{}, I{}, P{}, Temp{};

    template<typename = void>
    void clear() {
        U.clear();
        I.clear();
        P.clear();
        Temp.clear();
    }
};

class EnergyCounter {
    moodycamel::ReaderWriterQueue<Sample> sampleQueue{};
public:
    PowerSampler *sampler;
    std::string name;

private:
    unsigned long NumSamples = 0;
    unsigned long NSamplesLastSummary = 0;
    unsigned long NSamplesLastPrint = 0;
    unsigned long tLastPrint = 0;

    double Energy = 0;//Wh
    float LastP = 0.0f;

    float TotalCharge = 0; //Ah
    float LastI = 0.0f;

    unsigned long startTime = 0;
    unsigned long lastTime = 0;
    unsigned long maxDt = 0;
    unsigned long numTimeouts = 0;
    uint32_t numTimeoutsStreak = 0;

public:
    UIP<MeanWindow> winPoint{};
    UIP<MeanWindow> winPrint{};


    unsigned long long windowTimestamp = 0;

    float calibFactorU = 1, calibFactorI = 1;
private:
    const uint8_t eePromIndex;

    unsigned long maxDtReported = 0;

public:
    EnergyCounter(PowerSampler *sampler, std::string name_, size_t eePromIndex_) : sampler(sampler),
                                                                                   name(std::move(name_)),
                                                                                   eePromIndex(eePromIndex_) {
        if (readCalibrationFactors(eePromIndex_, calibFactorU, calibFactorI)) {
            UART_LOG("%s read calibration factors U/I = (%.8f/%.8f)", name.c_str(), calibFactorU, calibFactorI);
        }
    }

    EnergyCounter(const EnergyCounter &) = delete; // no copy
    EnergyCounter(EnergyCounter &&)  = default;

    void setCalibrationFactors(float u, float i, bool multiply) {
        if (!std::isnan(u))calibFactorU = (multiply ? calibFactorU : 1) * u;
        if (!std::isnan(i)) calibFactorI = (multiply ? calibFactorI : 1) * i;

        storeCalibrationFactors(eePromIndex, calibFactorU, calibFactorI);
    }

    void update() {
        PowerSampler &ps(*sampler);
        if (ps.hasData()) {
            Sample s = ps.getSample();
            unsigned long nowTime = micros(); // capture timestamp after value capture

            s.u *= calibFactorU;
            s.i *= calibFactorI;

            auto P = s.p();
            if (lastTime != 0) {
                // we use simple trapezoidal rule here
                unsigned long dt_us = nowTime - lastTime;
                Energy += (double) ((LastP + P) * 0.5f * (dt_us * (1e-6f / 3600.f)));
                s.e = (float) Energy;


                TotalCharge += (double) ((LastI + s.i) * 0.5f * (dt_us * (1e-6f / 3600.f)));
                //s.c = (float) TotalCharge;

                if (dt_us > maxDt)
                    maxDt = dt_us;
            } else {
                s.e = 0.0f;
                startTime = nowTime;
            }

            lastTime = nowTime;
            LastP = P;
            LastI = s.i;

            sampleQueue.emplace(s);
            auto qs = sampleQueue.size_approx();
            if(qs > 250 && qs < 355) {
                ESP_LOGW("ec", "Sample queue is growing beyond 250: %u", qs);
            }

            if (numTimeoutsStreak > 0) numTimeoutsStreak = 0;
        } else {
            unsigned long nowTime = micros();
            auto lt = lastTime; // capture
            if (nowTime > lt && (nowTime - lt) > 4e6 && nowTime > 10e6) {
                ++numTimeouts;
                ++numTimeoutsStreak;
                ESP_LOGW("ec", "\n%s Timeout waiting for new sample! %u %f %u %u \n",
                         name.c_str(),
                         numTimeoutsStreak,
                         (nowTime - lt) * 1e-6, lt, nowTime);
                ps.startReading();
                auto sl = min(100U, numTimeoutsStreak);
                delay(sl * sl);
            }
        }
    }

    void consumeQueue() {
        // TODO this is a bit inefficient
        // better to create the window summary inside the RT task and publish this
        Sample s{};
        while(sampleQueue.try_dequeue(s)) {
            //ESP_LOGD("ec", "DEQ!");
            auto P = s.p();

            winPoint.I.add(s.i);
            winPoint.U.add(s.u);
            winPoint.P.add(P);
            winPoint.Temp.add(s.temp);

            winPrint.I.add(s.i);
            winPrint.U.add(s.u);
            winPrint.P.add(P);
            winPrint.Temp.add(s.temp);

            windowTimestamp = s.t;

            ++NumSamples;
        }
    }

    bool newSamplesSinceLastSummary() const {
        return NumSamples > NSamplesLastSummary;
    }

    Sample printSample{};

    Point summary(unsigned long dt_us,  bool print) {

        // capture
        auto nSamples = NumSamples;
        auto energy = Energy;
        float i_max = winPoint.I.getMax(), u_max = winPoint.U.getMax();
        float i_mean = winPoint.I.pop(), u_mean = winPoint.U.pop(), p_mean = winPoint.P.pop();
        float temp_mean = winPoint.Temp.pop();



        Point point("smart_shunt");
        point.addTag("device", name.c_str());
        if(nSamples != NSamplesLastSummary) {
            point.addField("I", i_mean, 6);
            point.addField("U", u_mean, 6);
            point.addField("I_max", i_max, 3);
            point.addField("U_max", u_max, 3);
            point.addField("P", p_mean, 6);
            point.addField("E", energy, 4);
            point.addField("T", temp_mean, 2);
            if (maxDt > maxDtReported) {
                point.addField("dt_max", (float) maxDt * 1e-3f, 2);
                maxDtReported = maxDt;
            }

            if (numTimeouts)
                point.addField("timeouts", numTimeouts);
        }

        point.setTime(windowTimestamp);

        // client.writePoint(point);

        if (print) {
            auto now = micros();
            // compute
            float sps = (float)(nSamples - NSamplesLastPrint) / ((float)(now - tLastPrint) * 1e-6f);

            printSample.u = winPrint.U.pop();
            printSample.i = winPrint.I.pop();
            printSample.e = (float) energy;
            UART_LOG("%s %s U=%7.4fV I=%7.4fA P=%6.3fW, E=%6.3fWh, T=%2.1f° N=%lu, sps=%.1f, maxDt=%.2fms",
                     timeStr().c_str(), name.c_str(), printSample.u, printSample.i, winPrint.P.pop(), energy,
                     winPrint.Temp.pop(), nSamples, sps, maxDt * 1e-3f);

            // Serial0.print(", T=");
            // Serial0.print((nowTime - startTime) * 1e-6, 1);
            // Serial0.print("s");
            // Serial0.print(", numDropped=");
            // Serial0.print(numDropped);
            // Serial0.println();

            tLastPrint = now;
            NSamplesLastPrint = nSamples;
        }

        NSamplesLastSummary = nSamples;

        return point;
    }

    void reset() {
        NumSamples = 0;
        NSamplesLastSummary = 0;

        Energy = 0;
        LastP = 0.0f;

        startTime = 0;
        lastTime = 0;
        maxDt = 0;
        numTimeouts = 0;

        winPoint.clear();
        winPrint.clear();
    }
};
