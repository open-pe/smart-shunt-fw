#pragma once

#include <sys/time.h>


unsigned long long getTimeStamp(struct timeval *tv, int secFracDigits);

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
  public:
  virtual bool init() = 0;
  virtual void startReading() = 0;
  virtual bool hasData() = 0;
  virtual Sample getSample() = 0;  
};



unsigned long long getTimeStamp(struct timeval *tv, int secFracDigits); // defined in influxdb client
/*
unsigned long long getTimeStamp(struct timeval *tv, int secFracDigits) {
    unsigned long long tsVal = 0;
    switch(secFracDigits) {
        case 0:
            tsVal = tv->tv_sec;
            break;
        case 6:
            tsVal = tv->tv_sec * 1000000LL + tv->tv_usec;
            break;
        case 9:
            tsVal = tv->tv_sec * 1000000000LL + tv->tv_usec * 1000LL;
            break;
        case 3:
        default:
            tsVal = tv->tv_sec * 1000LL + tv->tv_usec / 1000LL;
            break;

    }
    return tsVal;
}
*/