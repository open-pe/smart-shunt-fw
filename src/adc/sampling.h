#pragma once

#ifdef TARGET_STM32H5
#include "esp_compat.h"
#else
#include <sys/time.h>
#endif
#include <cmath>


unsigned long long getTimeStamp(struct timeval *tv, int secFracDigits);

// minimize memory footprint
// - use raw ADC samples
// - remove p, e
// - instead of timeval store dt to the prev sample (uint_16)
#pragma pack(push, 1)
struct Sample {
  float u{NAN}, i{NAN}, p_{NAN}, e{NAN};
  unsigned long long t{0};  // 8byte
  float temp{NAN}; // adc die temperature
  uint32_t diag{0}; // encoded diagnostic: reason<<24 | sign<<20 | adc_code&0xFFFFF; 0=none

  inline float p() const {
      if(std::isnan(p_)) return u * i;
      else return p_;
  }

  void setTimeNow() {
    struct timeval u_time;
    gettimeofday(&u_time, NULL);
    t = getTimeStamp(&u_time, 3);
  }
};
#pragma pack(pop)

class PowerSampler {
  public:
  virtual bool init() = 0;
  virtual void startReading() = 0;
  virtual bool hasData() = 0;
  virtual Sample getSample() = 0;
  virtual uint8_t getStorageId() const = 0;

  /// Does this sampler measure power at all?  Only samplers that do get a vote in
  /// the idle-sleep decision (see looksActive()/noteWakeEvent() in main.cpp), which
  /// reads a non-finite mean power as "cannot judge -> stay awake".  For a power
  /// channel that is the right call: NaN there means a disabled channel or a failed
  /// read, i.e. a transient inability to judge.  For a temperature-only sampler it
  /// is not -- its mean power is NaN by construction, forever, so letting it vote
  /// would permanently pin the whole board awake.
  /// Defaults to true so a sampler must opt OUT deliberately: the failure direction
  /// of a wrong answer here is "stays awake", never "sleeps while there is data".
  virtual bool measuresPower() const { return true; }
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