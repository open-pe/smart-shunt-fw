
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