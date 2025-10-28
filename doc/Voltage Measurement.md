
* precision resistors for voltage divider
  * low temp drift (even if temp drift would cancel out)
  * time drift
* consider input cap as buffer to reduce noise
* single ended
  * using a differential ADC with negative input tied to ground halves the dynamic range
    * use offset voltage
    * use instrumentation amp (THS4551 from XDS110ET)
* consider ADC input common mode contraints
  * usually ADC with pga need some voltage margin
  * however, for voltage measurements above the 1V range PGA can usually be bypassed
  * measurement ground might go below AGND. there are ADCs that support common modes <-1V below AGND
* generally no noise issue


![img.png](img/img.png)
* INA228 devices appear to have similar distortion
* ADS1220 has a lower INL
  * 5k/200k voltage divider
  * until 1.22V error is <20ppm
  * TODO
    * try PGA gain = 1, 2


monolythic resistor network
monolythic thin film array


![img_7.png](img/img_7.png)
https://youtu.be/D28uSzCs7-k?si=xbc2kDCH82Io2PJb&t=880
vhp101

vishay vsp 1442,1445,1446




Y0115V0720VV9L 500/69.5k, ±0.1ppm/°C
Y0115V0549AA0L 1k/100k
Y0115V0635VV9L 1.4k, 60.2k


