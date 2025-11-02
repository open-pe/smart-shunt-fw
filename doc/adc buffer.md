* low offset for high linearity
* low drift
* low noise
* low input bias current / high input impedance
* low output impedance (<1Ω)

two stage buffer:

* input stage has low offset voltage and low input bias current
* output stage has low output impedance
* output stage offset voltage is compensated because it is part of the feedback loop

|               | offset       | offset<br/>Drift µV/K | noise<br/>.1Hz nVp-p | CMRR | inp Z  | out Z <br/>@DC | apps                         |   |   |                                                                                   |
|---------------|--------------|-----------------------|----------------------|------|--------|----------------|------------------------------|---|---|-----------------------------------------------------------------------------------|
| *OPA189IDGK*  | 3 μV (max)   | 0.005                 | 100                  | 168  | 100M   | 380            | MUX-friendly buffer          |   |   | https://www.ti.com/lit/ds/symlink/opa189.pdf                                      |
| ADA4523-1     | 4 µV  @5V    | 0.01     @5V          | 88                   |      |        |                | LTC2500-32, LS current sense |   |   | https://www.analog.com/media/en/technical-documentation/data-sheets/ada4523-1.pdf |
| ADA4522-1ARMZ | 5 µV max     |                       |                      |      | 150 pA | 4              |                              |   |   |                                                                                   |
| OPA388IDG     | 5µV (max)    | 0.005                 | 140                  |      | 100M   | 100            |                              |   |   |                                                                                   |
| LT6202        | 500µV Max    |                       |                      |      | 4M     | 0.1            | A/D drv                      |   |   |                                                                                   |
| *AD8065ART*   | 1.5 mV (max) |                       | 7                    |      |        | 0.01           | A/D drv in HPM7177           |   |   |                                                                                   |
| OPA2172       |              |                       |                      |      | 100M   | 30             |                              |   |   |                                                                                   |
| LTC2057       |              |                       |                      |      |        |                |                              |   |   |                                                                                   |
| LT1001        | 15µV         | 0.6                   | 300                  |      |        |                |                              |   |   |                                                                                   |

![img_13.png](img/img_13.png)

the HPM7177 reference:
![img_14.png](img/img_14.png)

* ADA4522-1ARMZ
* AD8065ART

* input buffer OPA189IDGK
* full-diff stage
    * OPA388IDG
    * AD8065ART
* ADC

https://edms.cern.ch/ui/file/2820336/1/EDA-04061-V4-1_sch.pdf

- OPA189IDGK and AD8065ART might work well for our DC-application
- need to figure voltage reference and suitable adc
- maybe ads127

"High quality capacitors and resistors should be used in the
RC filters since these components can add distortion. NPO
and silver mica type dielectric capacitors have excellent
linearity. Carbon surface mount resistors can generate
distortion from self-heating and from damage that may
occur during soldering. Metal film surface mount resistors
are much less susceptible to both problems."

![img_15.png](img/img_15.png)