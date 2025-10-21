<img src="hw/pcb-3d-render.webp" width="500">

A versatile isolated power monitor based on INA228.
I made this for precision power metering and testing DCDC converters.

* Digital Signal isolators for i2c and alert
* Isolated power supply
* battery connector
* optional Vbus voltage divider for dc bus voltages above 85V
* programmable i2c address
* NTC for shunt temperature measurement (in addition to the INA228 internal temperature sensor)

# Building

## Platformio Setup

* [install script](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html)
* [setup shell commands](https://docs.platformio.org/en/latest/core/installation/shell-commands.html)
* [clion](https://docs.platformio.org/en/latest/integration/ide/clion.html)

select the board:

`pio run -e esp32`
or
`pio run -e esp32s3`

# TODO

- add ssa-100 plug?
- add ntc
- add pin header for chassis mount resistor
- jumper to disconnect on-board shunt?
- add RSN-50-50 footprint (no available)
- add external alert (for OC shutdown)
- add external IO pins?

Supported ADC:

* ADS1115 / ADS 1015 (with SSA-100)
* INA226
* INA228

On startup it detects the ADCs on the I2C bus using there default addresses.

I tested it on these MCUs:

* ESP8266
* ESP32
* ESP32-S3

The program reads voltage and current and computes power.
It integrates power for energy measurement using the trapezoidal rule.

# Output

* Serial (human friendly logs)
* UDP messages using InfluxDB line protocol

# Voltage Divider Resistors

Resistors with low temperature drift are favourable. Tolerance is not so important because we can simply calibrate.
Consider the ADC input impedance when designing the voltage divider.
Thin film or metal film resistors. Cheap ($.5) SMD chip resistors with 10ppm/°C.
Resistors with 5ppm/°C drift cost $2.
Choose resistors with equal drift, so drift get eliminated.

# Current Sense Resistors

https://www.researchgate.net/profile/Gunnar-Fernqvist/publication/224304168_Characteristics_of_burden_resistors_for_high-precision_DC_current_transducers/links/65ad0815f323f74ff1e064ab/Characteristics-of-burden-resistors-for-high-precision-DC-current-transducers.pdf
https://accelconf.web.cern.ch/p07/papers/MOPAN071.pdf
"The highest precision is offered by a proprietary Zeranin wire design from Hitec,"
=> Bulk Metal Foil or “foil”.

https://accelconf.web.cern.ch/p77/PDF/PAC1977_1810.PDF
![img_37.png](img_37.png)
![img_35.png](img_35.png)
![img_36.png](img_36.png)

10A <1ppm reference https://cds.cern.ch/record/643293/files/cer-002399330.pdf

https://www.datatec.eu/caenels-ct-100-c
https://www.datatec.eu/stromwandler-dcct
https://www.datatec.eu/caenels-ct-100v-p

A CT100-P with Y007810R0000B9L (10ohm, 0ppm/K) or 4-terminal Y169025R0000T9L (25ohm, 0.2ppm/K)
with 25ohms: 25mV/A (equiv burden 25mΩ, and almost no self-heat)

Low drift is important. Drift can be compensated with temperature measurement.
Consider max current and power dissipation. For resistances <=1mΩ it is hard to find low drift SMD chip resistors.
A chassis mount resistor might be a better choice for current measurement >40 A.

Riedon https://riedon.com/resistors/current-sense/
VPG

RUG-Z isabellenhuette

# Current Reference

https://doc.xdevs.com/doc/_Metrology/wang2014.pdf

# Resistor References

Tinsley 1682 https://www.ebay.com.au/itm/385633870779
J.L. William Scientific Instruments 0.1 Ohm Cal Standard DC Resistor (SN: 3127) (10 amp)
3928 (14amp)

## SMD Resistors

| MFR            | MPN                     | mΩ  | P   | Imax | tol | TCR           | padW/mm | padH/mm | 4wire | px100 |                                                                                       |
|----------------|-------------------------|-----|-----|------|-----|---------------|---------|---------|-------|-------|---------------------------------------------------------------------------------------|
| isabellenhütte | BVR-Z-R0003-1.0         | .3  | 11W |      | 1%  | 20ppm         | 5.5     | 1.6     | y     | $1.9  |                                                                                       |
| bourns         | CSS2H-2512R-L300F       | .3  | 6W  |      |     | 150ppm(50ppm) | 3.4     | 1.8     | n     | $     |                                                                                       |
| Milliohm (CN)  | HoLRS5930-0.3mR-1%      | .3  | 7W  |      | 1%  | 50ppm         | 7.6     | 4.2     | n     | $.24  |                                                                                       |
| vishay         | WSLP5931L3000FEA        | .3  | 10W |      | 1%  | 175ppm        | 7.75    | 5.2     | n     | $1.7  |                                                                                       |
| RESI (CN)      | SEWF3951DL300P9         | .3  | 15W |      | .5% | 25ppm         | 13      | 2       | n     | $2.2  |                                                                                       |
| isabellenhütte | BVR-Z-R0005-1.0         | .5  |     |      | 1%  | 20ppm         |         |         |       |       |                                                                                       |
|                | BVR-M-R0007-1.0         | .7  | 8W  |      | 1%  | 20ppm         |         |         |       |       |                                                                                       |
|                | BVR-M-R001              | 1   | 7W  |      |     | 50ppm         |         |         |       |       |                                                                                       |
| vishay         | Y14880R00100D0R         | 1   | 3W  |      | .5% | 15ppm         |         |         | y     |       |                                                                                       |
|                | WSK12161L000FEA/B       | 1   | 3W  |      | .1% | 20ppm         |         |         |       |       |                                                                                       |
|                | Y14880R00200B9R         | 2   | 3W  |      | .1% | 15ppm         |         |         | y     |       |                                                                                       |
|                | Y14730R00300B0R         | 3   | 3W  | 31A  | .1% | 5ppm          |         |         |       |       |                                                                                       |
|                | Y14750R10000B9R         | 100 |     |      | .1% | 10ppm         |         |         |       |       |                                                                                       |
| vpg            | PCS301                  |     |     | 15A  |     | 15ppm         |         |         |       |       | [pdf](https://vpgfoilresistors.com/products/current-sense-resistors/pcs301/datasheet) |
|                | SHR 4-2321 0R005 S 1% M | 5   | 40W |      |     | 5ppm          |         |         |       |       |                                                                                       |
|                | SHR 4-2321 0R005 S 1% L | 5   |     |      |     | 2ppm          |         |         |       |       |                                                                                       |
|                | PCS 301 0R100 S 1%      | 100 | 30W |      | 1%  | 3ppm          |         |         |       |       |                                                                                       |

[Digikey Chip Mount Resistors 40ppm 1mΩ ](https://www.digikey.de/short/8dbrqrp4)

TODO TCR curve
https://docs.rs-online.com/2d4e/0900766b81142cbf.pdf

Choose Chassis Mount resistor, suchs as the 1mOhm RSN-100-100B with 15ppm/k

## THT Resistors

![img_7.png](img_7.png)

Riedon PCS (10ppm, 40W)
PCS-0R001D1

| MPN                 | mΩ | tol | P   | Imax | TCR   | px  |
|---------------------|----|-----|-----|------|-------|-----|
| VPG Y07340R05000G9L | 50 | 2%  | 10W | 14A  | 2ppm  | $82 |
| Riedon PCS-0R001D1  | 1  |     | 40W | 200A | 10ppm | $40 |
| Riedon PCS-0R002D1  | 2  | .5% | 40W | 141A | 10ppm | $40 |
| Riedon PCS-0R005D1  | 5  | .5% | 40W | 89A  | 10ppm | $40 |
| PCS-0R01D1          | 10 |     | 40W | 63A  | 10ppm |     |
|                     |    |     |     |      |       |     |

Riedon USR 4- 3425 (3ppm, 30W, 0.01%)

## Chassis Mount Resistors

These commonly have lower tolerance and lower temp drift as compared to chip mount.
The larger surface of the resistive material and larger terminals decreases thermal resistance to ambient, which
reduces temperature rise.

| ![img_3.webp](riedon-rsn-temp.webp) |
|:-----------------------------------:|
|  Manganin (Riedon RSN, RSW and RS)  |

![img.webp](riedon-rsw.webp)

![img_1.webp](riedon-rs.webp)

![img_2.webp](riedon-rsn.webp)

|                     | mΩ | P    | Tol   | TCR/K  | px    |
|---------------------|----|------|-------|--------|-------|
| Riedon RSW-50-50    | 1  |      | 0.25% | 15ppm  |       |
| Alpha FNPZR0040F    | 4  | 500w | 1%    | 1ppm   | $1060 |
| Alpha FNPZR0010B    | 1  | 500W | 0.1%  | 1ppm   | $1040 |
| FNPYR0020A          | 2  |      | 0.05% | 2.5ppm | $540  |
| Riedon RSN-100-100B | 1  | 10W  | 0.1%  | 15ppm  | $60   |
| RSA-50-100          | 1  | 5W   | 0.25% | 15ppm  | $30   |

## for battery monitoring

* SMD chip resistors are way cheaper (such as HoLRS5930-7W-0.25mR-1% )

|                                | mΩ | I(A) | tol |   |   | px |
|--------------------------------|----|------|-----|---|---|----|
| Milliohm HoFL2-75A-75mV-0.5%   | 1  | 75   | .5% |   |   | $5 |
| RESI ARCS8518JL100A9           | .1 |      | .5% |   |   |    |
| Milliohm HoFL3-8518-A-100uΩ-1% | .1 |      |     |   |   |    |
| HoFL2-200A-60mV-0.5%-HS        | .3 | 200  |     |   |   |    |

# 30A

ina228 has a shunt voltage range of 40.96 and 163.84 mV.
at a voltage drop of 40mV and 30A, we have 1.2 W of heat dissipation.

# applications

* bms/battery current sennsor
* smart shunt / lab shunt
* solar crowbar
* efuse
* mppt sensor

# set conversion time and averaging

* consder aliasing
* the ina22x ADC samples AND we sample the ina22x
* use conversion time to avoid aliasing during ADC
* use averaging to avoid aliasing during sampling the ina22x (i2c)
* when working with 50hz inverters, need a conversion time <=1ms.

# precision

* note that resistors change there resistance with temperature (see datasheet)
* use precision resistors
* keep temperature stable
* when measurement DCDC converter efficiency use 2x identical hardware. same chips and same resistors
* INA228 temp coefficient is 20ppm/°C, so choose resistor in that class
    * Ina228 features a temperature sensor

# AC Metrology

* Ti Design TIDU455A (MSP430AFE253, 24 Bit DeltaSigma ADC), class 0.2% accuracy
* Ti Eval Board EVM430-I2040S (MSP430I2040, 24bit, 8Khz or 80th harmonic of 50 Hz), <0.2% accuracy
    * AC, DC, 380V, 15 Amps
* Gossen Metrawatt Metrahit Energy (Starline Series) and
  Open-Source [IrDA USB](http://lemmini.de/IrDA%20USB/IrDA%20USB.html) (the Gossen USB X-Tra needs proprietary drivers,
  for windows only)
    * ESP32 has `UART_MODE_IRDA`, you only need the TFDU4101 [forum1](https://esp32.com/viewtopic.php?t=2766)

https://www.ti.com/lit/ug/tidua01/tidua01.pdf
https://www.analog.com/media/en/technical-documentation/data-sheets/ad637.pdf

* synchrounus subsampling (needs fast ADC)


* log-antilog

Thermal RMS Converters for ultimate
![img_42.png](img_42.png)

# EVM430

- the software "calibrator-20121120.exe" referenced in the datasheet is out-dated. I could not find a downlo
- use [MSP-EM-DESIGN-CENTER](https://www.ti.com/tool/MSP-EM-DESIGN-CENTER)
  , [intro](https://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/EnergyMeasurementDesignCenter/1_40_00_03/release/EnergyMeasurementDesignCenter_1_40_00_03/docs/users_guide/html/Energy_Measurement_Technology_Guide_html/markdown/ch_designcenter.html)
  . works on Windows, Mac, Linux
- voltage sense (C14)  pin 1 and 2, A0.0+ and A0.0-
- Current Sense (C10) pin 3 and 4, A1.0+ and A1.0-
- there is a project "EVM430-i2040S_SH_1V_1C_50Hz"
- Double click the MCU and "Generate Code"
- Open generated src code project with CCSTUDIO
    - Could not get the online version to open the project (https://dev.ti.com/ide/)
    - the download IDE based on eclipse succeeded to build the generated project, but need MSP-FET programmer. not sure
      if an ordinary USB2UART addapter works?

# Isolated Current Sensor

DC-DC: MIE1W0505BGLVH-3R-Z (monolithic)
i2c: TPT72616-SO1R (3peak)

# Calibration

```

ina22x-resistor-range 0.000333333333333333 120
calibrate ESP32_INA228 U 1.00
calibrate ESP32_INA228 I 1.00

ina22x-resistor-range 0.002 20
ina22x-resistor-range2 0.002 20  
ina22x-resistor-range3 0.002 20  

ina22x-resistor-range 0.001 40 
ina22x-resistor-range2 0.002 20 
ina22x-resistor-range3 0.002 20 


calibrate ESP32_INA228 U *1.000041
calibrate ESP32_INA228_2 U *1.000041
calibrate ESP32_INA228_3 U *1.0000233720530345

calibrate ESP32_INA228 I *1.0001119027192362
calibrate ESP32_INA228_2 I *0.9999067668612132

calibrate ESP32_INA228_3 I *1.0056727470362443



calibrate ESP32_INA228_2 U *1.00005
calibrate ESP32_INA228_3 I 1.0

calibrate ESP32_ADS1220 U *0.9985899493319389

calibrate ESP32_INA228 U *1.000032503295473
calibrate ESP32_INA228_2 U *1.000032503295473


calibrate ESP32_ADS1262 U 1.011406869687194   # an1->gnd
calibrate ESP32_ADS1262 U 1.0   # an1->gnd


calibrate ESP32_ADS1262 U 0.99882990

calibrate ESP32_ADS1262 U 1.01045



```

# Reference Designs

### LP-XDS110ET

[Design Files](https://www.ti.com/tool/LP-XDS110ET#design-files)

* ADC: ADS127L01IPBS
* Ref: REF6025IDGK (drift: 5ppm/K, noise: 5 µVRMS)
* Clock: SN74LVC2G74DCU
* Diff. Amp: THS4551IRUN
* Current Sense Amp: INA118UB

### ADS131B23

![img_14.png](img_14.png)

# Going Further

| error type      | mitigation                    | effort |
|-----------------|-------------------------------|--------|
| gain error      | single point calibration      | low    |
| gain temp drift | temperature measurement       | medium |
| non-linearity   | multi-point calibration (LUT) | medium |
| noise           | averaging                     | low    |

Simulatnous 2ch sampling instead of multiplexing.
Alternative: use 2 ADC chips

* ADS127L01 + THS4551 + INA169? ( see LP-XDS110ET circuit )

ADS131B23:

* 2x 24bit for redundant current measurement
* 1x 16bit for voltage measurement

# Selection Criteria

* Simultaneous sampling
    * more precise AC power sampling
    * easier to program
* PGA gain of >= 64
    * enable FSR of 40mV to directly measure voltage across current sense resistor without additional
      amplifier
* Linearity
    * Linearity can be improved with a look-up-table. Creating these tables require multi-point calibration.
    * Compare linearity error from the ADC data sheet to the temp drift of current sense resistor

# Voltage References

| MPN          | drift/K    | noise    | initial accuracy |                                                      |
|--------------|------------|----------|------------------|------------------------------------------------------|
| REF6025IDGK  | 5ppm       | 5 µVRMS  | ±0.05%           |                                                      |
| REF6225IDGKT | 3ppm       | 5 µVRMS  | ±0.05%           | [pdf](https://www.ti.com/lit/ds/symlink/ref6225.pdf) |
| LTC6655      | 2ppm       |          | ±0.025%          | for use with  LTC2508-32                             |
| LTC6652      | 5ppm       |          | 0.05%            |                                                      |
| LTZ1000      | 0.05ppm/°C | 1.2µVP-P |                  | for calibrators                                      |

![img_23.png](img_23.png)
![img_24.png](img_24.png)
https://youtu.be/upTgM_S5rAQ?si=JQq38q3_NsaYmSlv&t=2213

Fluke732A

https://github.com/marcoreps/ADRmu

Reference of all References
https://hackaday.com/2015/12/03/nuts-about-volts/
https://www.ptb.de/cms/service-seiten/news/forschungsnachricht.html?tx_news_pi1%5Bnews%5D=12912&tx_news_pi1%5Bcontroller%5D=News&tx_news_pi1%5Baction%5D=detail&tx_news_pi1%5Bday%5D=11&tx_news_pi1%5Bmonth%5D=10&tx_news_pi1%5Byear%5D=2023&cHash=8994d84faf7ba52fd97fa9559617df21

![img_33.png](img_33.png)
![img_34.png](img_34.png)

# Voltage Divider

VHD200 (digikey 15 weeks )
100mW. for measuring 80V at 2V ADC min 64k r_total
![img_25.png](img_25.png)

# Amps

THS4521 (ref6225)

# ADC Break-outs

https://protocentral.com/product/ads1115-16-bit-adc-4-channel-with-programmable-gain-amplifier/

https://protocentral.com/product/ads1220-24-bit-4-channel-low-noise-adc-breakout-board/

https://protocentral.com/product/protocentral-ads1262-32-bit-precision-adc-breakout-board/

https://www.mikroe.com/adc-15-click

https://www.waveshare.com/18983.htm (ADS1263)

MIKROE-4743 (ADS1262) ![img_15.png](img_15.png)

# Misc

# Resolution and Noise

from https://www.ti.com/lit/ds/symlink/ads1231.pdf :
The RMS and Peak-to-Peak noise are referred to the input. The effective number of bits (ENOB) is defined as:
ENOB = ln (FSR/RMS noise)/ln(2)
Noise-Free Bits = ln (FSR/Peak-to-Peak Noise)/ln(2)
FSR (Full-Scale Range) = VREF/Gain.

https://electronics.stackexchange.com/questions/274606/whats-the-highest-precision-achieved-for-an-adc

Let's consider the application of measuring conversion efficency of a DC/DC converter.

According to the data sheet, INA228 measures power with total meusurement
error of 0.5% at full scale. We need two power monitors, one at the input and one at the output, total worst case error
of two devices is around 1%.

This error is too high, considering the DC/DC has an efficiency of > 95 %.
So let's have a look at more precise alternatives.

First

|                            |           |   |
|----------------------------|-----------|---|
| Gain Err (max)             | ±0.05 %   |   |
| Gain Temp Drift (max)      | 20 ppm/°C |   |
| ADC Resolution             | 20 bit    |   |
| Integral Non-Linearity     | 20 ppm    |   |
| Differential Non-Linearity | 0.2 LSB   |   |

For noise analysis, we need to define ADC measurement range and AC component frequency.
What is the ripple current, voltage and frequency?
Is the ripple periodic?
Is the ripple neglictable?
Can we smooth is out with a LP filter so we can assume DC only?
What digital filter does the ADC have?
Do we need an analog filter? Do we need to filter Aliasing?

TODO

* zero cross detection?
* AC metering?
* simulatnous filtering AC?

To work with some numbers, we define a working point:

- max current 40A
- current sense resistor 1mΩ
- max heat: 1.6 W (i2r)
- max voltage 40mV


* define a sampling rate
    * do we need to capture AC component or can we filter them?
    * if reactive power matters, we need simultanous ( https://www.ti.com/lit/an/slaa638a/slaa638a.pdf )

"The sample frequency is [...] 8 kHz that give us Nyquist
bandwidth of 4 kHz. This bandwidth is sufficient to cover the 66th harmonic for 60Hz AC, 80th harmonic for
50Hz AC frequency as wider bandwidth is usually needed for server power monitoring due to the nature of the
switching power supplies being monitored." -https://www.ti.com/lit/an/slaa638a/slaa638a.pdf

* :
* (max):
*

Note 7: Integral nonlinearity is defined as the deviation of a code from a
straight line passing through the actual endpoints of the transfer curve.
The deviation is measured from the center of the quantization band.

- https://www.ftcelectronics.sg/datasheets-58/LTC2508CDKD-32-TRPBF.pdf

# SPI Isolators

https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6820.pdf
(needs 2 chips and 2 transformers)

https://www.mikroe.com/spi-isolator-5-click
using DCL541A01 (150Mbps, 3f/1r channels, default Low out)

https://www.electronics-lab.com/project/4-channel-spi-interface-isolator-with-three-forward-and-one-reverse-direction-channels/
based on ISO77418 (100Mbps, 5000V iso, 3f/1r ch, default High out)
![img_6.png](img_6.png)

# Tools

https://www.vishay.com/en/resistors/change-resistance-due-to-rtc-calculator/

# Heaters

https://www.amazon.de/Fyearfly-Temperatur-Heizplatte-Silikonkautschuk-elektrische/dp/B09MJ96BHK?_encoding=UTF8&pd_rd_i=B09MJ96BHK&pd_rd_w=NS24i&content-id=amzn1.sym.aab634e4-29b8-4848-9d24-fad213311357&pf_rd_p=aab634e4-29b8-4848-9d24-fad213311357&pf_rd_r=DECKKMJ91CMZ0HVX76A9&pd_rd_wg=tmowz&pd_rd_r=e27434a8-a59c-40f0-836b-ad5017ef97aa
https://www.digikey.de/en/products/detail/dfrobot/FIT0845/15848056
https://www.amazon.de/sourcing-map-selbstklebende-Heizelemente-Heizstreifen/dp/B0CTT8DCNG?mcid=2e62e8e0dd8637f59241dbf166a54a3d&th=1&psc=1&hvocijid=11518062653062946358-B0CTT8DCNG-&hvexpln=75&tag=googshopde-21&linkCode=df0&hvadid=696184104678&hvpos=&hvnetw=g&hvrand=11518062653062946358&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9043874&hvtargid=pla-2281435176898&psc=1&gad_source=1
https://electronics.stackexchange.com/questions/631263/using-mosfet-as-a-voltage-controlled-current-source

![img_11.png](img_11.png)
https://www.digikey.de/en/products/detail/innovative-sensor-technology-usa-division/HST-P11R0-1010-63A-2L-020/21407273

![img_12.png](img_12.png)
https://www.digikey.de/en/products/detail/riedon-products-by-bourns/PTCA-40/10271325

# Peltier

https://www.farnell.com/datasheets/4020641.pdf

# Heating Circuit

We want a precise temperature control loop to stabilize temperature of the resistive element.
The heater should be somehow coupled to the resistor for good heat transfer and it also needs
a heat-sink. We decide against PWM, since efficiency is not an issue, as we want to generate
heat anyway and it might cause unnecessary switching noise.

A linear control loop appear to make most sense. There are circuits like

![img_13.png](img_13.png)

the NTC is a LC471F3K, nominal 470 X at 25 °C, which acts as a sensor and heating element.

Despite the simplicity of this circuit, it not suitable in our application.
Measuring the temperature at the resistive element is important, since the resistor itself will produce heat.
Also finding an NTC with suitable power will likely be difficult.

We will measure

https://www.wellesu.com/10.1016/j.compag.2016.06.011
https://dr.lib.iastate.edu/server/api/core/bitstreams/d64d1b18-f1b3-41cd-be34-bc21360b506e/content
https://electronics.stackexchange.com/questions/690961/i-dont-understand-this-temperature-controller-circuits-behaviour
https://electronics.stackexchange.com/questions/498065/anemometer-with-opamp

https://www.circuitlab.com/circuit/63742k8e26j4/ntc-heater-control/

# Increasing Precision

The INA228 data sheet mentions a power monitoring accuracy of 0.5%. For monitoring DC/DC converters
with efficiencies of 95% or higher, INA228 accuracy is not sufficient. The integrated 0.5% precision oscillator adds
even more error for energy measurement.

# ADS1262 Journal

* ordered from protocentral
* are these genuine chips?
* review passive component circuitry of break-out board
* ads1262 needs at least 4.75V analog supply voltage !
* prefer MIKROE-4743 from digikey
* with PGA need at least 0.3V margin to rails (see Datasheet)
* linearity issue
    * sth appears to be wrong
    * absolute error seems quite high
    * maybe there is something wrong with the code?
* ads1220 appears to perform quite well
    * todo need spi isolator

# Measuring Single-Ended Signals with Diff-ADC

https://www.ti.com/lit/an/sbaa133a/sbaa133a.pdf?ts=1738999464967
![img_16.png](img_16.png)

THS4551
![img_20.png](img_20.png)

![img_17.png](img_17.png)
![img_18.png](img_18.png)
![img_19.png](img_19.png)

![img_38.png](img_38.png)
LTC2057 + LTC2508-32

# Current sense ampplifiers

MCP6C02

# Instrumentation AMps

- focus on precision (low noise, low offset, linearity)
- differential gain
- high CMRR
- high input impedance, low output impedance

![img_21.png](img_21.png)
![img_22.png](img_22.png)

https://www.ti.com/lit/ds/symlink/ina118.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1739209355414&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Fina118#page=4&zoom=100,0,96
https://www.ti.com/solution/battery-cell-formation-test-equipment?variantid=34913&subsystemid=292016

|               |     | max gain    | noise                           | offset voltage (max) | offset drift (max) | gain drift ppm/K  G=1/G>1 | gain nonlinearity | common mode rej |                       |
|---------------|-----|-------------|---------------------------------|----------------------|--------------------|---------------------------|-------------------|-----------------|-----------------------|
| **INA190**    |     |             | 75 nV/√Hz                       | ±15 μV               | 0.08 uV/°C         | 7 ppm/°C                  | 100ppm            |                 |                       |
| INA848        |     | 2000 (fix)  | 1.5 nV/√Hz                      | 35µV                 | 0.45 μV/°C         | 5                         | 10ppm             | 132dB (min)     |                       |
| **INA851**    |     | 1000        | 3.2 nV/√Hz                      | 35 µV                | 0.3 µV/°C          | 5/35                      | 5ppm  (typ)       | 120db (min)     | with fully diff outp. |
| INA849        |     | 10000       | 1-nV/√Hz                        | 35 µV                | 0.4 μV/°C          | 5                         | 10ppm             |                 | for bat test          |
| INA826        |     |             | 18 nV/√Hz (G>=100)              | 150 µV               |                    | 1/35                      | 5ppm              |                 | bat test              |
| ina828        |     |             | 7 nV/√Hz                        | 50 µV                | 0.5 µV/°C          | 5/50                      | 15ppm             |                 |                       |
| INA2128       |     |             | 8 nV/√Hz at 1 kHz               | 50µV                 | 0.5 µV/°C          | ±10ppm                    |                   | 120db (min)     |                       |
| INA103        |     |             | 1nV/√Hz                         |                      |                    | ?                         | 100ppm            |                 |                       |
| **INA818**    |     |             | 8 nV/√Hz                        | 35µV                 |                    | 5/35                      | 15ppm (max)       |                 |                       |
| INA118        |     |             | 10nV/√Hz                        | 50µV                 | 0.5 µV/°C          |                           | 20ppm             |                 |                       |
| INA333        |     |             | 50 nV/√Hz                       | 25 μV                | 0.1 μV/°C          |                           | 10ppm             |                 |                       |
| AD624C        |     |             |                                 |                      |                    |                           |                   |                 |                       |
| LTC6915       | $9  | 4096 (prog) | 2.5µVP-P (0.01Hz to 10Hz)       | 10µV                 | 0.05µV/°C          |                           | 15ppm  (max)      |                 |                       |
| INA241A       | $3  |             | 28nV/Hz                         | 10µV                 |                    | 1ppm                      | 10ppm             |                 |                       |
| LTC1100       | $18 | 10 or 100   | ■ 0.1Hz to 10Hz Noise: 1.9µVP-P | 10uV (typ 1µV)       | 0.1µV/°C           | 4ppm                      | 8ppm (max)        |                 |                       |
| **LTC2053**   | $10 | 1000        | 2.5µVP-P (0.01Hz to 10Hz)       | 10µV                 | 0.05uV/°C          | 0                         | 3(typ) 12 (max)   | 116db (typ)     |                       |
| LT1167        |     |             | 0.1Hz to 10Hz Noise: 0.28μVP-P  | G = 10, 60μV Max     | 0.3μV/°C           | 50ppm       ?             | G = 10, 10ppm Max |                 |                       |
| LTC6363       |     |             | 0.2µVpp, DC to 10Hz (Typ)       |                      | 0.015μV/°C         |                           |                   | 150db (typ)     |                       |
| MCP6C02(G=20) |     |             | <=1Hz:0.48μVpp, 1.54            | G=20: 16µV           | 0.09 µV/°C         | 5ppm                      | 50ppm/100ppm      |                 | current sense amp     |

![img_30.png](img_30.png)
![img_31.png](img_31.png)
![img_32.png](img_32.png)

LNA10 (OPA4727) see https://www.youtube.com/watch?v=NoCp6cz32yo
Stanford Research SR560

https://www.thorlabs.com/thorproduct.cfm?partnumber=AMP100
https://github.com/marcoreps/low_noise_amplifier

![img_39.png](img_39.png)
LTC2057HV + LT1991A

# Op Amps

OPA4727
offset: 15µV (typ), 150µV (max)
drift: 0.3µV/°C (typ), 1.5µV/°C (max)
BANDWIDTH: 20MHz
LOW NOISE: 6nV/√Hz at 100kHz
AOL > 110dB

OPAx209
(higher drift, lower noise)
: 2.2 nV/√Hz at 1 kHz
Low Offset Voltage: 150 µV (Maximum)
drift: 3 µV/°C (max)

OPAx189
– Zero-drift: 0.005 μV/°C

- Ultra-low offset voltage: 3 μV maximum
  – CMRR: 168 dB
- en at 1 kHz: 5.2 nV/√Hz
- 0.1-Hz to 10-Hz noise: 0.1 µVPP

OPAx388

- offset voltage: ±0.25 µV
- drift ±0.005 µV/°C
- 140-dB CMRR
- Low noise: 7.0 nV√Hz at 1 kHz
- No 1/f noise: 0.14 uVPP (0.1 Hz to 10 Hz)
- https://www.ti.com/solution/analytical-lab-instrumentation#block-diagram
-

# Youtube

https://www.youtube.com/watch?v=upTgM_S5rAQ
https://www.youtube.com/watch?v=NoCp6cz32yo

# Literature

https://docs.ampnuts.ru/eevblog.docs/HP_Agilent_Keysight/HP%203458A%20Component-Level%20Information%20Packet.pdf
https://vtda.org/pubs/HP_Journal/HP_Journal_1989-04.pdf
https://doc.xdevs.com/doc/HP_Agilent_Keysight/3458A/ac_algo/Swerleins_Algorithm.pdf

* highest accuracy for lowest freq

# DataLogging

https://github.com/marcoreps/multiinstrumentalist

* HP3458A
* HP34420A
* W4950
* F8508A
* K2182A
* D1281

# Temp Sensors

TMP117 (i2c)

# ADC

important for dc metrology: stability and linearity (DC to 103- 104 Hz )
https://www.researchgate.net/publication/325285614_Analog-to-digital_conversion_beyond_20_bits_Applications_architectures_state_of_the_art_limitations_and_future_prospects

https://www.ti.com/lit/ug/tiduco9b/tiduco9b.pdf?ts=1739063348030&ref_url=https%253A%252F%252Fwww.google.com%252F

https://indico.cern.ch/event/814368/contributions/3397918/attachments/1832255/3000935/Class_0_ADC_-_Beev.pdf

https://www.welectron.com/Digilent-MCC-172-IEPE-Sensor-DAQ-HAT-for-Raspberry-Pi-2-CH-24-bit

# Thermal EMF

https://www.ab-precision.de/products/cables-accessories/

https://www.ni.com/docs/de-DE/bundle/ni-switch/page/thermal-emf-and-offset-voltage.html

https://www.youtube.com/watch?v=EW58sm8ciWk
![img_26.png](img_26.png)
![img_27.png](img_27.png)
https://www.youtube.com/watch?v=KiYhEP6m7Pc

![img_29.png](img_29.png)
https://dam-assets.fluke.com/s3fs-public/p18-21.pdf

https://www.eevblog.com/forum/metrology/binding-postsbanana-sockets-cleaning-on-dmmsmetrology-lab-gear/

https://www.eevblog.com/forum/chat/help-wanted-low-and-high-ohm-measuremet/?action=dlattach;attach=35289

Pomona 2305

diy Fluke 5440A-7003
https://www.eevblog.com/forum/metrology/diy-low-emf-cable-and-connectors/msg545213/#msg545213

Riedon RSN Brass Terminals:
Thermal EMF to copper wire ~2µV, at 50mV FSR this is 40ppm.

# Gold Plated Banana Plugs

CT3099
![img_28.png](img_28.png)

[Pomona 4897-2](https://www.digikey.de/en/products/detail/pomona-electronics/4897-2/737024)

https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/4846/DS_1768_Catalog.pdf

Multicomp 1699021

Farnell has more than digikey

# Kelvin Clips Gold Plated

CTM-78K
CTM-75K

# Linearity, INL

https://xdevs.com/article/inlperf/

![img_40.png](img_40.png)

# CERN HPM7177 (open soruce) DMM

https://ohwr.org/project/opt-adc-10k-32b-1cha/-/wikis/home
base on AD7177-2
effective resolution >23 bits for BW < 10 Hz
temperature drift (offset and gain) <0.1 ppm/°C from 20°C to 40°C
INL <0.6 ppm (0 to +10 V)

![img_41.png](img_41.png)

# Notes on Precision

High-Precision (ppm range) current sensing can be tricky due to thermal effects.
Most resistors have a positive temperature coefficient when they heat up, so current is measured too high.
A closed-loop temperature control of the current sense resistor can improve stability.
There are Vishay VPG resistors with 0ppm/K or <1ppm/K, but only in the sub-1W range, so not suitable for higher
currents.
Low burden voltage decreases resistor heating but Thermoelectric noise (thermal EMF) becomes more dominant.
Use as few material boundaries, not connectors, no gold, just copper. Use un-finished copper trace to contact resistors
with screw terminals, which are usually some sort of brass.

Offset drifts can be compensated with a chopper circuit, that frequently bypasses the load current and measures the
offset current
which is then compensated in following measurement.

If you want to measure ratio of currents, use the same sense resistors and chop both currents over it to eliminate
drift.
This way any error produced by the resistor is eliminated, offset by thermal EMF is still relevant and the ADC
performance
(offset and linearity).

For best linearity, highest thermal imunity and lowest drift, consider using DC-current transformers (DCCT).