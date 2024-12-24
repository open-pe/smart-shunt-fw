![img_2.png](img_2.png)

This is a energy monitor based on the INA228.
It comes with a Würth PowerPlus M5 terminals and a 4026 shunt resistor slot for resistors from e.g.
Bourns, Vishay, Isabellenhütte.
Alternately you can use chassis mount resistors such as the Riedon RSN series with very low temperature drift.

The board comes with an isolated dc-dc and i2c isolator (and another isolator for the alert signal).

## Measuring voltage >85V

Although the INA228 only supports Vbus up to 85V you can measure much higher voltages (I tested up to 400V DC).
Put the shunt on the ground sound and populate the voltage divider with THT precision resistors or SMD chip resistors.

For high voltages higher than 85V the `IN-` pin must be pulled to ground. There will be a lot of noise otherwise.

# isolators

![img.png](img.png)

TPT774x:

![img_1.png](img_1.png)

TPT7721-SO1R:
2 ch

FXMAR2102UMX:
2ch bidirectional! $.32

INA241

* motor drives
  https://www.ti.com/de-de/video/6314459715112

![img_3.png](img_3.png)

INA296A

* Version A: ±0.01%, ±1ppm/°C drift
* app: Test & Meausrement
* https://github.com/CrabLabsLLC/OpenMD/blob/develop/docs/Schematic.pdf
*

![img_4.png](img_4.png)

![img_5.png](img_5.png)

![img_6.png](img_6.png)

TPA6271:
110V 16bit power monitor

|                | Vbus   | CMRR(min) | BW      | OffsetErr | Gain Err(max) | NonLinErr | TempDrift | BiDir? | ShuntVRange | Outp       |                                                                  |
|----------------|--------|-----------|---------|-----------|---------------|-----------|-----------|--------|-------------|------------|------------------------------------------------------------------|
| 3peak TPA6271  | 102.4V | 120 dB    | 140µs   | ±10µV     | 0.15%         |           | 10ppm/C   | y      | 81.92mV     | i2c, 16bit | [pdf](https://static.3peak.com/res/doc/ds/Datasheet_TPA6271.pdf) |
| TI INA226      |        |           |         |           |               |           |           |        |             |            |                                                                  |
| TI INA228      |        | 154 dB    |         |           | ±0.05%        | ±0.002%   | ±20 ppm/C | y      |             |            |                                                                  |
| TI INA229      |        |           |         |           |               |           |           |        |             |            |                                                                  |
| TI INA241      |        |           |         |           |               |           |           |        |             |            | enhanced PWM-suppr.                                              |
| TI **INA296A** | 110V   | 150 dB    | 1.1MHz  | ±10µV     | ±0.01%        | ±0.001%   | ±1ppm/°C  | y      |             |            |                                                                  |
| TI INA310A     | 110V   | 140 dB    | 1.3 MHz | ±20 µV    | ±0.15%        | ±0.01%    | 10ppm/C   |        |             |            |                                                                  |
| LMV93x-N       | n/a    | 60 dB     | 1.4 MHz | 4 mV      |               |           |           |        |             |            |                                                                  |
| THS4551        |        |           | 150 MHz | ±175 µV   |               |           |           |        |             |            |                                                                  |

We can calibrate Gain Err/ Gain Offset with 2 point calibration.
Non-linearity can be reduced by multi-point calibration (LUT).
TempDrift can be compensated with a temperature sensor and the drift curve from the datasheet.

Non linearity and temp drift are crucial since harder to compensate.

ADC

| MPN        | eff resolution | gain   | sps | ch | tempDrift | NonLinear | offset  | OffsetDrift | GainErr | GainDrift |
|------------|----------------|--------|-----|----|-----------|-----------|---------|-------------|---------|-----------|
| ADS1220    | 20bit          | 1..128 | 2k  | 4  | 5 ppm/C   |           |         |             |         |           |
| ADS127L21B |                |        |     |    |           | 0.5ppm    | +-250µV | 200 nV/C    |         | 1ppm/C    |
| ADS127L11  |                |        |     |    |           |           |         |             |         |           |
| ADS124S06  |                |        |     |    |           |           |         |             |         |           |

## Analog Version

- INA296A + ADS1220 (or ADS127L21B)
- SPI need 3 up, and 2 down ch. (or only 1 down?)
- analog level translation for oscilloscope connection. using high-side current sense circuit?
  ![img_7.png](img_7.png) ( LMV93x-N datasheet)
    - unfortunately the INA296 does not give access to (-) inp of


- THS4551 (amp) +

https://www.ti.com/solution/power-analyzer?variantid=34826&subsystemid=23445


PwrTool 500