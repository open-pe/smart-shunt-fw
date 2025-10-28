https://youtu.be/KrYkQ5vnV9s?si=kV43mM-iGArl_4rd&t=593

https://www.digikey.de/en/products/detail/danisense-a-s/DC200IF/10406105

DS50ID

|                             | RMS   | ratio  | noise         | tot accuracy | lin    | offset | offsetTC | OSstab       | BW   | burden    | supply |                                                                                |
|-----------------------------|-------|--------|---------------|--------------|--------|--------|----------|--------------|------|-----------|--------|--------------------------------------------------------------------------------|
| Danisense DS50ID            | 50A   | 500:1  |               | 61ppm        | 1ppm   | 60ppm  | 0.4ppm/K | 0.4ppm/month | 1Mhz |           |        | [pdf](https://danisense.com/wp-content/uploads/DS50ID-2.pdf)                   |
| DT100ID                     |       |        |               |              | 1ppm   | 50ppm  | 0.3      | 0.1          |      |           |        |                                                                                |
| DS200ID                     | 200A  | 500:1  |               | 16ppm        | 1pm    | 15ppm  | 0.1      | 0.1          |      |           |        | https://danisense.com/wp-content/uploads/DS200ID-3.pdf                         |
| DS300ID                     | 300A  | 1000:1 |               | 15ppm        | 1ppm   | 14ppm  | 0.1      | 0.1ppm/month |      |           |        |                                                                                |
| DC200IF                     |       |        |               |              | 6ppm   | 15ppm  | 2ppm/K   |              |      |           |        |                                                                                |
| LEM IN 100-S                | 100A? | 500:1  |               |              | 1.2ppm | 15ppm  | 0.4ppm/K | 0.1ppm/month | 1Mhz |           | +-15V  | https://www.farnell.com/datasheets/3630373.pdf 830€                            |
| LEM IT 200-S ULTRASTAB      |       |        |               |              |        |        |          |              |      |           |        |                                                                                |
| IT 205-S ULTRASTAB          | 200A  |        |               |              | 3ppm   |        | NID      | 1ppm/month   |      |           |        |                                                                                |
| CT-100-P                    | 71A   | 1000:1 | 0.5ppm(typ)   |              | 3ppm   | ???    | 0.5ppm   |              |      |           |        | 475 €                                                                          |
| LEM IT 60-S ULTRASTAB       | 60A   |        |               |              |        |        | 2.5ppm/K | 2.ppm/month  |      |           |        |                                                                                |
| LEM ULTRASTAB ITN-600-S/SP2 | 600   | 1500:1 |               |              | 1.5ppm | 15ppm  | 0.5ppm/K |              |      |           |        |                                                                                |
| LEM HTA 600-S               | 600A  |        |               | 1%           |        |        |          |              |      |           |        | https://www.lem.com/sites/default/files/products_datasheets/hta_100-1000-s.pdf |
| ULTRASTAB 867-600           | 600A  |        |               |              | 3ppm   |        |          |              |      |           |        |                                                                                |
| ULTRASTAB 866-600           | 600A  | 1500:1 | 0.05ppm(10Hz) |              | 1ppm   | 20ppm  | 0.2ppm/k |              |      | 2.5..100Ω |        |   https://archive.org/details/manualzilla-id-5685713                                                                             |

# Resistors

Model SR1
https://testequipment.center/Product_Documents/Tegam-SR1-100-Specifications-F898F.pdf

|                   | Ω   | init tol | LT tol | cal tol | term | TC  | P   | Imax  |
|-------------------|-----|----------|--------|---------|------|-----|-----|-------|
| Y0078100R000B9L   | 100 | 1000ppm  |        |         | 2    | 0   | .3W |       |
| Y472650R0000V0L   | 50  | 50ppm    |        |         | 4    | 0.2 | 1W  |       |
| ESI SR1-100 ($80) | 100 | 20ppm    | 50     | 10      | 2    | 5   | 1W  | 100mA |

# Design Example: 50A

- DS50ID
- Y472650R0000V0L, VPG Foil Resistors
  RES 50 OHM 1W 0.005% (50ppm) AXIAL, ±0.2ppm/°C, 4 terminations

# Danisense

https://www.eevblog.com/forum/metrology/dccts-(ultrastab-for-example)-infos-and-workingdefective-devices-sought/
![img_8.png](img/img_8.png)


# DCCT Example:

Choose ULTRASTAB 866-600 because it is cheap on ebay. The current range of interest is 0-40A.
To cover the range of the DCCT, we use 10 windings, so at 40A the DCCT will measure 400A and outputs 267 mA.

![img_9.png](img/img_9.png)

We want to capture the signal with an ADC, so keep in mind the range. We choose ADS1262-32bit-38kSPS, because
there's a breakout board and it has an internal PGA and a secondary 24bit aux ADC.
Its INL (all gains and PGA bypass) is 12ppm (1.2m%) max and 3ppm typ).

the ADS131M02 has 2 differential inputs (simultaneous sampling).
the recommended abs min AIN voltage is -1.3 V below GND. So 