1. Ovenized Current Sense Resistor


2. Amplifier stage
   use TI Instrumentation amp with differential output for simplicity

INA851

* Low offset voltage: 10 µV (typ), 35 µV (max)
* Low offset drift: 0.1 µV/°C (typ), 0.3 µV/°C (ma
* Input stage noise: 3.2 nV/√Hz, 0.8 pA/√Hz
* Input overvoltage protection to ±40 V beyond supplies
* gain drift: ±35 ppm/°C (max, G_IN > 1)
* INL 5ppm
* pssr: 126 (GIN = 100 ) (power supply rejection)
* Supply range:
  – Single supply: 8 V to 36 V
  – Dual supply: ±4 V to ±18 V
* optimized to drive inputs of modern high performance analog-to-digital converters (ADCs) with fully differential
  inputs.
* super beta transistors

LTC2053CMS8-SYNC#PBF


# ADC
AD7177-2BRUZ
* 32bit
* 5 SPS to 10 kSPS
* 24.6 noise free bits at 5 SPS
* INL: ±1 ppm of FSR
* On-chip 2.5 V reference (±2 ppm/°C drift)
* differential input range: ±VREF
* VREF = 1V ~ AVDD1 
* AVDD1 = 4.5 V to 5.5 V

# Voltage Reference
- ADR1000 is a bit better than LTZ1000ACH, however hard to acquire (https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10363412)
  - there is ADR1000H-EBZ which is available

https://edms.cern.ch/ui/file/2820336/1/EDA-04061-V4-1_sch.pdf
* input buffers: OPA189IDG
* VREF=5V (from LTZ1000A)



# amp gain desing
assuming a burden voltage of 170mV
![img_16.png](img/2/img_16.png)

- connect AIN- to Vrefp
- AIN+ is now our single-ended input
- with VREF=5V we can measure 0..5V
  => half of dynamic range
- with VREF = 2.5V we can meausure 0..5V
  => full dynamic range
- 5/0.170 = 29.4
- chose a gain of 30
- lower gain 12.5

#isolated psu
https://de.farnell.com/analog-devices/adr1001e-ebz/evaluationsboard-spannungsreferenz/dp/4364175?mfot=true
