
* datasheet says INL(typ) = 20ppm
* Linearity can be improved with a second order polynom (~2ppm INL)
* polynom is the same for the three tested chips
* non-linearity does not appear to drift
* gain drifts, need to calibrate gain before each measurement
* offset didnt drift across measurements

# INL
![ina228_inl.png](ina228_inl.png)


- reference HP3458A (nplc=10)
- good INL
- suitable for polyfit
- only Vbus+dietemp measured (conversion time ~4000us)

across devices the curve looks pretty much the same:
![inl2.png](inl2.png)


going up to 82V reveals more linear error (the DMM6500 continues to do its job very well):
![ina228_inl_.png](ina228_inl_80v.png)

notice these measurements where only taken for 90s each with some noise on the power supply.

here is a measurement with another less noisy power supply and 900s (15min) measurements:
TODO 

this looks like we can do a polyfit, and indeed it helps.





# Noise

![img_7.png](img_7.png)
![img_8.png](img_8.png)


# etc

current measurements seem to suffer from crosstalk from bus voltage. it changes the offset voltage.
for better precision, use one ina228 for current only and the other for voltage only.
Vbus INL is quite good.
TODO need to measure INL for current



Ina228 with ESP32-S3, WiFi connected

![img_1.png](img_1.png)

The Esp32 produces significant heat.

Current sense input:
10 ohm series resistors and 100nF cap

6h drift, temperature droping at night: (Vbus open clamps)
![img_2.png](img_2.png)
![img_3.png](img_3.png)


suddenly, there was an unexplainable drop for 45 minutes:

![img_4.png](img_4.png)

shortings Vbus to GND:
![img_5.png](img_5.png)
![img_6.png](img_6.png)

- offset voltage of current amp falls near zero.
- ADC temperature rises (due to more conversion iterations?)
- after opening Vbus again it returns to the same offset as before
- connecting a 3.2V battery doesnt change a lot


![img.png](img.png)




# Offset Voltage

- best to connect shunt to circuit GND
- change Vbus from 0 to 60V -> current offset voltage changes by 1.4µV
- 