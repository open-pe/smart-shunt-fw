# Precision Considerations
* Gain error is typical 0.01%
* The temperature gain drift is 7ppm/°C at FSR 0.256V
* For all other ranges the data sheet states typ 5ppm/°C and max 40ppm/°C
* the ADS1118 features a temperature sensor

# Voltage sensing
* Use precision resistors with low temperature coefficient
* consider the ADC input impedance (3-100MΩ depending on range)
  * 6MΩ at ±2.048 V
* Example of voltage divider values for FSR 2.048V:
  * 80V: 5k/200k 
  * 40V: 5k/100k


## Analog Aliasing filter
* As you can read in the datasheet, the analog aliasing filter should have a cut-off freq of 1x to 10x the sampling rate
* the voltage divider source resistance is nearly equal the resistance of the smaller resistors, e.g. 5k
* with 5k and 22nF cut-off is at 1448 Hz


# Current Sensing with SSA-100
* 
The SSA-100 has a common mode voltage of 1.44V. With a sensitivity 12.5mV/A.