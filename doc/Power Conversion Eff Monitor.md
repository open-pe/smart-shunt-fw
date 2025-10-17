
eff = Pout / Pin = (Uout*Iout) / (Uin*Iin)

Absolute errors can be mostly be eliminated from the result by calibration.
We just need to calibrate the device measuring Uout and Uin to the same reference.
Then its mostly about linearity, which we can also cross check.

To elimante the drift of the ADC we can use one ADC for Uout and Uin (multiplexing)
A second ADC measures Iin and Iout. Gain error eliminated. Linearity is still important.

If we use a ADC with high resolution, and Uout is in the same range of Uin, we can do the multiplexing *after* 
the voltage divider resistors. This way


* can use a DMM with external trigger to multiplex inputs between input and output
* DMM6500 has secondary measurement and voltage ratio!


For current measuring need lower thermal emf switches
todo marco reps videos about the hp
https://www.eevblog.com/forum/projects/diy-low-thermal-emf-switchscanner-for-comparisons-of-voltage-and-resistor-stand/50/
g6ku-2f-y
maybe use mosfets. low 



Multiplex Front-End


* high-current multiplex of shunt resistor
  * eliminates gain error due to resistor TCO
  * bypass mux to measure offset voltage (thermal EMF)
* works best if voltage ratio is not too different
  * or use 32-bit adc
* add trigger port for DMM mux