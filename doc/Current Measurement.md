* higher burden voltage, lower offset error, less noise, better precision
* keep the burden resistor heated at stable temperature, to avoid temp drift
* place first gain amp close to the resistor (on the same PCB). connectors might add offset voltage due to
  thermoelectric effect between different metals. and long cables add noise. temperature of the PCB will be regulated
  anyways so temp drift of the gain amp will not matter.

# adc

* low drift

from ina228:
"The ADC architecture enables
lower drift measurement across temperature and consistent offset measurements across the common-mode
voltage, temperature, and power supply variations. A low-offset ADC is preferred in current sensing applications
to provide a near 0-V offset voltage that maximizes the useful dynamic range of the system."

# Avoid Jumper Cables

When measuring voltages in the mV range, avoid jumper cables. Instead use soldered connections with tinned copper
cables.

Here's an experiment:
This 2.54mm pitch connector is placed between a DM6500 and a 40mV voltage source.
(The 40mV come from a 2mΩ current sense resistor).
A heat gun set to 100°C is pointed at the connector, with T_Ambient = 20°C.
![img_2.png](img/img_2.png)

The orange line is the voltage measured by an INA228 chip on the same board with the resistor.
The green line is the voltage from the DM6500. Heat was added where the green line abruptly falls (17:49:07).
At 17:49:45 the hot air was removed.
![img_1.png](img/img_1.png)

* A temp rise by 80°C adds 0.15% error
* the connector in the test has a temperature drift ~20ppm/°C at 40mV
* that's 0.8µV/°C
* connectors have a temp drift (compare to Seebeck effect, [thermocouples](https://en.wikipedia.org/wiki/Thermocouple

Here's another example:
![img_5.png](img/img_5.png)

Two smart-shunt boards with ina228, supply and i2c isolated, the left board has a current sense resistor populated (
2mΩ).
The board on the right is left without CSR and the current sense inputs are connected in parallel to the one on the left
with a
standard
jumper cable. The connector pins (PRPC040SACN-RC) are made of brass with flash gold plating (thin gold and nickel
layer). The jumper wire terminals have a tin plating (similar to
JST [eXH](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf), see below).

![img_3.png](img/img_3.png)
The yellow curve ist what the board on the right (ESP32_INA228) measured over time and is drifting from the jumper
cables. Peak-to-peak: 4mA which is 8µV here.
* Orange: what the board on the left measured.
* Blue: a third board for reference with same CSR.
* Green: DM6500 measuring voltage across a Riedon RSN-20-50B (20A/50mV). All devices measure the same current, which was ~300µA here.

I then replaced the jumper cable with wires soldered to the PCBs:
![img_4.png](img/img_4.png)

Tada, the drift is gone!

Notice that this time the current was actually 0, which only offset all curves.

This also tells us that the offset voltage of all 3 INA228 chips is 2µV (1mA).
The INA228 Datasheet says 1µV max.


A smoothin filter:
![img_6.png](img/img_6.png)

Reveals the `ESP32_INA228` and the `ESP32_INA228_3` both have a AC component with a period of 6 seconds.
TODO

# Jumper Wire

JST [eXH](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf)).

[molex SL connectors](https://www.molex.com/en-us/products/connectors/wire-to-wire-connectors/sl-connectors) (
0016020103, 0016020087) have a gold coating.

See digikey for
more [contacts](https://www.digikey.de/en/products/filter/rectangular-connectors/rectangular-connector-contacts/331)
and [housings](https://www.digikey.de/en/products/filter/rectangular-connectors/rectangular-connector-housings/319).

Extra-long 2.5 pitch pin header with flash gold plating: PRPC040SACN-RC .


# Low Thermal EMF connectors
https://www.eevblog.com/forum/metrology/source-for-unplated-copper-fork-lugs/msg5804671/#msg5804671
