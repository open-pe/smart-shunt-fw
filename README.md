A versatile lab power monitor.
I wrote this for precision power metering and testing DCDC converters.

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
* Gossen Metrawatt Metrahit Energy (Starline Series) and Open-Source [IrDA USB](http://lemmini.de/IrDA%20USB/IrDA%20USB.html) (the Gossen USB X-Tra needs proprietary drivers, for windows only)
  * ESP32 has `UART_MODE_IRDA`, you only need the TFDU4101 [forum1](https://esp32.com/viewtopic.php?t=2766)

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