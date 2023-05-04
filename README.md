
A versatile energy monitor.
I wrote this for precision power metering.

Supported ADC:
* ADS1115 / ADS 1015
* INA226

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
