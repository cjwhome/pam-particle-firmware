HIH61XX Library for Spark Core
===================
Honeywell HumidIconTM Digital Humidity/Temperature Sensors: HIH6130/6131 and 6120/6121 Series is a digital output-type relative humidity (RH) and temperature sensor combined in the same package. The library uses I2C to connect to the sensor (see the Wire library). ***SPI can also be used, but is not implemented in this library***.

Note
----
If you have multiple devices connected to the I2C bus, remember that this device screws up the bus when it is not powered. One solution is to keep the device powered all the time. Another possible solution is to add an analog switch to physically disconnect the GND line or the SDA line to the device.

This library has been tested, but might still contain bugs, so please be careful.

About
-----
This project is based on https://code.google.com/p/arduino-hih61xx/ and is part of Tomas's work.