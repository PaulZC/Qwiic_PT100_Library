# Qwiic PT100 Library

**A library to support the TI ADS122C04 on the Qwiic PT100**

The Qwiic PT100 is based on the TI ADS122C04 and allows you to measure temperature accurately from
100 Ohm Platinum Resistance Thermometers using 2, 3 or 4-wire connections.

The ADS122C04 is a precision 24-bit delta-sigma analog to digital converter. It can measure voltage very
accurately through two differential or four single-ended inputs. It has a flexible input multiplexer,
a low-noise programmable gain amplifier, two programmable current sources, a voltage reference
and an in-built precision temperature sensor. In summary, it contains everything you need to measure
temperature accurately from remote sensors. This library will let you access all of those features
should you need to.

The Qwiic PT100 is dedicated to reading the temperature of 100 Ohm Platinum Resistance Thermometers
using 2, 3 or 4-wire connections. Using a 3 or 4-wire connection allows the ADS122C04 to automatically
compensate for the cable resistance. The differential input provides noise rejection for industrial
applications.

## Repository Contents

- **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
- **/src** - Source files for the library (.cpp, .h).
- **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
- **library.properties** - General library properties for the Arduino package manager.

## Documentation

- **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
- **[Product Repository](https://github.com/PaulZC/Qwiic_PT100)** - Main repository (including hardware files)

## License Information

This product is _**open source**_!

Please use, reuse, and modify these files as you see fit.
Please maintain the attribution and release anything derivative under the same license.

Distributed as-is; no warranty is given.

Enjoy!

**_Paul_**
