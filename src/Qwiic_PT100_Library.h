/*
  This is a library written for the Qwiic PT100 based on the
  TI ADS122C04 24-Bit 4-Channel 2-kSPS Delta-Sigma ADC With I2C Interface

  SparkFun sells these at its website: www.sparkfun.com
	Do you like this library? Help support SparkFun. Buy a board!

  Written by: Paul Clark (PaulZC)
  Date: May 4th 2020

  Based on the TI datasheet:
  https://www.ti.com/product/ADS122C04
  https://www.ti.com/lit/ds/symlink/ads122c04.pdf
  Using the example code from the "High Precision Temperature Measurement
  for Heat and Cold Meters Reference Design" (TIDA-01526) for reference:
  http://www.ti.com/tool/TIDA-01526
  http://www.ti.com/lit/zip/tidcee5

  The MIT License (MIT)
	Copyright (c) 2020 Paul Clark
	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
	associated documentation files (the "Software"), to deal in the Software without restriction,
	including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
	and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
	do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial
	portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
	NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef QWIIC_PT100_ARDUINO_LIBRARY_H
#define QWIIC_PT100_ARDUINO_LIBRARY_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

//#elif __MK20DX256__
//Teensy

#endif

#ifndef I2C_BUFFER_LENGTH

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32
//#define I2C_BUFFER_LENGTH 16 //For testing on Artemis

#endif

// Define Serial for SparkFun SAMD based boards.
// You may need to comment these three lines if your SAMD board supports Serial (not SerialUSB)
#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
#endif

// Single Conversion Timeout (millis)
// The maximum time we will wait for DRDY to go valid for a single conversion
#define ADS122C04_CONVERSION_TIMEOUT 75

// Define 2/3/4-Wire and Temperature modes
#define ADS122C04_4WIRE_MODE         0x0
#define ADS122C04_3WIRE_MODE         0x1
#define ADS122C04_2WIRE_MODE         0x2
#define ADS122C04_TEMPERATURE_MODE   0x3

// ADS122C04 Table 16 in Datasheet
#define ADS122C04_RESET_CMD          0x06     //0000 011x      Reset
#define ADS122C04_START_CMD          0x08     //0000 100x      Start/Sync
#define ADS122C04_POWERDOWN_CMD      0x02     //0000 001x      PowerDown
#define ADS122C04_RDATA_CMD          0x10     //0001 xxxx      RDATA
#define ADS122C04_RREG_CMD           0x20     //0010 rrxx      Read REG rr= register address 00 to 11
#define ADS122C04_WREG_CMD           0x40     //0100 rrxx      Write REG rr= register address 00 to 11

#define ADS122C04_WRITE_CMD(reg)     (ADS122C04_WREG_CMD | (reg << 2))    //Shift is 2-bit in ADS122C04
#define ADS122C04_READ_CMD(reg)      (ADS122C04_RREG_CMD | (reg << 2))    //Shift is 2-bit in ADS122C04

// ADS122C04 Table 16 in Datasheet
#define ADS122C04_CONFIG_0_REG      0 // Configuration Register 0
#define ADS122C04_CONFIG_1_REG      1 // Configuration Register 1
#define ADS122C04_CONFIG_2_REG      2 // Configuration Register 2
#define ADS122C04_CONFIG_3_REG      3 // Configuration Register 3

/*

// Configuration Register 0
// ADS122C04 Table 19 in Datasheet

// Input Multiplexer Configuration
#define ADS122C04_MUX_MASK          0xf0
#define ADS122C04_MUX_SHIFT         4
#define ADS122C04_MUX_AIN0_AIN1     (0x0 << ADS122C04_MUX_SHIFT)    //default
#define ADS122C04_MUX_AIN0_AIN2     (0x1 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN0_AIN3     (0x2 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN1_AIN0     (0x3 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN1_AIN2     (0x4 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN1_AIN3     (0x5 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN2_AIN3     (0x6 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN3_AIN2     (0x7 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN0_AVSS     (0x8 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN1_AVSS     (0x9 << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN2_AVSS     (0xa << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AIN3_AVSS     (0xb << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_REFPmREFN     (0xc << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_AVDDmAVSS     (0xd << ADS122C04_MUX_SHIFT)
#define ADS122C04_MUX_SHORTED       (0xe << ADS122C04_MUX_SHIFT)

// Gain Configuration
#define ADS122C04_GAIN_MASK         0x0e
#define ADS122C04_GAIN_SHIFT        1
#define ADS122C04_GAIN_1            (0x0 << ADS122C04_GAIN_SHIFT)
#define ADS122C04_GAIN_2            (0x1 << ADS122C04_GAIN_SHIFT)
#define ADS122C04_GAIN_4            (0x2 << ADS122C04_GAIN_SHIFT)
#define ADS122C04_GAIN_8            (0x3 << ADS122C04_GAIN_SHIFT)
#define ADS122C04_GAIN_16           (0x4 << ADS122C04_GAIN_SHIFT)
#define ADS122C04_GAIN_32           (0x5 << ADS122C04_GAIN_SHIFT)
#define ADS122C04_GAIN_64           (0x6 << ADS122C04_GAIN_SHIFT)
#define ADS122C04_GAIN_128          (0x7 << ADS122C04_GAIN_SHIFT)

// PGA Bypass
#define ADS122C04_PGA_ENABLED       1
#define ADS122C04_PGA_DISABLED      0

// Configuration Register 1
// ADS122C04 Table 19 in Datasheet

// Data Rate
#define ADS122C04_DATA_RATE_MASK    0xe0
#define ADS122C04_DATA_RATE_SHIFT   5
// Turbo mode = Normal mode * 2 (Samples per Second)
// Normal mode
#define ADS122C04_DATA_RATE_20SPS   (0 << ADS122C04_DATA_RATE_SHIFT)
#define ADS122C04_DATA_RATE_45SPS   (1 << ADS122C04_DATA_RATE_SHIFT)
#define ADS122C04_DATA_RATE_90SPS   (2 << ADS122C04_DATA_RATE_SHIFT)
#define ADS122C04_DATA_RATE_175SPS  (3 << ADS122C04_DATA_RATE_SHIFT)
#define ADS122C04_DATA_RATE_330SPS  (4 << ADS122C04_DATA_RATE_SHIFT)
#define ADS122C04_DATA_RATE_600SPS  (5 << ADS122C04_DATA_RATE_SHIFT)
#define ADS122C04_DATA_RATE_1000SPS (6 << ADS122C04_DATA_RATE_SHIFT)

// Operating Mode
#define ADS122C04_OP_MODE_MASK      0x10
#define ADS122C04_OP_MODE_SHIFT     4
#define ADS122C04_OP_MODE_NORMAL    (0 << ADS122C04_OP_MODE_SHIFT)
#define ADS122C04_OP_MODE_TURBO     (1 << ADS122C04_OP_MODE_SHIFT)

// Conversion Mode
#define ADS122C04_CONVERSION_MODE_MASK          0x08
#define ADS122C04_CONVERSION_MODE_SHIFT         3
#define ADS122C04_CONVERSION_MODE_SINGLE_SHOT   (0 << ADS122C04_CONVERSION_MODE_SHIFT)
#define ADS122C04_CONVERSION_MODE_CONTINUOUS    (1 << ADS122C04_CONVERSION_MODE_SHIFT)

// Voltage Reference Selection
#define ADS122C04_VREF_MASK                0x06
#define ADS122C04_VREF_SHIFT               1
#define ADS122C04_VREF_INTERNAL            (0 << ADS122C04_VREF_SHIFT)  //2.048V internal
#define ADS122C04_VREF_EXT_REF_PINS        (1 << ADS122C04_VREF_SHIFT)  //REFp and REFn external
#define ADS122C04_VREF_AVDD                (2 << ADS122C04_VREF_SHIFT)  //Analog Supply AVDD and AVSS

// Temperature Sensor Mode
#define ADS122C04_TEMP_SENSOR_ON           1
#define ADS122C04_TEMP_SENSOR_OFF          0

// Configuration Register 2
// ADS122C04 Table 22 in Datasheet

// Conversion Result Ready Flag (READ ONLY)
#define ADS122C04_DRDY_MASK                0x80
#define ADS122C04_DRDY_SHIFT               7

// Data Counter Enable
#define ADS122C04_DCNT_MASK                0x40
#define ADS122C04_DCNT_SHIFT               6
#define ADS122C04_DCNT_DISABLE             (0 << ADS122C04_DCNT_SHIFT)
#define ADS122C04_DCNT_ENABLE              (1 << ADS122C04_DCNT_SHIFT)

// Data Integrity Check Enable
#define ADS122C04_CRC_MASK                 0x30
#define ADS122C04_CRC_SHIFT                4
#define ADS122C04_CRC_DISABLED             (0 << ADS122C04_CRC_SHIFT)
#define ADS122C04_CRC_INVERTED             (1 << ADS122C04_CRC_SHIFT)
#define ADS122C04_CRC_CRC16_ENABLED        (2 << ADS122C04_CRC_SHIFT)

// Burn-Out Current Source
#define ADS122C04_BURN_OUT_MASK            0x08
#define ADS122C04_BURN_OUT_SHIFT           3
#define ADS122C04_BURN_OUT_CURRENT_ON      (1 << ADS122C04_BURN_OUT_SHIFT)
#define ADS122C04_BURN_OUT_CURRENT_OFF     (0 << ADS122C04_BURN_OUT_SHIFT)

// IDAC Current Setting
#define ADS122C04_IDAC_CURRENT_MASK        0x07
#define ADS122C04_IDAC_CURRENT_SHIFT       0
#define ADS122C04_IDAC_CURRENT_OFF         (0 << ADS122C04_IDAC_CURRENT_SHIFT)
#define ADS122C04_IDAC_CURRENT_10_UA       (1 << ADS122C04_IDAC_CURRENT_SHIFT)
#define ADS122C04_IDAC_CURRENT_50_UA       (2 << ADS122C04_IDAC_CURRENT_SHIFT)
#define ADS122C04_IDAC_CURRENT_100_UA      (3 << ADS122C04_IDAC_CURRENT_SHIFT)
#define ADS122C04_IDAC_CURRENT_250_UA      (4 << ADS122C04_IDAC_CURRENT_SHIFT)
#define ADS122C04_IDAC_CURRENT_500_UA      (5 << ADS122C04_IDAC_CURRENT_SHIFT)
#define ADS122C04_IDAC_CURRENT_1000_UA     (6 << ADS122C04_IDAC_CURRENT_SHIFT)
#define ADS122C04_IDAC_CURRENT_1500_UA     (7 << ADS122C04_IDAC_CURRENT_SHIFT)

// Configuration Register 3
// ADS122C04 Table 23 in Datasheet

// IDAC1 Routing Configuration
#define ADS122C04_IDAC1_MUX_MASK           0xe0
#define ADS122C04_IDAC1_MUX_SHIFT          5
#define ADS122C04_IDAC1_DISABLED           (0 << ADS122C04_IDAC1_MUX_SHIFT)
#define ADS122C04_IDAC1_AIN0               (1 << ADS122C04_IDAC1_MUX_SHIFT)
#define ADS122C04_IDAC1_AIN1               (2 << ADS122C04_IDAC1_MUX_SHIFT)
#define ADS122C04_IDAC1_AIN2               (3 << ADS122C04_IDAC1_MUX_SHIFT)
#define ADS122C04_IDAC1_AIN3               (4 << ADS122C04_IDAC1_MUX_SHIFT)
#define ADS122C04_IDAC1_REFP               (5 << ADS122C04_IDAC1_MUX_SHIFT)
#define ADS122C04_IDAC1_REFN               (6 << ADS122C04_IDAC1_MUX_SHIFT)

// IDAC2 Routing Configuration
#define ADS122C04_IDAC2_MUX_MASK           0x1c
#define ADS122C04_IDAC2_MUX_SHIFT          2
#define ADS122C04_IDAC2_DISABLED           (0 << ADS122C04_IDAC2_MUX_SHIFT)
#define ADS122C04_IDAC2_AIN0               (1 << ADS122C04_IDAC2_MUX_SHIFT)
#define ADS122C04_IDAC2_AIN1               (2 << ADS122C04_IDAC2_MUX_SHIFT)
#define ADS122C04_IDAC2_AIN2               (3 << ADS122C04_IDAC2_MUX_SHIFT)
#define ADS122C04_IDAC2_AIN3               (4 << ADS122C04_IDAC2_MUX_SHIFT)
#define ADS122C04_IDAC2_REFP               (5 << ADS122C04_IDAC2_MUX_SHIFT)
#define ADS122C04_IDAC2_REFN               (6 << ADS122C04_IDAC2_MUX_SHIFT)

*/

// Unshifted register definitions
// The bit field register definitions will do the bit shifting

// Configuration Register 0
// ADS122C04 Table 19 in Datasheet

// Input Multiplexer Configuration
#define ADS122C04_MUX_AIN0_AIN1     0x0
#define ADS122C04_MUX_AIN0_AIN2     0x1
#define ADS122C04_MUX_AIN0_AIN3     0x2
#define ADS122C04_MUX_AIN1_AIN0     0x3
#define ADS122C04_MUX_AIN1_AIN2     0x4
#define ADS122C04_MUX_AIN1_AIN3     0x5
#define ADS122C04_MUX_AIN2_AIN3     0x6
#define ADS122C04_MUX_AIN3_AIN2     0x7
#define ADS122C04_MUX_AIN0_AVSS     0x8
#define ADS122C04_MUX_AIN1_AVSS     0x9
#define ADS122C04_MUX_AIN2_AVSS     0xa
#define ADS122C04_MUX_AIN3_AVSS     0xb
#define ADS122C04_MUX_REFPmREFN     0xc
#define ADS122C04_MUX_AVDDmAVSS     0xd
#define ADS122C04_MUX_SHORTED       0xe

// Gain Configuration
#define ADS122C04_GAIN_1            0x0
#define ADS122C04_GAIN_2            0x1
#define ADS122C04_GAIN_4            0x2
#define ADS122C04_GAIN_8            0x3
#define ADS122C04_GAIN_16           0x4
#define ADS122C04_GAIN_32           0x5
#define ADS122C04_GAIN_64           0x6
#define ADS122C04_GAIN_128          0x7

// PGA Bypass
#define ADS122C04_PGA_DISABLED      0x0
#define ADS122C04_PGA_ENABLED       0x1

// Configuration Register 1
// ADS122C04 Table 19 in Datasheet

// Data Rate
// Turbo mode = Normal mode * 2 (Samples per Second)
// Normal mode
#define ADS122C04_DATA_RATE_20SPS   0x0
#define ADS122C04_DATA_RATE_45SPS   0x1
#define ADS122C04_DATA_RATE_90SPS   0x2
#define ADS122C04_DATA_RATE_175SPS  0x3
#define ADS122C04_DATA_RATE_330SPS  0x4
#define ADS122C04_DATA_RATE_600SPS  0x5
#define ADS122C04_DATA_RATE_1000SPS 0x6

// Operating Mode
#define ADS122C04_OP_MODE_NORMAL    0x0
#define ADS122C04_OP_MODE_TURBO     0x1

// Conversion Mode
#define ADS122C04_CONVERSION_MODE_SINGLE_SHOT   0x0
#define ADS122C04_CONVERSION_MODE_CONTINUOUS    0x1

// Voltage Reference Selection
#define ADS122C04_VREF_INTERNAL            0x0 //2.048V internal
#define ADS122C04_VREF_EXT_REF_PINS        0x1 //REFp and REFn external
#define ADS122C04_VREF_AVDD                0x2 //Analog Supply AVDD and AVSS

// Temperature Sensor Mode
#define ADS122C04_TEMP_SENSOR_OFF          0x0
#define ADS122C04_TEMP_SENSOR_ON           0x1

// Configuration Register 2
// ADS122C04 Table 22 in Datasheet

// Conversion Result Ready Flag (READ ONLY)

// Data Counter Enable
#define ADS122C04_DCNT_DISABLE             0x0
#define ADS122C04_DCNT_ENABLE              0x1

// Data Integrity Check Enable
#define ADS122C04_CRC_DISABLED             0x0
#define ADS122C04_CRC_INVERTED             0x1
#define ADS122C04_CRC_CRC16_ENABLED        0x2

// Burn-Out Current Source
#define ADS122C04_BURN_OUT_CURRENT_OFF     0x0
#define ADS122C04_BURN_OUT_CURRENT_ON      0x1

// IDAC Current Setting
#define ADS122C04_IDAC_CURRENT_OFF         0x0
#define ADS122C04_IDAC_CURRENT_10_UA       0x1
#define ADS122C04_IDAC_CURRENT_50_UA       0x2
#define ADS122C04_IDAC_CURRENT_100_UA      0x3
#define ADS122C04_IDAC_CURRENT_250_UA      0x4
#define ADS122C04_IDAC_CURRENT_500_UA      0x5
#define ADS122C04_IDAC_CURRENT_1000_UA     0x6
#define ADS122C04_IDAC_CURRENT_1500_UA     0x7

// Configuration Register 3
// ADS122C04 Table 23 in Datasheet

// IDAC1 Routing Configuration
#define ADS122C04_IDAC1_DISABLED           0x0
#define ADS122C04_IDAC1_AIN0               0x1
#define ADS122C04_IDAC1_AIN1               0x2
#define ADS122C04_IDAC1_AIN2               0x3
#define ADS122C04_IDAC1_AIN3               0x4
#define ADS122C04_IDAC1_REFP               0x5
#define ADS122C04_IDAC1_REFN               0x6

// IDAC2 Routing Configuration
#define ADS122C04_IDAC2_DISABLED           0x0
#define ADS122C04_IDAC2_AIN0               0x1
#define ADS122C04_IDAC2_AIN1               0x2
#define ADS122C04_IDAC2_AIN2               0x3
#define ADS122C04_IDAC2_AIN3               0x4
#define ADS122C04_IDAC2_REFP               0x5
#define ADS122C04_IDAC2_REFN               0x6

// Bit field type register configuration
// Configuration Map register ADS122C04
//--------------Address 0x00---------------------------------
struct CONFIG_REG_0{
  uint8_t PGA_BYPASS:1;                           // 0
  uint8_t GAIN:3;                                 // 1-3
  uint8_t MUX:4;                                  // 4-7
};
union CONFIG_REG_0_U {
  uint8_t all;
  struct CONFIG_REG_0 bit;
};

//--------------Address 0x01---------------------------------
struct CONFIG_REG_1{
  uint8_t TS:1;                                   // 0
  uint8_t VREF:2;                                 // 1-2
  uint8_t CMBIT:1;                                // 3
  uint8_t MODE:1;                                 // 4
  uint8_t DR:3;                                   // 5-7
};
union CONFIG_REG_1_U {
  uint8_t all;
  struct CONFIG_REG_1 bit;
};

//--------------Address 0x02---------------------------------
struct CONFIG_REG_2{
  uint8_t IDAC:3;                                 // 0-2
  uint8_t BCS:1;                                  // 3
  uint8_t CRC:2;                                  // 4-5
  uint8_t DCNT:1;                                 // 6
  uint8_t DRDY:1;                                 // 7
};
union CONFIG_REG_2_U {
  uint8_t all;
  struct CONFIG_REG_2 bit;
};

//--------------Address 0x03---------------------------------
struct CONFIG_REG_3{
  uint8_t RESERVED:2;                             // 0-1
  uint8_t I2MUX:3;                                // 2-4
  uint8_t I1MUX:3;                                // 5-7
};
union CONFIG_REG_3_U {
  uint8_t all;
  struct CONFIG_REG_3 bit;
};

// All four registers
typedef struct ADS122C04Reg{
  union CONFIG_REG_0_U reg0;
  union CONFIG_REG_1_U reg1;
  union CONFIG_REG_2_U reg2;
  union CONFIG_REG_3_U reg3;
}ADS122C04Reg_t;

// Union for the 14-bit internal Temperature
// To simplify converting from uint16_t to int16_t
// without using a cast
union internal_temperature_union{
  int16_t INT16;
  uint16_t UINT16;
};

// Union for the 24-bit raw voltage
// To simplify converting from uint32_t to int32_t
// without using a cast
union raw_voltage_union{
  int32_t INT32;
  uint32_t UINT32;
};

// struct to hold the initialisation parameters
typedef struct{
  uint8_t inputMux;
  uint8_t gainLevel;
  uint8_t pgaBypass;
  uint8_t dataRate;
  uint8_t opMode;
  uint8_t convMode;
  uint8_t selectVref;
  uint8_t tempSensorEn;
  uint8_t dataCounterEn;
  uint8_t dataCRCen;
  uint8_t burnOutEn;
  uint8_t idacCurrent;
  uint8_t routeIDAC1;
  uint8_t routeIDAC2;
} ADS122C04_initParam;

class SFE_QWIIC_PT100
{
public:
	SFE_QWIIC_PT100(void);

  //By default use the default I2C address, and use Wire port
	boolean begin(uint8_t deviceAddress = 0x45, TwoWire &wirePort = Wire); //Returns true if module is detected

  //Returns true if device answers on _deviceAddress
	boolean isConnected(void);

  void enableDebugging(Stream &debugPort = Serial); // enable debug messages
  void disableDebugging(); // disable debug messages

  float readPT100Centigrade(void); // Read the PT100 temperature in Centigrade
  float readPT100Fahrenheit(void); // Read the PT100 temperature in Fahrenheit

  // Read the raw signed 24-bit ADC value as int32_t
  // The result needs to be multiplied by VREF / GAIN to convert to Volts
  int32_t readRawVoltage(void);

  // Read the internal temperature (C)
  float readInternalTemperature(void);

  boolean reset(void); // Reset the ADS122C04
  boolean start(void); // Start a conversion
  boolean powerdown(void); // Put the chip into low power mode

  boolean configure234wire(uint8_t wire_mode = ADS122C04_4WIRE_MODE); // Configure the chip for the chosen mode

  boolean setInputMultiplexer(uint8_t mux_config = ADS122C04_MUX_AIN1_AIN0); // Configure the input multiplexer
  boolean setGain(uint8_t gain_config = ADS122C04_GAIN_8); // Configure the gain
  boolean enablePGA(uint8_t enable = ADS122C04_PGA_ENABLED); // Enable/disable the Programmable Gain Amplifier
  boolean setDataRate(uint8_t rate = ADS122C04_DATA_RATE_20SPS); // Set the data rate (sample speed)
  boolean setOperatingMode(uint8_t mode = ADS122C04_OP_MODE_NORMAL); // Configure the operating mode (normal / turbo)
  boolean setConversionMode(uint8_t mode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT); // Configure the conversion mode (single-shot / continuous)
  boolean setVoltageReference(uint8_t ref = ADS122C04_VREF_EXT_REF_PINS); // Configure the voltage reference
  boolean enableInternalTempSensor(uint8_t enable = ADS122C04_TEMP_SENSOR_OFF); // Enable / disable the internal temperature sensor
  boolean setDataCounter(uint8_t enable = ADS122C04_DCNT_DISABLE); // Enable / disable the conversion data counter
  boolean setDataIntegrityCheck(uint8_t setting = ADS122C04_CRC_DISABLED); // Configure the data integrity check
  boolean setBurnOutCurrent(uint8_t enable = ADS122C04_BURN_OUT_CURRENT_OFF); // Enable / disable the 10uA burn-out current source
  boolean setIDACcurrent(uint8_t current = ADS122C04_IDAC_CURRENT_1000_UA); // Configure the internal programmable current sources
  boolean setIDAC1mux(uint8_t setting = ADS122C04_IDAC1_AIN3); // Configure the IDAC1 routing
  boolean setIDAC2mux(uint8_t setting = ADS122C04_IDAC2_DISABLED); // Configure the IDAC2 routing

  boolean checkDataReady(void); // Check the status of the DRDY bit in Config Register 2

  uint8_t getInputMultiplexer(void); // Get the input multiplexer configuration
  uint8_t getGain(void); // Get the gain setting
  uint8_t getPGAstatus(void); // Get the Programmable Gain Amplifier status
  uint8_t getDataRate(void); // Get the data rate (sample speed)
  uint8_t getOperatingMode(void); // Get the operating mode (normal / turbo)
  uint8_t getConversionMode(void); // Get the conversion mode (single-shot / continuous)
  uint8_t getVoltageReference(void); // Get the voltage reference configuration
  uint8_t getInternalTempSensorStatus(void); // Get the internal temperature sensor status
  uint8_t getDataCounter(void); // Get the data counter status
  uint8_t getDataIntegrityCheck(void); // Get the data integrity check configuration
  uint8_t getBurnOutCurrent(void); // Get the burn-out current status
  uint8_t getIDACcurrent(void); // Get the IDAC setting
  uint8_t getIDAC1mux(void); // Get the IDAC1 mux configuration
  uint8_t getIDAC2mux(void); // Get the IDAC2 mux configuration

private:
  //Variables
	TwoWire *_i2cPort;		//The generic connection to user's chosen I2C hardware
	uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.

	Stream *_debugPort;			 //The stream to send debug messages to if enabled. Usually Serial.
	boolean _printDebug = false; //Flag to print debugging variables

  // Keep a copy of the wire mode so we can restore it after reading the internal temperature
  uint8_t _wireMode = ADS122C04_4WIRE_MODE;

  // Resistance of the reference resistor
  const float PT100_REFERENCE_RESISTOR = 1620.0;

  // Amplifier gain setting
  // ** MAKE SURE THE CONFIG REGISTER 0 GAIN IS THE SAME AS THIS **
  const float PT100_AMPLIFIER_GAIN = 8.0;

  // Internal temperature sensor resolution
  // One 14-bit LSB equals 0.03125°C
  const float TEMPERATURE_SENSOR_RESOLUTION = 0.03125;

  ADS122C04Reg_t ADS122C04_Reg; // Global to hold copies of all four configuration registers

  void debugPrint(char *message); // print a debug message
  void debugPrintln(char *message); // print a debug message with line feed

  boolean ADS122C04_init(ADS122C04_initParam *param); // initialise the ADS122C04 with these parameters

  boolean ADS122C04_writeReg(uint8_t reg, uint8_t writeValue); // write a value to the selected register
  boolean ADS122C04_readReg(uint8_t reg, uint8_t *readValue); // read a value from the selected register (returned in readValue)

  boolean ADS122C04_getConversionData(uint32_t *conversionData); // read the raw 24-bit conversion result
  boolean ADS122C04_getConversionDataWithCount(uint32_t *conversionData, uint8_t *count); // read the raw conversion result and count (if enabled)

  boolean ADS122C04_sendCommand(uint8_t command); // write to the selected command register
  boolean ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value); // write a value to the selected command register
};
#endif
