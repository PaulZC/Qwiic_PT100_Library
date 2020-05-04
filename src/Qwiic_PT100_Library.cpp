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

#include "Qwiic_PT100_Library.h"

ADS122C04Reg_t ADS122C04_Reg; // Global to hold copies of all four registers

//Attempt communication with the device and initialise it
//Return true if successful
boolean SFE_QWIIC_PT100::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
	_deviceAddress = deviceAddress; //If provided, store the I2C address from user
	_i2cPort = &wirePort; //Grab which port the user wants us to use
	_printDebug = false; //Flag to print debugging variables

	delay(1); // wait for power-on reset to complete (datasheet says we should do this)

	if (isConnected() == false)
	{
		debugPrintln("begin: isConnected returned false");
		return (false);
	}

	reset(); // reset the ADS122C04 (datasheet says we should do this)

	// Initialise the parameters for PT100 measurement
	ADS122C04_initParam initParams;
	initParams.inputMux = ADS122C04_MUX_AIN0_AIN1;
	initParams.gainLevel = ADS122C04_GAIN_1;
	initParams.pgaBypass = ADS122C04_PGA_ENABLED;
	initParams.dataRate = ADS122C04_DATA_RATE_20SPS;
	initParams.opMode = ADS122C04_OP_MODE_NORMAL;
	initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT;
	initParams.selectVref = ADS122C04_VREF_INTERNAL;
	initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;
	initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;
	initParams.dataCRCen = ADS122C04_CRC_DISABLED;
	initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;
	initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF;
	initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED;
	initParams.routeIDAC2 = ADS122C04_IDAC1_DISABLED;

	return(ADS122C04_init(&initParams));
}

//Returns true if device answers on _deviceAddress
boolean SFE_QWIIC_PT100::isConnected(void)
{
	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
	return (_i2cPort->endTransmission() == 0);
}

//Enable or disable the printing of debug messages
void SFE_QWIIC_PT100::enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging
  _printDebug = true; //Should we print the commands we send? Good for debugging
}

void SFE_QWIIC_PT100::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

//Safely print messages
void SFE_QWIIC_PT100::debugPrint(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial->print(message);
  }
}

//Safely print messages
void SFE_QWIIC_PT100::debugPrintln(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial->println(message);
  }
}

float SFE_QWIIC_PT100::readPT100Centigrade(void) // Read the temperature in Centigrade
{

}

float SFE_QWIIC_PT100::readPT100Fahrenheit(void) // Read the temperature in Fahrenheit
{

}

// Read the raw signed 24-bit ADC value as int32_t
// The result needs to be multiplied by VREF / GAIN to convert to Volts
int32_t SFE_QWIIC_PT100::readRawVoltage(void)
{
	raw_voltage_union raw_v; // union to convert uint32_t to int32_t
	unsigned long start_time = millis(); // Record the start time so we can timeout
	boolean drdy = false; // DRDY (1 == new data is ready)

	// Start the conversion (assumes we are using single shot mode)
	start();

	// Wait for DRDY to go valid
	while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
	{
		delay(5); // Don't pound the bus too hard
		drdy = checkDataReady();
	}

	// Check if we timed out
	if (drdy == false)
	{
		debugPrintln("readRawVoltage: checkDataReady timed out");
		return(0);
	}

	// Read the conversion result
	if(ADS122C04_getConversionData(&raw_v.UINT32) == false)
	{
		debugPrintln("readRawVoltage: ADS122C04_getConversionData failed");
		return(0);
	}

	// The raw voltage is in the bottom 24 bits of raw_temp
	// If we just do a <<8 we will multiply the result by 256
	// Instead pad out the MSB with the MS bit of the 24 bits
	if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
		raw_v.UINT32 |= 0xFF000000;
	return(raw_v.INT32);
}

// Read the internal temperature (degrees C * 0.0078125)
// (14-bit signed converted to 16-bit signed)
float readInternalTemperature(void)
{
	internal_temperature_union int_temp; // union to convert uint16_t to int16_t
	uint32_t raw_temp; // The raw temperature from the ADC
	unsigned long start_time = millis(); // Record the start time so we can timeout
	boolean drdy = false; // DRDY (1 == new data is ready)
	float ret_val = 0.0; // The return value

	// Enable the internal temperature sensor
	// Reading the ADC value will return the temperature
	enableInternalTempSensor(ADS122C04_TEMP_SENSOR_ON);
	// Start the conversion
	start();

	// Wait for DRDY to go valid
	while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
	{
		delay(5); // Don't pound the bus too hard
		drdy = checkDataReady();
	}

	// Check if we timed out
	if (drdy == false)
	{
		debugPrintln("readInternalTemperature: checkDataReady timed out");
		return(ret_val);
	}

	// Read the conversion result
	if(ADS122C04_getConversionData(&raw_temp) == false)
	{
		debugPrintln("readInternalTemperature: ADS122C04_getConversionData failed");
		return(ret_val);
	}
	// Disable the temperature sensor
	enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF);

	// The temperature is in the top 14 bits of the bottom 24 bits of raw_temp
	int_temp.UINT16 = (uint16_t)(raw_temp >> 8);
	debugPrint("readInternalTemperature: raw_temp = 0x");
	debugPrintln(int_temp.INT16);
	ret_val = ()(float)int_temp.INT16) * 0.0078125; // Convert to float including the 2 bit shift
	return(ret_val);
}

void SFE_QWIIC_PT100::setInputMultiplexer(uint8_t mux_config)
{
	ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
	ADS122C04_Reg.reg0.bit.MUX = mux_config;
	ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
}

void SFE_QWIIC_PT100::setGain(uint8_t gain_config)
{
	ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
	ADS122C04_Reg.reg0.bit.GAIN = gain_config;
	ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
}

void SFE_QWIIC_PT100::enablePGA(uint8_t enable)
{
	ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
	ADS122C04_Reg.reg0.bit.PGA_BYPASS = enable;
	ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
}

void SFE_QWIIC_PT100::setDataRate(uint8_t rate)
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	ADS122C04_Reg.reg1.bit.TS = rate;
	ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
}

void SFE_QWIIC_PT100::setOperatingMode(uint8_t mode)
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	ADS122C04_Reg.reg1.bit.MODE = mode;
	ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
}

void SFE_QWIIC_PT100::setConversionMode(uint8_t mode)
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	ADS122C04_Reg.reg1.bit.CMBIT = mode;
	ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
}

void SFE_QWIIC_PT100::setVoltageReference(uint8_t ref)
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	ADS122C04_Reg.reg1.bit.VREF = ref;
	ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
}

void SFE_QWIIC_PT100::enableInternalTempSensor(uint8_t enable)
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	ADS122C04_Reg.reg1.bit.TS = enable;
	ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
}

void SFE_QWIIC_PT100::setDataCounter(uint8_t enable)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	ADS122C04_Reg.reg2.bit.DCNT = enable;
	ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
}

void SFE_QWIIC_PT100::setDataIntegrityCheck(uint8_t setting)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	ADS122C04_Reg.reg2.bit.CRC = setting;
	ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
}

void SFE_QWIIC_PT100::setBurnOutCurrent(uint8_t enable)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	ADS122C04_Reg.reg2.bit.BCS = enable;
	ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
}

void SFE_QWIIC_PT100::setIDACcurrent(uint8_t current)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	ADS122C04_Reg.reg2.bit.IDAC = current;
	ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
}

void SFE_QWIIC_PT100::setIDAC1mux(uint8_t setting)
{
	ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
	ADS122C04_Reg.reg3.bit.I1MUX = setting;
	ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all);
}

void SFE_QWIIC_PT100::setIDAC2mux(uint8_t setting)
{
	ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
	ADS122C04_Reg.reg3.bit.I2MUX = setting;
	ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all);
}

// Read Config Reg 2 and check the DRDY bit
// Data is ready when DRDY is high
boolean SFE_QWIIC_PT100::checkDataReady(void)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	return(ADS122C04_Reg.reg2.bit.DRDY > 0);
}

uint8_t SFE_QWIIC_PT100::getInputMultiplexer(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
	return(ADS122C04_Reg.reg0.bit.MUX);
}

uint8_t SFE_QWIIC_PT100::getGain(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
	return(ADS122C04_Reg.reg0.bit.GAIN);
}

uint8_t SFE_QWIIC_PT100::getPGAstatus(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
	return(ADS122C04_Reg.reg0.bit.PGA_BYPASS);
}

uint8_t SFE_QWIIC_PT100::getDataRate(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	return(ADS122C04_Reg.reg1.bit.DR);
}

uint8_t SFE_QWIIC_PT100::getOperatingMode(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	return(ADS122C04_Reg.reg1.bit.MODE);
}

uint8_t SFE_QWIIC_PT100::getConversionMode(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	return(ADS122C04_Reg.reg1.bit.CMBIT);
}

uint8_t SFE_QWIIC_PT100::getVoltageReference(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	return(ADS122C04_Reg.reg1.bit.VREF);
}

uint8_t SFE_QWIIC_PT100::getInternalTempSensorStatus(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
	return(ADS122C04_Reg.reg1.bit.TS);
}

uint8_t SFE_QWIIC_PT100::getDataCounter(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	return(ADS122C04_Reg.reg2.bit.DCNT);
}

uint8_t SFE_QWIIC_PT100::getDataIntegrityCheck(void);
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	return(ADS122C04_Reg.reg2.bit.CRC);
}

uint8_t SFE_QWIIC_PT100::getBurnOutCurrent(void)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	return(ADS122C04_Reg.reg2.bit.BCS);
}

uint8_t SFE_QWIIC_PT100::getIDACcurrent(void)
{
	ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
	return(ADS122C04_Reg.reg2.bit.IDAC);
}

uint8_t SFE_QWIIC_PT100::getIDAC1mux(void)
{
	ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
	return(ADS122C04_Reg.reg3.bit.I1MUX);
}

uint8_t SFE_QWIIC_PT100::getIDAC2mux(void)
{
	ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
	return(ADS122C04_Reg.reg3.bit.I2MUX);
}

// Update ADS122C04_Reg and initialise the ADS122C04 using the supplied parameters
boolean SFE_QWIIC_PT100::ADS122C04_init(ADS122C04_initParam *param)
{
	ADS122C04_Reg.reg0.all = 0; // Reset all four register values to the default value of 0x00
	ADS122C04_Reg.reg1.all = 0;
	ADS122C04_Reg.reg2.all = 0;
	ADS122C04_Reg.reg3.all = 0;

	ADS122C04_Reg.reg0.bit.MUX = param->inputMux;
	ADS122C04_Reg.reg0.bit.GAIN = param->gainLevel;
	ADS122C04_Reg.reg0.bit.PGA_BYPASS = param->pgaBypass;

	ADS122C04_Reg.reg1.bit.DR = param->dataRate;
	ADS122C04_Reg.reg1.bit.MODE = param->opMode;
	ADS122C04_Reg.reg1.bit.CMBIT = param->convMode;
	ADS122C04_Reg.reg1.bit.VREF = param->selectVref;
	ADS122C04_Reg.reg1.bit.TS = param->tempSensorEn;

	ADS122C04_Reg.reg2.bit.DCNT = param->dataCounterEn;
	ADS122C04_Reg.reg2.bit.CRC = param->dataCRCen;
	ADS122C04_Reg.reg2.bit.BCS = param->burnOutEn;
	ADS122C04_Reg.reg2.bit.IDAC = param->idacCurrent;

	ADS122C04_Reg.reg3.bit.I1MUX = param->routeIDAC1;
	ADS122C04_Reg.reg3.bit.I2MUX = param->routeIDAC2;

	boolean ret_val = true; // Flag to show if the four writeRegs were successful
	// (If any one writeReg returns false, ret_val will be false)
	ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
	ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
	ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
	ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all);

	return(ret_val);
}

boolean SFE_QWIIC_PT100::reset(void)
{
	return(ADS122C04_sendCommand(ADS122C04_RESET_CMD));
}

boolean SFE_QWIIC_PT100::start(void)
{
	return(ADS122C04_sendCommand(ADS122C04_START_CMD));
}

boolean SFE_QWIIC_PT100::powerdown(void)
{
	return(ADS122C04_sendCommand(ADS122C04_POWERDOWN_CMD));
}

boolean SFE_QWIIC_PT100::ADS122C04_writeReg(uint8_t reg, uint8_t writeValue)
{
	uint8_t command = 0;
	command = ADS122C04_WRITE_CMD(reg);
	return(ADS122C04_sendCommandWithValue(command, writeValue));
}

boolean SFE_QWIIC_PT100::ADS122C04_readReg(uint8_t reg, uint8_t *readValue)
{
	uint8_t command = 0;
	command = ADS122C04_READ_CMD(reg);

	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
	_i2cPort->write(command);

	if (_i2cPort->endTransmission(false) != 0)    //Do not release bus
	{
		debugPrintln("ADS122C04_readReg: sensor did not ACK");
    return (false); //Sensor did not ACK
	}

	_i2cPort->requestFrom((uint8_t)_deviceAaddress, (uint8_t)1); // Request one byte
  if (_i2cPort->available() >= 1)
  {
    readValue = _i2cPort->read();
		return(true);
	}

	debugPrintln("ADS122C04_readReg: requestFrom returned no data");
	return(false);
}

boolean SFE_QWIIC_PT100::ADS122C04_sendCommand(uint8_t command)
{
	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
	_i2cPort->write(command);
	return (_i2cPort->endTransmission() == 0);
}

boolean SFE_QWIIC_PT100::ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value)
{
	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
	_i2cPort->write(command);
	_i2cPort->write(value);
	return (_i2cPort->endTransmission() == 0);
}

// Read the conversion result with count byte.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
boolean SFE_QWIIC_PT100::ADS122C04_getConversionDataWithCount(uint32_t *conversionData, uint8_t *count)
{
	uint8_t RXByte[4] = {0};

	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
	_i2cPort->write(ADS122C04_RDATA_CMD);

	if (_i2cPort->endTransmission(false) != 0)    //Do not release bus
	{
		debugPrintln("ADS122C04_getConversionDataWithCount: sensor did not ACK");
    return(false); //Sensor did not ACK
	}

	_i2cPort->requestFrom((uint8_t)_deviceAaddress, (uint8_t)4); // Request four bytes

  if (_i2cPort->available() >= 4)
  {
		RXByte[0] = _i2cPort->read(); // Count
		RXByte[1] = _i2cPort->read(); // MSB
		RXByte[2] = _i2cPort->read();
		RXByte[3] = _i2cPort->read(); // LSB
	}
	else
	{
		debugPrintln("ADS122C04_getConversionDataWithCount: requestFrom failed");
		return(false);
	}

	*count = RXByte[0];
	*conversionData = ((uint32_t)RXByte[3]) | ((uint32_t)RXByte[2]<<8) | ((uint32_t)RXByte[1]<<16);
	return(true);
}

// Read the conversion result.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
boolean SFE_QWIIC_PT100::ADS122C04_getConversionData(uint32_t *conversionData)
{
	uint8_t RXByte[3] = {0};

	_i2cPort->beginTransmission((uint8_t)_deviceAddress);
	_i2cPort->write(ADS122C04_RDATA_CMD);

	if (_i2cPort->endTransmission(false) != 0)    //Do not release bus
	{
		debugPrintln("ADS122C04_getConversionData: sensor did not ACK");
    return(false); //Sensor did not ACK
	}

	_i2cPort->requestFrom((uint8_t)_deviceAaddress, (uint8_t)3); // Request three bytes

  if (_i2cPort->available() >= 3)
  {
    RXByte[0] = _i2cPort->read(); // MSB
		RXByte[1] = _i2cPort->read();
		RXByte[2] = _i2cPort->read(); // LSB
	}
	else
	{
		debugPrintln("ADS122C04_getConversionData: requestFrom failed");
		return(false);
	}

	*conversionData = ((uint32_t)RXByte[2]) | ((uint32_t)RXByte[1]<<8) | ((uint32_t)RXByte[0]<<16);
	return(true);
}
