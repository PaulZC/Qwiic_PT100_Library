/*
  Using the Qwiic PT100
  By: Paul Clark (PaulZC)
  Date: May 5th, 2020

  When the ADS122C04 is initialised by .begin, it is configured for 4-wire mode.
  If you want to manually configure the chip, you can. This example demonstrates how.

  The IDAC current source is disabled, the gain is set to 1 and the internal 2.048V reference is selected.
  The conversion is started manually using .start.
  DRDY is checked manually using .checkDataReady.
  The ADC result is read using .readADC.

  readADC returns a uint32_t. The ADC data is returned in the least-significant 24-bits.

  Hardware Connections:
  Plug a Qwiic cable into the PT100 and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>

#include <Qwiic_PT100_Library.h>

SFE_QWIIC_PT100 mySensor;

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println(F("Qwiic PT100 Example"));

  Wire.begin();

  //mySensor.enableDebugging(); //Uncomment this line to enable debug messages on Serial

  if (mySensor.begin() == false) //Connect to the PT100 using the defaults: Address 0x45 and the Wire port
  {
    Serial.println(F("Qwiic PT100 not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // The ADS122C04 will now be configured for 4-wire mode.
  // We can override that using these commands:

  mySensor.setInputMultiplexer(ADS122C04_MUX_AIN0_AIN1); // Route AIN0 and AIN1 to AINP and AINN
  mySensor.setGain(ADS122C04_GAIN_1); // Set the gain to 1
  mySensor.enablePGA(ADS122C04_PGA_DISABLED); // Disable the Programmable Gain Amplifier
  mySensor.setDataRate(ADS122C04_DATA_RATE_20SPS); // Set the data rate (samples per second) to 20
  mySensor.setOperatingMode(ADS122C04_OP_MODE_NORMAL); // Disable turbo mode
  mySensor.setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT); // Use single shot mode
  mySensor.setVoltageReference(ADS122C04_VREF_INTERNAL); // Use the internal 2.048V reference
  mySensor.enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF); // Disable the temperature sensor
  mySensor.setDataCounter(ADS122C04_DCNT_DISABLE); // Disable the data counter (Note: the library does not currently support the data count)
  mySensor.setDataIntegrityCheck(ADS122C04_CRC_DISABLED); // Disable CRC checking (Note: the library does not currently support data integrity checking)
  mySensor.setBurnOutCurrent(ADS122C04_BURN_OUT_CURRENT_OFF); // Disable the burn-out current
  mySensor.setIDACcurrent(ADS122C04_IDAC_CURRENT_OFF); // Disable the IDAC current
  mySensor.setIDAC1mux(ADS122C04_IDAC1_DISABLED); // Disable IDAC1
  mySensor.setIDAC2mux(ADS122C04_IDAC2_DISABLED); // Disable IDAC2

}

void loop()
{
  mySensor.start(); // Start the conversion

  unsigned long start_time = millis(); // Record the start time so we can timeout
  boolean drdy = false; // DRDY (1 == new data is ready)

  // Wait for DRDY to go valid
  // (You could read the DRDY pin instead, especially if you are using continuous conversion mode.)
  while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT)))
  {
    delay(5); // Don't pound the I2C bus too hard
    drdy = mySensor.checkDataReady();
  }

  // Check if we timed out
  if (drdy == false)
  {
    Serial.println(F("checkDataReady timed out"));
    return;
  }

  // Read the rawADC data
  uint32_t raw_ADC_data = mySensor.readADC();

  // Print the raw ADC data
  Serial.print(F("The raw ADC data is 0x"));
  Serial.println(raw_ADC_data, HEX);

  delay(250); //Don't pound the I2C bus too hard
}
