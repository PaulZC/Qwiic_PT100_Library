/*
  Using the Qwiic PT100
  By: Paul Clark (PaulZC)
  Date: May 5th, 2020

  This example demonstrates how to read the PT100 temperature in Fahrenheit

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

  mySensor.configure234wire(ADS122C04_4WIRE_MODE); // Make sure the PT100 is in 4-wire mode

}

void loop()
{
  // Get the temperature in Centigrade
  float temperature = mySensor.readPT100Fahrenheit();

  // Print the temperature
  Serial.print(F("The temperature is: "));
  Serial.print(temperature);
  Serial.println(F("F"));

  delay(250); //Don't pound too hard on the I2C bus
}
