/*
 * This sketch reads the temperature with the ds18b20 temperature sensor using the dallas temperature library.
 * 
 */

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// vars
double Input;

void setup() {
  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");
  
  // Start up the library
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement

  sensors.getAddress(tempSensor, 0);
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);
}

void loop() {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  //sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
   
  //Serial.print("Temperature for Device 1 is: ");
  //Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  //Serial.print("  ");

  ///////////////////////////////

  Serial.print("main loop. ");
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    Serial.print("Temp: ");
    Serial.println(Input);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
}
