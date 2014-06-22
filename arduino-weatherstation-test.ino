// This is just some (dirty) test case that outputs the measurements of multiple
// bs18b20's as well as a connected bmp180.
//
// Connect the bmp180 to your Arduino UNO as follows:
//  - SCL to analog 5
//  - SDA to analog 4
//  - VDD to 3.3v DC
//  - Ground to common ground
//
// Connect the bs18b20's as shown here: http://bildr.org/2011/07/ds18b20-arduino/
// and http://www.strangeparty.com/2010/12/13/arduino-1-wire-temperature-sensors/
//
// OneWire and DallasTemperature libraries from
//   http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library
// Code based on examples from above and at
//   http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html
// See also http://www.arduino.cc/playground/Learning/OneWire
 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
 
// Data wire is plugged into pin 2 on the Arduino (can be any digital I/O pin)
#define ONE_WIRE_BUS 2
 
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 
int numberOfSensors;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
 
void setup(void) {
  // start serial port
  Serial.begin(9600);
  Serial.println("Temperature reading demo using multiple sensors");
 
  initializeOneWireSensors();
  initializeBmp180Sensor();
}

void initializeOneWireSensors() {
  // Start up the library
  sensors.begin();
 
  Serial.print("Sleeping a bit to let the sensors startup, serial port can lock up otherwise on linux");
  delay(5000);  //important on linux as serial port can lock up otherwise
  numberOfSensors = discoverOneWireDevices();
}

void initializeBmp180Sensor() {
  if (!bmp.begin()) {
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  displaySensorDetails();
}
 
void loop(void) {
  printOneWireTemperaturesToSerial();
  printBmpTempereaturesToSerial();
  delay(1000);
}
 
void printOneWireTemperaturesToSerial(void) {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
 
  // Read each of our sensors and print the value
  for(int i=0; i < numberOfSensors; i++) {
    float temperature = sensors.getTempCByIndex(i);
    Serial.print("{\"sensor\": \"oneWire");
    Serial.print(i);
    Serial.print("\", \"temperature\": ");
    Serial.print(temperature);
    Serial.print ("}\n");
  }
}

void printBmpTempereaturesToSerial()
{
    /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    float temperature;
    bmp.getTemperature(&temperature);
    
    /* Display atmospheric pressue in hPa */
    Serial.print("{\"sensor\": \"bmp180\", \"temperature\": ");
    Serial.print(temperature);
    Serial.print(", \"pressure\": ");
    Serial.print(event.pressure);
    Serial.println("}\n");
  }
  else
  {
    Serial.println("{sensor: \"bmp180\", error: \"bmp180 sensor error\"}\n");
  }
}
 
// Based on http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html
int discoverOneWireDevices(void) {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  int count = 0;
 
  Serial.println("Looking for 1-Wire devices...");
  while(oneWire.search(addr)) {
    Serial.print("Found \'1-Wire\' device with address: ");
    for( i = 0; i < 8; i++) {
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');
      }
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return 0;
    }
    Serial.println();
    count++;
  }
  Serial.println("That's it.");
  oneWire.reset_search();
  return count;
}

/**
 * Displays some basic information on this sensor from the unified
 * sensor API sensor_t type (see Adafruit_Sensor for more information) 
 */
void displaySensorDetails(void) {
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Bmp Sensor:   "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
}
