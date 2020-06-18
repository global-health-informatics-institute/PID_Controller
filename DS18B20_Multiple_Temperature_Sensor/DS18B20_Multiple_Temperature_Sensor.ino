#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged TO GPIO 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Number of temperature devices found
int numberOfDevices;

// We'll use this variable to store a found device address
DeviceAddress tempDeviceAddress; 

void setup(){
  // start serial port
  Serial.begin(9600);
  
  // Start up the library
  sensors.begin();
  
}

void loop()
{ 
   Serial.print(",");
   sensors.requestTemperatures(); // Send the command to get temperatures
   if(sensors.getAddress(tempDeviceAddress, 0)){
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      Serial.print(tempC);
      Serial.print(",");
   }
   if(sensors.getAddress(tempDeviceAddress, 1)){
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      Serial.print(tempC);
      Serial.print(",");
   }
   if(sensors.getAddress(tempDeviceAddress, 2)){
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      Serial.print(tempC);
      Serial.print(",");
   }
   if(sensors.getAddress(tempDeviceAddress, 3)){
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      Serial.print(tempC);
      Serial.print(",");
   }
   if(sensors.getAddress(tempDeviceAddress, 4)){
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      Serial.println(tempC);
   }
}
