#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <Wire.h> 

// Data wire for 5 sensors is plugged TO GPIO 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// We'll use this variable to store a found device address
DeviceAddress tempDeviceAddress; 

//Inputs and outputs
int firing_pin = 32;
int zero_cross = 33;
const int ADDR = 0x40;
const int MeasureTemp = 0xE3;
int X0,X1,temp;
double X,X_out;

LiquidCrystal_I2C lcd(0x27,16,2);  //sometimes the adress is not 0x27. Change to 0x3f if it dosn't work.
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//Variables
int last_CH1_state = 0;
bool zero_cross_detected = false;
const int maximum_firing_delay = 9000;
// Max firing delay set to 9ms based on AC frequency of 50Hz
unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;
int temp_read_Delay = 500;
float real_temperature = 0;
int setpoint = 55;
int print_firing_delay;
//PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
//PID constants
int kp =1200;   int ki= 0.5;   int kd = 20000;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

//Zero Crossing Interrupt Function
void IRAM_ATTR zero_crossing()
{
  //If the last state was 0, then we have a state change...
  if(last_CH1_state == 0) 
    zero_cross_detected = true; //We have detected a state change! We need both falling and rising edges
  //If pin 8 is LOW and the last state was HIGH then we have a state change      
  else if(last_CH1_state == 1){ 
    zero_cross_detected = true;    //We have detected a state change!  We need both falling and rising edges.
    last_CH1_state = 0;            //Store the current state into the last state for the next loop
  }   
}

void setup() 
{
  // Start up the library
  sensors.begin();
  //Define the pinsmaximum_firing_delay - PID_value
  pinMode (firing_pin,OUTPUT); 
  pinMode (zero_cross,INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, CHANGE);
  lcd.begin(); //Begin the LCD communication through I2C
  lcd.backlight();  //Turn on backlight for LCD
  //inititalize the I2C the sensor and bing it
  Serial.begin(9600);
  I2Cone.begin(21,22,50000); // SDA GPIO33, SCL GPIO32, 50kHz frequency
  I2Ctwo.begin(2,4,50000); // SDA GPIO19, SCL GPIO18, 50kHz frequency
}


void loop() 
{   
  currentMillis = millis();           //Save the value of time before the loop
  // We create this if so we will read the temperature and change values each "temp_read_Delay"
  if(currentMillis - previousMillis >= temp_read_Delay){
    previousMillis += temp_read_Delay;              //Increase the previous time for next loop
    real_temperature = (GetTemp(&I2Cone));  //get the real temperature in Celsius degrees
    Serial.print(",");
    Serial.print((maximum_firing_delay - PID_value)/100.0);
    Serial.print("," + String(real_temperature) + "," + String(GetTemp(&I2Ctwo)));
    Serial.println(GetTemp2());
    PID_error = setpoint - real_temperature;        //Calculate the pid ERROR
    if(PID_error > 30)                              //integral constant will only affect errors below 30ºC             
      PID_i = 0;
    PID_p = kp * PID_error;                         //Calculate the P value
    PID_i = PID_i + (ki * PID_error);               //Calculate the I value
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = millis();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000;   
    PID_d = kd*((PID_error - previous_error)/elapsedTime); //Calculate the D value
    PID_value = PID_p + PID_i + PID_d; //Calculate total PID value
    //We define firing delay range between 0 and 7400. Read above why 7400!!!!!!!
    if(PID_value < 0)      
      PID_value = 0;       
    if(PID_value > maximum_firing_delay)      
      PID_value = maximum_firing_delay;    
    //Print the values on the LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set: ");
    lcd.setCursor(5,0);
    lcd.print(setpoint);
    lcd.setCursor(0,1);
    lcd.print("Real temp: ");
    lcd.setCursor(11,1);
    lcd.print(real_temperature);
    previous_error = PID_error; //Remember to store the previous error.
  }

  //If the zero cross interruption was detected we create the 100us firing pulse  
  if (zero_cross_detected){
      delayMicroseconds(maximum_firing_delay - PID_value); //This delay controls the power
      digitalWrite(firing_pin,HIGH);
      delayMicroseconds(100);
      digitalWrite(firing_pin,LOW);
      zero_cross_detected = false;
  } 
}
//End of void loop

//Extracts temperature from the si7021 Sensor
double GetTemp(TwoWire *Sensor)
{
  Sensor->beginTransmission(ADDR);
  Sensor->write(MeasureTemp);
  Sensor->endTransmission();
  Sensor->requestFrom(ADDR,2);
  if(Sensor->available()<=2);{
    X0 = Sensor->read();
    X1 = Sensor->read();
    X0 = X0<<8;
    X_out = X0+X1;
  }
  /**Calculate and display temperature**/
  X=(175.72*X_out)/65536;
  X=X-46.85;
  return X;
}

//Extracts temperature from the DS18B20 Sensor

double GetTemp2()
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
      Serial.print(tempC);
      Serial.print(",");
   }
  
}
