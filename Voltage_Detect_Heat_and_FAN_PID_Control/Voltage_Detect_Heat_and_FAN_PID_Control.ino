#include <MCP3XXX.h>
//#include <LiquidCrystal_I2C.h>
#include <Wire.h> 

extern TwoWire Wire1; //// THIS IS NEW
MCP3002 adc;

/*
 * The logic here is as follows ...
 * The 5 sensor readings tell us that the temperature at the outside of the oven is always warmer then the temp at the center.
 * We believe this is true because the fan is pushing all the warm air to the outside when running at full speed.
 * If we slow down the fan we believe that we will even out the temperature.
 * The PID will adjust the firing delay on the fan so that the error between the two temps will be zero.
 * 
 * We know that ft we keep power on the input to the MOC3020 then it runs well.
 * We will create are range of power control that will be divided into 15 power levels.
 * We will turn the fan ON and OFF with varying duty cycles between 44% and 100% every half second.
 * Since we want to turn the power on and off at ZERO CROSSING, so as to not risk damaging the triac.
 * We will have the following combination
 * Full speed  = 25 on 0 off, then proceed down ... 24/1, 23/2, ... , 12/13, 11/14 << Here we are at about 44% power (11/25 = 44)
 */

//Inputs and outputs
gpio_num_t ELEMENT_firing_pin = GPIO_NUM_33; // THIS IS FOR THE ELEMENT TRIAC AS PER PCB LAYOUT
gpio_num_t FAN_firing_pin = GPIO_NUM_32; // THIS IS FOR THE FAN TRIAC AS PER PCB LAYOUT
gpio_num_t zero_cross = GPIO_NUM_35; // THIS IS FOR THE ZERO CROSSING DETECTION AS PER PCB LAYOUT  *** CHANGED TO GPIO25 to match PCB ***

const int ADDR = 0x40;
const int MeasureTemp = 0xE3;
int X0,X1,temp;
double X,X_out;

//LiquidCrystal_I2C lcd(0x27,20,4);  //sometimes the adress is not 0x27. Change to 0x3f if it dosn't work.

//Variables
float tau = 0.02;
int last_CH1_state = 0;
bool zero_cross_detected = false;
const int maximum_firing_delay = 9000;
// Max firing delay set to 9ms based on AC frequency of 50Hz
//unsigned long previousMillis = 0; 
//unsigned long currentMillis = 0;
int temp_read_Delay = 500000;
int setpoint = 115;
int print_firing_delay;
//int PID_dArrayIndex = 0; //we use this to keep track of where we are inserting into the array
//double LastTwentyPID_d[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // An Array for the values
//PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;

//Voltage detection variables
bool Voltage_read = false;
float Voltage = 0;
unsigned long previousMicros = 0; 
unsigned long currentMicros = 0;
int voltage_read_Delay = 4800;
float volts = 0;
float actualvolts = 0;
float volt_greater_than_twenty = 0;
int voltage_read_difference=0;
unsigned long Last_Zero_Crossing_Time = 0;
float LastFiftyVolts[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // An Array for the values
int VoltsArrayIndex = 0;

//PID constants
int kp =1500;   float ki = 0.5;   int kd = 0;
int PID_p = 0;    float PID_i = 0;    int PID_d = 0;

// NEW FAN VARIABLES FOR ON-OFF CONTROL METHOD
int FanSpeed = 25;
int FanCyclesOn, FanCyclesOff;
bool FanOn = true;
bool TempRequestSent = false; // THIS IS NEW
int FAN_firing_delay = 0;  // Initialize this to ZERO and we will adjust for different to see how the speed of the fan varies

//FAN PID variables
float FAN_PID_error = 0;
float FAN_previous_error = 0;
int FAN_PID_value = 0;
int FAN_maximum_firing_delay = 7000; //TESTING THIS VALUE

//FAN PID constants
int FAN_kp = 500;   int FAN_ki = 1;     int FAN_kd = 1000;
int FAN_PID_p = 0;   int FAN_PID_i = 0;  int FAN_PID_d = 0;

//OVEN Temp values
double Outer_Temp, Inner_Temp;  // These hold the values of the two temp sensors we will use for PID control. 
double real_temperature = 0.00;
double Old_Inner_Temp = 0.00;
double Old_Outer_Temp = 0.00;
double Old_Real_Temp = 0.00; 
double prev_real_temperature; //record previous measurement

//Zero Crossing Interrupt Function
void IRAM_ATTR zero_crossing()
{
 delayMicroseconds(10);
 if (gpio_get_level(zero_cross)){ 
  zero_cross_detected = true; 
  Last_Zero_Crossing_Time = micros(); 
 }  
}

double GetTemp(int SDA_Pin, int SLC_pin) {
  Wire.begin(SDA_Pin,SLC_pin,10000);
  Wire.beginTransmission(ADDR);
  Wire.write(MeasureTemp);
  Wire.endTransmission();
  Wire.requestFrom(ADDR,2);
  if(Wire.available()<=2);{
    X0 = Wire.read();
    X1 = Wire.read();
    X0 = X0<<8;
    X_out = X0+X1;
  }
  /**Calculate temperature**/
  X=(175.72*X_out)/65536;
  X=X-46.85;
  return X;
}

//Voltagae averaging
double VoltsArrayAverage() {
  double S=0;
  int Voltge_Values = 0;
  for (int i=0; i<50; i++) {
    if (LastFiftyVolts[i] != 0 ){
      S = S+LastFiftyVolts[i];
      Voltge_Values++;
    }   
  }
  if (Voltge_Values > 0)
    return S/Voltge_Values;
  else
   return 0;
}

////This Is New
/*double PID_dArrayAverage() {
  double S=0;
  int Values = 0;
  for (int i=0; i<20; i++) {
    if (LastTwentyPID_d[i] != 0 ){
      S = S+LastTwentyPID_d[i];
      Values++;
    }   
  }
  if (Values > 0)
    return S/Values;
  else
   return 0;
}*/

void setup() 
{
  Serial.begin(115200);
//  lcd.begin(); //Begin the LCD communication through I2C
//  lcd.backlight();  //Turn on backlight for LCD

  //adc.begin(SS, MOSI, MISO, SCK);// Or use custom pins to use a software SPI interface.
  adc.begin(27,13,12,14);// Use the defined pins for SPI hardware interface.
  pinMode (FAN_firing_pin, OUTPUT);
  pinMode (ELEMENT_firing_pin, OUTPUT);
  pinMode (zero_cross, INPUT); 
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, RISING);  //
  /*Wire.begin(18, 19, 50000);  //Inner sensor
  Wire1.begin(16, 17, 50000);  //Outer sensor
  Wire2.begin(21, 22, 50000);  //Element_PID sensor*/
  digitalWrite(FAN_firing_pin, HIGH);
// digitalWrite(ELEMENT_firing_pin,HIGH);


}

void loop() 
{   
//  currentMillis = millis();           //Save the value of time before the loop
  currentMicros = micros(); 

  //get voltage reading
   if (((currentMicros - Last_Zero_Crossing_Time) >= voltage_read_Delay) && (!Voltage_read)) {
    Voltage_read = true;
    volt_greater_than_twenty = adc.analogRead(0) / 1024.0*420*0.7071;
    if(volt_greater_than_twenty > 50) {
      LastFiftyVolts[VoltsArrayIndex] = volt_greater_than_twenty; 
      volts = VoltsArrayAverage();
      VoltsArrayIndex = (VoltsArrayIndex + 1) % 50;       
    }
    
   }//end get voltage reading
    
//  // SEND RESUEST TO Si7021 SENSORS 10 milliceconds BEFORE we want to read them
/*  if (((currentMillis - previousMillis) >= (temp_read_Delay -10)) and !TempRequestSent) {
     Wire.beginTransmission(ADDR);
     Wire.write(MeasureTemp);
     Wire.endTransmission();
     Wire1.beginTransmission(ADDR);
     Wire1.write(MeasureTemp);
     Wire1.endTransmission();
     TempRequestSent = true;
  }*/


  // We create this if so we will read the temperature and change values each "temp_read_Delay"
  if(currentMicros - previousMicros >= temp_read_Delay){
    previousMicros += temp_read_Delay;              //Increase the previous time for next loop
   /* Wire.requestFrom(ADDR, 2);
    if (Wire.available() <= 2); {
      X0 = Wire.read();
      X1 = Wire.read();
      X0 = X0 << 8;
      X_out = X0 + X1;
    }
    //Calculate temperature
    X = (175.72 * X_out) / 65536;
    Inner_Temp = X - 46.85;

    Wire1.requestFrom(ADDR, 2);
    if (Wire1.available() <= 2); {
      X0 = Wire1.read();
      X1 = Wire1.read();
      X0 = X0 << 8;
      X_out = X0 + X1;
    }
    //Calculate temperature
    X = (175.72 * X_out) / 65536;
    Outer_Temp = X - 46.85;*/

    //Geting Inner Temperature by checking Difference of  pr
    Inner_Temp = GetTemp(21, 22);//GetTemp(18, 19);  
    
    if(Old_Inner_Temp == 0.00){
      Old_Inner_Temp = Inner_Temp;
    } else{
        if((abs(Inner_Temp - Old_Inner_Temp)) > 5.00)
          Inner_Temp = Old_Inner_Temp;
        else
          Old_Inner_Temp = Inner_Temp;
      }
        
    Outer_Temp = GetTemp(16, 17);
    if(Old_Outer_Temp == 0.00){
      Old_Outer_Temp = Outer_Temp;
    } else{
        if((abs(Outer_Temp - Old_Outer_Temp)) > 5.00)
          Outer_Temp = Old_Outer_Temp;
        else
          Old_Outer_Temp = Outer_Temp;
      }

    real_temperature = GetTemp(18, 19); //GetTemp(21, 22);   //get Element PID Control Temperature : NOW COntrolled by Middle Cell Temperature for testing
    if(Old_Real_Temp == 0.00){
      Old_Real_Temp = real_temperature;
    } else{
        if((abs(real_temperature - Old_Real_Temp)) > 5.00)
          real_temperature = Old_Real_Temp;
        else
          Old_Real_Temp = real_temperature;
      }
    
    //Time Tracking
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = millis();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000000;   
        
    // Element PID Control

    //This Is New to avoid Over Heating and Smoothen Graph 
   /* if (real_temperature < setpoint)
       PID_error = 110 - Outer_Temp;    
    else{
       previous_error = 0;
       PID_error = setpoint - real_temperature;        //Calculate the pid ERROR
    }*/

    PID_error = setpoint - real_temperature;  
    if(PID_error > 30)                              //integral constant will only affect errors below 30ºC             
      PID_i = 0;
    PID_p = kp * PID_error;                         //Calculate the P value
   PID_i = PID_i + 0.5f * ki * elapsedTime * (PID_error + previous_error);              //Calculate the I value

    //THIS NEW //Calculate the D value 
   PID_d = -(2.0f*kd*(real_temperature-prev_real_temperature) + (2.0f*tau - elapsedTime)) / (2.0f * tau + elapsedTime);
    
    //PID_d = kd*((PID_error - previous_error)/elapsedTime); Calculate the D value
    PID_value = PID_p + PID_i + PID_d; //Calculate total PID value
    //We define firing delay range between 0 and 9000.
    if(PID_value < 0)      
      PID_value = 0;       
    if(PID_value > maximum_firing_delay)      
      PID_value = maximum_firing_delay; 
    previous_error = PID_error; //Remember to store the previous error.

    //FAN PID Control
   /* FAN_PID_error = Outer_Temp - Inner_Temp;        //Calculate the pid ERROR as the difference between ths center and edge of ove

    if(FAN_PID_error > 30)                              //integral constant will only affect errors below 30ºC             
      FAN_PID_i = 0;
    FAN_PID_p = FAN_kp * FAN_PID_error;                         //Calculate the P value
    FAN_PID_i = FAN_PID_i + (FAN_ki * FAN_PID_error);               //Calculate the I value 
    FAN_PID_d = FAN_kd*((FAN_PID_error - FAN_previous_error)/elapsedTime); //Calculate the D value
    FAN_PID_value = FAN_PID_p + FAN_PID_i + FAN_PID_d; //Calculate total PID value
    if(FAN_PID_value < 0)      
      FAN_PID_value = 0;       
    if(FAN_PID_value > FAN_maximum_firing_delay)      
      FAN_PID_value = FAN_maximum_firing_delay;    
    FAN_previous_error = FAN_PID_error; //Remember to store the previous error.
    TempRequestSent = false;  // THIS IS REQUIRED
    //end new FAN PID code*/    
    
//  MAP FAN_PID_value to a FanSpeed  
    FanSpeed = 11;//((FAN_maximum_firing_delay - FAN_PID_value) / 500) + 11; // this value will always be between 11 AND 25
 
    //Print the values on the LCD
   /* Wire.begin(21,22,50000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set: ");
    lcd.setCursor(5,0);
    lcd.print(setpoint);
    lcd.setCursor(0,1);
    lcd.print("Real temp: ");
    lcd.setCursor(11,1);
    lcd.print(real_temperature);*/
   TempRequestSent = false;  // THIS IS REQUIRED
   
    // Print the firing delay and the temps of the locations so we can graph them
    Serial.print(", Heat Firing Delay="  + String ((maximum_firing_delay - PID_value)/100.0));
    /*Serial.print(", Heat Control Temp=" + String(real_temperature)); */
    Serial.print(", Center Temp=" + String(Inner_Temp ));
    Serial.print(", Outer Temp=" + String(Outer_Temp )); 
    /*Serial.print(", Fan Firing Delay=" + String(FAN_PID_value)); 
    Serial.print(", Fan Speed =" + String(FanSpeed));     
    Serial.print(", Error=" + String(FAN_PID_error)); */  // THIS IS THE DIFFERENCE IN TEMP BETWEEN THE OUTER AND INNER SENSOR THAT WE ARE TRYING TO REDUCE TO ZERO
    Serial.print(", Set Point =" + String(setpoint));
    Serial.print(", PID_p=" + String(PID_p)); 
    Serial.print(", PID_i=" + String(PID_i)); 
    Serial.print(", PID_d=" + String(PID_d));
    Serial.print(", Voltage=" + String(volts));
    /*Serial.print (", "+ String(PID_dArrayIndex));
   for (int i=0; i<20; i++) {
     Serial.print(", " + String(LastTwentyPID_d[i]));
    }*/
    Serial.println();
  //This is new
  prev_real_temperature = real_temperature;

  }  
  
  //If the zero cross interruption was detected we create the 100us firing pulse  
  if (zero_cross_detected){
    Voltage_read = false;
    zero_cross_detected = false;
    /*if (FanOn == true) {
      if (FanCyclesOn > 0)
        FanCyclesOn -= 1;
      else {
        FanOn = false;
        FanCyclesOff = 25 - FanSpeed;
        digitalWrite(FAN_firing_pin, LOW);          
      }
    }  
    else {
      if (FanCyclesOff > 0)
         FanCyclesOff -= 1;
      else {
         FanOn = true;
         FanCyclesOn = FanSpeed;
         digitalWrite(FAN_firing_pin, HIGH);
      } 
    }*/
    delayMicroseconds(maximum_firing_delay - PID_value); //This delay controls the power
    digitalWrite(ELEMENT_firing_pin,HIGH);
    delayMicroseconds(100);
    digitalWrite(ELEMENT_firing_pin,LOW);
  } 
}
//End of void loop
