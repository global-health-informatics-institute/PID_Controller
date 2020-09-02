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
gpio_num_t HINGE_LEFT_Element_Firing_Pin = GPIO_NUM_33; // THIS IS FOR THE HINGE LEFT WARMER TRIAC AS PER PCB LAYOUT
gpio_num_t HINGE_RIGHT_Element_Firing_Pin = GPIO_NUM_32; // THIS IS FOR THE HINGE RIGHT WARMER  TRIAC AS PER PCB LAYOUT
gpio_num_t FRONT_LEFT_Element_Firing_Pin = GPIO_NUM_4; // THIS IS FOR THE FRONTLEFT WARMER TRIAC AS PER PCB LAYOUT
gpio_num_t FRONT_RIGHT_Element_Firing_Pin = GPIO_NUM_25; // THIS IS FOR THE FRONT RIGHT WARMER  TRIAC AS PER PCB LAYOUT
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
bool TempRequestSent = false; // THIS IS NEW
const int maximum_firing_delay = 9000;
// Max firing delay set to 9ms based on AC frequency of 50Hz
//unsigned long previousMillis = 0; 
//unsigned long currentMillis = 0;
int temp_read_Delay = 500000;
int setpoint = 100;
int print_firing_delay;
//int PID_dArrayIndex = 0; //we use this to keep track of where we are inserting into the array
//double LastTwentyPID_d[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // An Array for the values
//PID variables
float PID_error = 0;
float previous_error = 0;
unsigned long elapsedTime, Time, timePrev;
float PID_value = 0;
int left_firing_delay=0;
int transition_state;

//Voltage detection variables
bool Voltage_read = false;
float Voltage = 0;
unsigned long previousMicros = 0; 
unsigned long currentMicros = 0;
int voltage_read_Delay = 4800;
float volts = 0;
float actualvolts = 0;
float volt_reading = 0;
unsigned long Last_Zero_Crossing_Time = 0;
float LastFiftyVolts[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // An Array for the values
int VoltsArrayIndex = 0;

//PID constants
int kp =2500;   float ki = 0;   int kd = 0;
//HINGE LEFT PID constants
int PID_p = 0;    float PID_i = 0;    int PID_d = 0;

//OVEN Temp values
double Outer_Temp, Inner_Temp, F_Right_Temp;  // These hold the values of the two temp sensors we will use for PID control. 
double real_temperature = 0.00;
double Old_Inner_Temp = 0.00;
double Old_Outer_Temp = 0.00;
double Old_Real_Temp = 0.00; 
double Old_F_Right_Temp = 0.00; 
double prev_real_temperature; //record previous measurement
double prev_F_Right_Temp;//record previous measurement
double prev_Outer_Temp; //record previous measurement
double prev_Inner_Temp ; //record previous measurement

//NEW HINGE RIGHT Warmer Contol values
//PID variables
float hinge_right_PID_error = 0;
float hinge_right_previous_error = 0;
float hinge_right_PID_value = 0;
int hinge_right_firing_delay = 0;
int hinge_right_transition_state;

//HINGE RIGHT Warmer PID constants
int hinge_right_PID_p = 0;    float hinge_right_PID_i = 0;    int hinge_right_PID_d = 0;

//NEW FRONT RIGHT Warmer Contol values
//PID variables
float front_right_PID_error = 0;
float front_right_previous_error = 0;
float front_right_PID_value = 0;
int front_right_firing_delay = 0;
int front_right_transition_state;

//FRONT RIGHT Warmer PID constants
int front_right_PID_p = 0;    float front_right_PID_i = 0;    int front_right_PID_d = 0;

//NEW FRONT LEFT Warmer Contol values
//PID variables
float front_left_PID_error = 0;
float front_left_previous_error = 0;
float front_left_PID_value = 0;
int front_left_firing_delay = 0;
int front_left_transition_state;

//FRONT LEFT Warmer PID constants
int front_left_PID_p = 0;    float front_left_PID_i = 0;    int front_left_PID_d = 0;

//Interrupt
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

void setup() 
{
  Serial.begin(115200);
  transition_state = 0;
//  lcd.begin(); //Begin the LCD communication through I2C
//  lcd.backlight();  //Turn on backlight for LCD

  //adc.begin(SS, MOSI, MISO, SCK);// Or use custom pins to use a software SPI interface.
  adc.begin(27,13,12,14);// Use the defined pins for SPI hardware interface.
  pinMode (HINGE_LEFT_Element_Firing_Pin, OUTPUT);
  pinMode (HINGE_RIGHT_Element_Firing_Pin, OUTPUT);
  pinMode (FRONT_LEFT_Element_Firing_Pin, OUTPUT);
  pinMode (FRONT_RIGHT_Element_Firing_Pin, OUTPUT);
  pinMode (zero_cross, INPUT); 
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, RISING);  //
  /*Wire.begin(18, 19, 50000);  //Inner sensor
  Wire1.begin(16, 17, 50000);  //Outer sensor*/
 // digitalWrite(RIGHT_Element_Firing_Pin, HIGH);
// digitalWrite(LEFT_Element_Firing_Pin,HIGH);
}

void loop() 
{   
  /*currentMillis = millis();           //Save the value of time before the loop8*/
  currentMicros = micros(); 

  //get voltage reading
   if (((currentMicros - Last_Zero_Crossing_Time) >= voltage_read_Delay) && (!Voltage_read)) {
    Voltage_read = true;
    volt_reading = adc.analogRead(0) / 1024.0*420*0.7071;
    if(volt_reading > 50) {
      LastFiftyVolts[VoltsArrayIndex] = volt_reading; 
      volts = VoltsArrayAverage();
      VoltsArrayIndex = (VoltsArrayIndex + 1) % 50;       
    }
    
   }//end get voltage reading

  // We create this if so we will read the temperature and change values each "temp_read_Delay"
  if(currentMicros - previousMicros >= temp_read_Delay){
    previousMicros += temp_read_Delay;              //Increase the previous time for next loop

    //We use Real_temp for the Hinge LEFT Warmer
    real_temperature = GetTemp(21, 22) + 4.99; //GetTemp(21, 22);   //get Element PID Control Temperature : NOW COntrolled by Middle Cell Temperature for testing
    if(Old_Real_Temp == 0.00){
      Old_Real_Temp = real_temperature;
    } else{
        if((abs(real_temperature - Old_Real_Temp)) > 5.00)
          real_temperature = Old_Real_Temp;
        else
          Old_Real_Temp = real_temperature;
      }

    //We use Inner_temp for the Hinge RIGHT Warmer
    Inner_Temp = GetTemp(18, 19) + 1.24;//GetTemp(18, 19);  
    if(Old_Inner_Temp == 0.00){
      Old_Inner_Temp = Inner_Temp;
    } else{
        if((abs(Inner_Temp - Old_Inner_Temp)) > 5.00)
          Inner_Temp = Old_Inner_Temp;
        else
          Old_Inner_Temp = Inner_Temp;
      }


    //We use Outer_temp for the FRONT LEFT Warmer
    Outer_Temp = GetTemp(16, 17);
    if(Old_Outer_Temp == 0.00){
      Old_Outer_Temp = Outer_Temp;
    } else{
        if((abs(Outer_Temp - Old_Outer_Temp)) > 5.00)
          Outer_Temp = Old_Outer_Temp;
        else
          Old_Outer_Temp = Outer_Temp;
      }


    //We use Inner_temp for the FRONT RIGHT Warmer.
    F_Right_Temp = GetTemp(15,2) + 4.99; //get Element PID Control Temperature : NOW COntrolled by Middle Cell Temperature for testing
    if(Old_F_Right_Temp == 0.00){
      Old_F_Right_Temp = F_Right_Temp;
    } else{
        if((abs(F_Right_Temp - Old_F_Right_Temp)) > 5.00)
          F_Right_Temp = Old_F_Right_Temp;
        else
          Old_F_Right_Temp = F_Right_Temp;
      }

        
    //Time Tracking
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = micros();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000000;   
        
//    //Preheating State
//    if (real_temperature > 97)
//      transition_state = 1;
//    if (transition_state == 0){
//      PID_error = 125 - Outer_Temp; 
//      kp = 1000; //Transitioning Kp in relation to Setpoint
//    }
//    //Stablizing State
//    if (transition_state == 1){
//      previous_error = 0;
//      PID_error = setpoint - real_temperature;        //Calculate the pid ERROR
//      kp = 1500; //Transitioning Kp in relation to Setpoint
//    }

    //this is for PID calculation for Hinge LEFT Warmer
    PID_error = setpoint - real_temperature;        //Calculate the pid ERROR
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
     // End of Hinge LEFT Warmer

    //NEW.. this is for PID calculation for Hinge Right Warmer
    hinge_right_PID_error = setpoint - Inner_Temp;        //Calculate the pid ERROR
    if(hinge_right_PID_error > 30)                              //integral constant will only affect errors below 30ºC             
      hinge_right_PID_i = 0;
    hinge_right_PID_p = kp * hinge_right_PID_error;                         //Calculate the P value
    hinge_right_PID_i = hinge_right_PID_i + 0.5f * ki * elapsedTime * (hinge_right_PID_error + hinge_right_previous_error); //Calculate the I value

    //THIS NEW //Calculate the D value 
    hinge_right_PID_d = -(2.0f*kd*(Inner_Temp-prev_Inner_Temp) + (2.0f*tau - elapsedTime)) / (2.0f * tau + elapsedTime);
    
    //PID_d = kd*((PID_error - previous_error)/elapsedTime); Calculate the D value
    hinge_right_PID_value = hinge_right_PID_p + hinge_right_PID_i + hinge_right_PID_d; //Calculate total PID value
    //We define firing delay range between 0 and 9000.
    if(hinge_right_PID_value < 0)      
      hinge_right_PID_value = 0;       
    if(hinge_right_PID_value > maximum_firing_delay)      
      hinge_right_PID_value = maximum_firing_delay; 
    hinge_right_previous_error = hinge_right_PID_error; //Remember to store the previous error.  
    // End of Hinge Right Warmer

    //NEW.. this is for PID calculation for Front Right Warmer
    front_right_PID_error = setpoint - F_Right_Temp;        //Calculate the pid ERROR
    if(front_right_PID_error > 30)                              //integral constant will only affect errors below 30ºC             
      front_right_PID_i = 0;
    front_right_PID_p = kp * front_right_PID_error;                         //Calculate the P value
    front_right_PID_i = front_right_PID_i + 0.5f * ki * elapsedTime * (front_right_PID_error + front_right_previous_error); //Calculate the I value

    //THIS NEW //Calculate the D value 
    front_right_PID_d = -(2.0f*kd*(F_Right_Temp - prev_F_Right_Temp) + (2.0f*tau - elapsedTime)) / (2.0f * tau + elapsedTime);
    
    //PID_d = kd*((PID_error - previous_error)/elapsedTime); Calculate the D value
    front_right_PID_value = front_right_PID_p + front_right_PID_i + front_right_PID_d; //Calculate total PID value
    //We define firing delay range between 0 and 9000.
    if(front_right_PID_value < 0)      
      front_right_PID_value = 0;       
    if(front_right_PID_value > maximum_firing_delay)      
      front_right_PID_value = maximum_firing_delay; 
    front_right_previous_error = front_right_PID_error; //Remember to store the previous error.  
    // End of Front Right Warmer

    //NEW.. this is for PID calculation for Front Left Warmer
    front_left_PID_error = setpoint - Outer_Temp;        //Calculate the pid ERROR
    if(front_left_PID_error > 30)                              //integral constant will only affect errors below 30ºC             
      front_left_PID_i = 0;
    front_left_PID_p = kp * front_left_PID_error;                         //Calculate the P value
    front_left_PID_i = front_left_PID_i + 0.5f * ki * elapsedTime * (front_left_PID_error + front_left_previous_error); //Calculate the I value

    //THIS NEW //Calculate the D value 
    front_left_PID_d = -(2.0f*kd*(Outer_Temp - prev_Outer_Temp) + (2.0f*tau - elapsedTime)) / (2.0f * tau + elapsedTime);
    
    //PID_d = kd*((PID_error - previous_error)/elapsedTime); Calculate the D value
    front_left_PID_value = front_left_PID_p + front_left_PID_i + front_left_PID_d; //Calculate total PID value
    //We define firing delay range between 0 and 9000.
    if(front_left_PID_value < 0)      
      front_left_PID_value = 0;       
    if(front_left_PID_value > maximum_firing_delay)      
      front_left_PID_value = maximum_firing_delay; 
    front_left_previous_error = front_left_PID_error; //Remember to store the previous error.  
    // End of Front Right Warmer
    
    
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
    Serial.print(", Heat Firing Delay HL="  + String ((maximum_firing_delay - PID_value)/100.0));
    Serial.print(",Heat Firing Delay HR=" +String((maximum_firing_delay - hinge_right_PID_value)/100.0));
    Serial.print(",Heat Firing Delay FR=" +String((maximum_firing_delay - front_right_PID_value)/100.0));
    Serial.print(",Heat Firing Delay FL=" +String((maximum_firing_delay - front_left_PID_value)/100.0));
    Serial.print(", Heat Control Temp HL=" + String(real_temperature)); 
    Serial.print(", Heat Control Temp HR=" + String(Inner_Temp ));
    Serial.print(", Heat Control Temp FR=" + String(F_Right_Temp ));
    Serial.print(", Heat Control Temp FL=" + String(Outer_Temp )); 
    Serial.print(", Set Point =" + String(setpoint));
    Serial.print(", PID_p=" + String(PID_p)); 
    Serial.print(", PID_i=" + String(PID_i)); 
    Serial.print(", PID_d=" + String(PID_d));
    Serial.print(", Kp =" + String(kp)); 
//    Serial.print(", Tran_State L=" + String(transition_state)); 
//    Serial.print(", Tran_State L=" + String(right_transition_state)); 
    Serial.print(", Voltage=" + String(volts));
    Serial.println();
  //This is new
  prev_real_temperature = real_temperature;

  }  
  
  //If the zero cross interruption was detected we create the 100us firing pulse  
  if (zero_cross_detected){
    Voltage_read = false;
    zero_cross_detected = false;
    
  //HINGE LEFT WARMER CONTROL
  if((currentMicros - Last_Zero_Crossing_Time) > left_firing_delay)
    digitalWrite(HINGE_LEFT_Element_Firing_Pin,HIGH);
  if((currentMicros - Last_Zero_Crossing_Time) > (left_firing_delay + 100,000))
    digitalWrite(HINGE_LEFT_Element_Firing_Pin,LOW); 

  //HINGE RIGHT WARMER CONTROL
  if((currentMicros - Last_Zero_Crossing_Time) > hinge_right_firing_delay)
    digitalWrite(HINGE_RIGHT_Element_Firing_Pin,HIGH);
  if((currentMicros - Last_Zero_Crossing_Time) > (hinge_right_firing_delay + 100,000))
    digitalWrite(HINGE_RIGHT_Element_Firing_Pin,LOW); 

  //FRONT LEFT WARMER CONTROL
  if((currentMicros - Last_Zero_Crossing_Time) > front_left_firing_delay)
    digitalWrite(FRONT_LEFT_Element_Firing_Pin,HIGH);
  if((currentMicros - Last_Zero_Crossing_Time) > (front_left_firing_delay + 100,000))
    digitalWrite(FRONT_LEFT_Element_Firing_Pin,LOW);  

  //FRONT RIGHT WARMER CONTROL
  if((currentMicros - Last_Zero_Crossing_Time) > front_right_firing_delay)
    digitalWrite(FRONT_RIGHT_Element_Firing_Pin,HIGH);
  if((currentMicros - Last_Zero_Crossing_Time) > (front_right_firing_delay + 100,000))
    digitalWrite(FRONT_RIGHT_Element_Firing_Pin,LOW);  
  } 
}
//End of void loop
