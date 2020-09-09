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
gpio_num_t FRONT_LEFT_Element_Firing_Pin = GPIO_NUM_26; // THIS IS FOR THE FRONT LEFT WARMER TRIAC AS PER PCB LAYOUT
gpio_num_t FRONT_RIGHT_Element_Firing_Pin = GPIO_NUM_15; // THIS IS FOR THE FRONT RIGHT WARMER  TRIAC AS PER PCB LAYOUT
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
bool LFPS = true; //used to control when to send the firing pulse
bool LHPS = true; //used to control when to send the firing pulse
bool RFPS = true; //used to control when to send the firing pulse
bool RHPS = true; //used to control when to send the firing pulse

const int maximum_firing_delay = 9500;
// Max firing delay set to 9ms based on AC frequency of 50Hz
//unsigned long previousMillis = 0; 
//unsigned long currentMillis = 0;
int temp_read_Delay = 500000;
int setpoint = 20;
//int PID_dArrayIndex = 0; //we use this to keep track of where we are inserting into the array
//double LastTwentyPID_d[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // An Array for the values
unsigned long elapsedTime, Time, timePrev;

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

//OVEN Temp values
double Front_Left_Temp, Hinge_Right_Temp, Front_Right_Temp;  // These hold the values of the two temp sensors we will use for PID control. 
double Hinge_Left_Temp = 0.00;
double Old_Hinge_Right_Temp = 0.00;
double Old_Front_Left_Temp = 0.00;
double Old_Real_Temp = 0.00; 
double Old_Front_Right_Temp = 0.00; 
double prev_Hinge_Left_Temp; //record previous measurement
double prev_Front_Right_Temp;//record previous measurement
double prev_Front_Left_Temp; //record previous measurement
double prev_Hinge_Right_Temp ; //record previous measurement

//PID constants
int kp =1000;   float ki = 0;   int kd = 0;

float Prev_PID_error; //used to store PID error for use in next calculation.

//NEW HINGE LEFT Warmer Contol values
//PID variables
//float hinge_left_PID_error = 0;
float hinge_left_previous_error = 0;
float hinge_left_PID_value = 0;
int hinge_left_firing_delay=0;
int hinge_left_transition_state;

//HINGE LEFT PID constants
int hinge_left_PID_p = 0;    float hinge_left_PID_i = 0;    int hinge_left_PID_d = 0;

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
//float front_right_PID_error = 0;
float front_right_previous_error = 0;
float front_right_PID_value = 0;
int front_right_firing_delay = 0;
int front_right_transition_state;

//FRONT RIGHT Warmer PID constants
int front_right_PID_p = 0;    float front_right_PID_i = 0;    int front_right_PID_d = 0;

//NEW FRONT LEFT Warmer Contol values
//PID variables
//float front_left_PID_error = 0;
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
  LFPS = false;
  LHPS = false; 
  RFPS = false; 
  RHPS = false;
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

//PID value calculation
float GetPidValue(int PID_p, float PID_i, int PID_d, float Prev_error, double CurrentTemp, double prevTemp) {
    float PID_error = 0; float PID_value = 0;

    PID_error = setpoint - CurrentTemp; //Calculate the pid ERROR
    if(PID_error > 30)                  //integral constant will only affect errors below 30ºC    
      PID_i = 0;
    PID_p = kp * PID_error;             //Calculate the P value
    PID_i = PID_i + 0.5f * ki * elapsedTime * (PID_error + Prev_error); //Calculate the I value
    PID_d = -(2.0f*kd*(CurrentTemp-prevTemp) + (2.0f*tau - elapsedTime)) / (2.0f * tau + elapsedTime);  //Calculate the D value 
    PID_value = PID_p + PID_i + PID_d; //Calculate total PID value
    
    //We define firing delay range between 0 and 9000.
    if(PID_value < 0)      
      PID_value = 0;       
    if(PID_value > maximum_firing_delay)      
      PID_value = maximum_firing_delay; 
    Prev_error = PID_error; //Remember to store the previous error.
    Prev_PID_error = Prev_error;

    return PID_value;    
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
//  lcd.begin(); //Begin the LCD communication through I2C
//  lcd.backlight();  //Turn on backlight forHinge_Left_Temp LCD

  //adc.begin(SS, MOSI, MISO, SCK);// Or use custom pins to use a software SPI interface.
  adc.begin(27,13,12,14);// Use the defined pins for SPI hardware interface.
  pinMode (HINGE_LEFT_Element_Firing_Pin, OUTPUT);
  pinMode (HINGE_RIGHT_Element_Firing_Pin, OUTPUT);
  pinMode (FRONT_LEFT_Element_Firing_Pin, OUTPUT);
  pinMode (FRONT_RIGHT_Element_Firing_Pin, OUTPUT);
  pinMode (zero_cross, INPUT); 
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, RISING);
}

void loop() 
{   
  /*currentMillis = millis();           //Save the value of time before the loop8*/
  currentMicros = micros(); 

  //get voltage reading
   if (((micros() - Last_Zero_Crossing_Time) >= voltage_read_Delay) && (!Voltage_read)) {
    Voltage_read = true;
    volt_reading = adc.analogRead(0) / 1024.0*420*0.7071;
    if(volt_reading > 50) {
      LastFiftyVolts[VoltsArrayIndex] = volt_reading; 
      volts = VoltsArrayAverage();
      VoltsArrayIndex = (VoltsArrayIndex + 1) % 50;       
    }
    
   }//end get voltage reading

  // We create this if so we will read the temperature and change values each "temp_read_Delay"
  if(micros() - previousMicros >= temp_read_Delay){
    previousMicros += temp_read_Delay;              //Increase the previous time for next loop

    //We use Real_temp for the Hinge LEFT Warmer
    Hinge_Left_Temp = GetTemp(18, 19) + 1.24; //GetTemp(21, 22);   //get Element PID Control Temperature : NOW COntrolled by Middle Cell Temperature for testing
    if(Old_Real_Temp == 0.00){
      Old_Real_Temp = Hinge_Left_Temp;
    } else{
        if((abs(Hinge_Left_Temp - Old_Real_Temp)) > 5.00)
          Hinge_Left_Temp = Old_Real_Temp;
        else
          Old_Real_Temp = Hinge_Left_Temp;
      }

    //We use Hinge_Right_Temp for the Hinge RIGHT Warmer
    Hinge_Right_Temp = GetTemp(16, 17);//GetTemp(18, 19);  
    if(Old_Hinge_Right_Temp == 0.00){
      Old_Hinge_Right_Temp = Hinge_Right_Temp;
    } else{
        if((abs(Hinge_Right_Temp - Old_Hinge_Right_Temp)) > 5.00)
          Hinge_Right_Temp = Old_Hinge_Right_Temp;
        else
          Old_Hinge_Right_Temp = Hinge_Right_Temp;
      }

    //We use Front_Left_Temp for the FRONT LEFT Warmer
    Front_Left_Temp = GetTemp(21, 22)+ 4.99;
    if(Old_Front_Left_Temp == 0.00){
      Old_Front_Left_Temp = Front_Left_Temp;
    } else{
        if((abs(Front_Left_Temp - Old_Front_Left_Temp)) > 5.00)
          Front_Left_Temp = Old_Front_Left_Temp;
        else
          Old_Front_Left_Temp = Front_Left_Temp;
      }

    //We use the 4th sensor for the FRONT RIGHT Warmer.
    Front_Right_Temp = GetTemp(4,25);//get Element PID Control Temperature : NOW COntrolled by Middle Cell Temperature for testing
    if(Old_Front_Right_Temp == 0.00){
      Old_Front_Right_Temp = Front_Right_Temp;
    } else{
        if((abs(Front_Right_Temp - Old_Front_Right_Temp)) > 5.00)
          Front_Right_Temp = Old_Front_Right_Temp;
        else
          Old_Front_Right_Temp = Front_Right_Temp;
      }
      
    //Time Tracking
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = micros();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000000;   

    //this is for PID calculation for Hinge LEFT Warmer
    hinge_left_PID_value = GetPidValue(hinge_left_PID_p, hinge_left_PID_i, hinge_left_PID_d, hinge_left_previous_error, Hinge_Left_Temp, prev_Hinge_Left_Temp);
    hinge_left_previous_error = Prev_PID_error;
     // End of Hinge LEFT Warmer

    //NEW.. this is for PID calculation for Hinge Right Warmer             
    hinge_right_PID_value = GetPidValue(hinge_right_PID_p, hinge_right_PID_i, hinge_right_PID_d, hinge_right_previous_error, Hinge_Right_Temp, prev_Hinge_Right_Temp);//Calculate total PID value
    hinge_right_previous_error = Prev_PID_error;          //integral constant will only affect errors below 30ºC
    // End of Hinge Right Warmer

    //NEW.. this is for PID calculation for Front Right Warmer
    front_right_PID_value = GetPidValue(front_right_PID_p, front_right_PID_i, front_right_PID_d, front_right_previous_error, Front_Right_Temp, prev_Front_Right_Temp);
    front_right_previous_error = Prev_PID_error;
    // End of Front Right Warmer

    //NEW.. this is for PID calculation for Front Left Warmer    
     front_left_PID_value = GetPidValue(front_left_PID_p, front_left_PID_i, front_left_PID_d, front_left_previous_error, Front_Left_Temp, prev_Front_Left_Temp);
     front_left_previous_error = Prev_PID_error;
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
    lcd.print(Hinge_Left_Temp);*/
    TempRequestSent = false;  // THIS IS REQUIRED
   
    // Print the firing delay and the temps of the locations so we can graph them
    Serial.print(",Firing Delay HL="  + String ((maximum_firing_delay - hinge_left_PID_value)/100.0));
    Serial.print(",Firing Delay HR=" +String((maximum_firing_delay - hinge_right_PID_value)/100.0));
    Serial.print(",Firing Delay FR=" +String((maximum_firing_delay - front_right_PID_value)/100.0));
    Serial.print(",Firing Delay FL=" +String((maximum_firing_delay - front_left_PID_value)/100.0));
    Serial.print(",Temp HL=" + String(Hinge_Left_Temp)); 
    Serial.print(",Temp HR=" + String(Hinge_Right_Temp ));
    Serial.print(",Temp FR=" + String(Front_Right_Temp ));
    Serial.print(",Temp FL=" + String(Front_Left_Temp )); 
    Serial.print(", Set Point =" + String(setpoint));
//    Serial.print(", PID_p=" + String(PID_p)); 
//    Serial.print(", PID_i=" + String(PID_i)); 
//    Serial.print(", PID_d=" + String(PID_d));
//    Serial.print(", Kp =" + String(kp)); 
//    Serial.print(", Tran_State L=" + String(transition_state)); 
//    Serial.print(", Tran_State L=" + String(right_transition_state)); 
    Serial.print(", Voltage=" + String(volts));
    Serial.println();
  //This is new
    prev_Hinge_Left_Temp = Hinge_Left_Temp;
  }  
  
  //If the zero cross interruption was detected we create the 100us firing pulse  
    if (zero_cross_detected){
      Voltage_read = false;
      zero_cross_detected = false;   
    } 
    hinge_left_firing_delay = maximum_firing_delay - hinge_left_PID_value;
    hinge_right_firing_delay = maximum_firing_delay - hinge_right_PID_value;
    front_left_firing_delay = maximum_firing_delay - front_left_PID_value;
    front_right_firing_delay = maximum_firing_delay - front_right_PID_value;

// ////    HINGE LEFT WARMER CONTROL
//   if (!LHPS) {
//    if (((currentMicros - Last_Zero_Crossing_Time) > hinge_left_firing_delay) && (!gpio_get_level(HINGE_LEFT_Element_Firing_Pin)))
//      digitalWrite(HINGE_LEFT_Element_Firing_Pin,HIGH);
//    if (((currentMicros - Last_Zero_Crossing_Time) > (hinge_left_firing_delay + 100)) && (gpio_get_level(HINGE_LEFT_Element_Firing_Pin))) {
//      digitalWrite(HINGE_LEFT_Element_Firing_Pin,LOW);
//      LHPS = true;
//    }
//  }
//  
//////    HINGE RIGHT WARMER CONTROL
//   if (!RHPS) {
//    if (((currentMicros - Last_Zero_Crossing_Time) > hinge_right_firing_delay) && (!gpio_get_level(HINGE_RIGHT_Element_Firing_Pin)))
//      digitalWrite(HINGE_RIGHT_Element_Firing_Pin,HIGH);
//    if (((currentMicros - Last_Zero_Crossing_Time) > (hinge_right_firing_delay + 100)) && (gpio_get_level(HINGE_RIGHT_Element_Firing_Pin))) {
//      digitalWrite(HINGE_RIGHT_Element_Firing_Pin,LOW);
//      RHPS = true;
//    }
//  }

  //    //FRONT LEFT WARMER CONTROL
   if (!LFPS) {
    if (((currentMicros - Last_Zero_Crossing_Time) > front_left_firing_delay) && (!gpio_get_level(FRONT_LEFT_Element_Firing_Pin)))
      digitalWrite(FRONT_LEFT_Element_Firing_Pin,HIGH);
    if (((currentMicros - Last_Zero_Crossing_Time) > (front_left_firing_delay + 100)) && (gpio_get_level(FRONT_LEFT_Element_Firing_Pin))) {
      digitalWrite(FRONT_LEFT_Element_Firing_Pin,LOW);
      LFPS = true;
    }
  }

 //    //FRONT RIGHT WARMER CONTROL
   if (!RFPS) {
    if (((currentMicros - Last_Zero_Crossing_Time) > front_right_firing_delay) && (!gpio_get_level(FRONT_RIGHT_Element_Firing_Pin)))
      digitalWrite(FRONT_RIGHT_Element_Firing_Pin,HIGH);
    if (((currentMicros - Last_Zero_Crossing_Time) > (front_right_firing_delay + 100)) && (gpio_get_level(FRONT_RIGHT_Element_Firing_Pin))) {
      digitalWrite(FRONT_RIGHT_Element_Firing_Pin,LOW);
      RFPS = true;
    }
  }

      //micros() = micros();
}
//End of void loop
