#include <Wire.h>
//#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Gerry added this comments to test git push

const int ADDR = 0x40;
const int MeasureTemp = 0xE3;
int X0,X1,temp;
double X,X_out;

//Variables to assist delay to track with oven delay
int temp_read_Delay = 500000;
unsigned long previousMicros = 0; 
unsigned long currentMicros = 0;

double Temp_S1, Temp_S2, Temp_S3, Temp_S4, Temp_S5;
double Old_Temp_S1 = 0.00;
double Old_Temp_S2 = 0.00;
double Old_Temp_S3 = 0.00;
double Old_Temp_S4 = 0.00;
double Old_Temp_S5 = 0.00;

void setup() 
{
  Serial.begin(9600);
//  lcd.init();                      // initialize the lcd 
//  lcd.backlight();
//  lcd.clear();
}

double GetTemp(int SDA_Pin, int SLC_pin) 
{
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

void loop() 
{ 
 currentMicros = micros(); 
 if(currentMicros - previousMicros >= temp_read_Delay){
    previousMicros += temp_read_Delay; 
    //Get Value from Sensor #1 (Hinge Left)
    Temp_S1 = GetTemp(16,18) + 2.20;  
    
    if(Old_Temp_S1 == 0.00){
      Old_Temp_S1 = Temp_S1;
    } else{
        if((abs(Temp_S1 - Old_Temp_S1)) > 5.00)
          Temp_S1 = Old_Temp_S1;
        else
          Old_Temp_S1 = Temp_S1;
      }
      
    //Get Value from Sensor #2 (Front Left)
    Temp_S2 = GetTemp(17,19) + 2.07;  
    
    if(Old_Temp_S2 == 0.00){
      Old_Temp_S2 = Temp_S2;
    } else{
        if((abs(Temp_S2 - Old_Temp_S2)) > 5.00)
          Temp_S2 = Old_Temp_S2;
        else
          Old_Temp_S2 = Temp_S2;
      }
      
    //Get Value from Sensor #3 (Center)
    Temp_S3 = GetTemp(15,4);  
    
    if(Old_Temp_S3 == 0.00){
      Old_Temp_S3 = Temp_S3;
    } else{
        if((abs(Temp_S3 - Old_Temp_S3)) > 5.00)
          Temp_S3 = Old_Temp_S3;
        else
          Old_Temp_S3 = Temp_S3;
      }
  
      
    //Get Value from Sensor #4 (Hinge Right)
    Temp_S4 = GetTemp(14,25);  
    
    if(Old_Temp_S4 == 0.00){
      Old_Temp_S4 = Temp_S4;
    } else{
        if((abs(Temp_S4 - Old_Temp_S4)) > 5.00)
          Temp_S4 = Old_Temp_S4;
        else
          Old_Temp_S4 = Temp_S4;
      }
  
    //Get Value from Sensor #5 (Front Right)
    Temp_S5 = GetTemp(27,33) - 3.50;  
    
    if(Old_Temp_S5 == 0.00){
      Old_Temp_S5 = Temp_S5;
    } else{
        if((abs(Temp_S5 - Old_Temp_S5)) > 5.00)
          Temp_S5 = Old_Temp_S5;
        else
          Old_Temp_S5 = Temp_S5;
      } 
         
    Serial.print("," +String(Temp_S1)); // Hinge Left
    Serial.print("," +String(Temp_S2)); // Front Left
    Serial.print("," +String(Temp_S3)); // Center
    Serial.print("," +String(Temp_S4)); // Front Right
    Serial.print("," +String(Temp_S5)); // Hinge Right 
    Serial.println(); 
 }
}
