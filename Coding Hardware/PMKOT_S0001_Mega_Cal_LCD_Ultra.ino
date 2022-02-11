//Written by: Muhammad Zacky Asy'ari. auftechnique.com December 2020

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <GravityTDS.h>
#include <DFRobot_PH.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Ultrasonic.h>


// Push button
#define pb_red 12 // hitam
#define pb_yellow 11 //kuning
#define pb_blue 10 //putih
#define pb_green 9 //merah



//LCD Display
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 (SDA and SCL pin) for a 16 chars and 2 line display

// FOR TEMPERATURE SENSOR
// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// FOR Water level
Ultrasonic ultrasonic (6,7);  //trig - echo
float distance;

//FOR pH SENSOR
#define pH_pin A3

//FOR TDS SENSOR
#define TdsSensorPin A0


//Variable
#define ref 5.0      // analog reference voltage(Volt) of the ADC
#define adcRange 1024      // analog reference voltage(Volt) of the ADC
#define TdsFactor 0.5      // analog reference voltage(Volt) of the ADC

unsigned long lastTime = 0;
unsigned long timerDelay = 1000;

int ok = 0;
int up = 0;
int down = 0;
int back = 0;

float ppm_target, ppm=0;
float k_factor, k=0;
float analogValue, voltage, voltagePH, analogValuepH, temperature=25;
float ecValue, ecValue25, tdsValue, temp;
float phValue, neutralVoltage, acidVoltage;
float tinggiAir;

String kirim = "";


//union  value
typedef union{
  float flt;
  byte array[4];
} FloatConverter;

//write
void writeEEPROM(int address, float value){
  FloatConverter aConverter; //create a new variable of type FloatConverter
  aConverter.flt = value; //set its value (using the float blueprint) to the value of config
  for(byte i = 0; i < 4; i++){
    EEPROM.write(address+i,aConverter.array[i]); //store each of the 4 bytes of aConverter to the EEPROM, accessing them using the byte[4] blueprint
   // Serial.print(aConverter.array[i]);
  };
   // Serial.println();
}


//read
float readEEPROM(int address)
{
  float value;
  FloatConverter aConverter; //create a new variable of type FloatConverter
  for(byte i = 0; i < 4; i++){
    aConverter.array[i] = EEPROM.read(address+i); //read 4 bytes from the EEPROM to aConverter using the byte[4] blueprint
  }
  value = aConverter.flt; //set the value of config to the value of aConverter using the float blueprint}
  //Serial.println(value);
  return value;

}

void read_tds(){
  sensors.requestTemperatures();
  analogValue = analogRead(TdsSensorPin);
  voltage = analogValue/adcRange*ref;
  ecValue=(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*k_factor;
  ecValue25  =  ecValue / (1.0+0.02*(sensors.getTempCByIndex(0)-25.0));  //temperature compensation
  tdsValue = ecValue25 * TdsFactor;  
}

void read_pH(){
  float slope = (7.0-4.0)/((neutralVoltage-1.5)/3.0 - (acidVoltage-1.5)/3.0);  // two point: (_neutralVoltage,7.0),(_acidVoltage,4.0)
  float intercept =  7.0 - slope*(neutralVoltage-1.5)/3.0;    
  phValue = slope*(voltagePH-1.5)/3.0+intercept;  //y = k*x + b
  
}



void cal_tds(int cal){
  sensors.requestTemperatures();
  analogValue = analogRead(TdsSensorPin);
  voltage = analogValue/adcRange*ref;
  
  ecValue25 = cal/TdsFactor;
  ecValue = ecValue25 * (1.0+0.02*(sensors.getTempCByIndex(0)-25.0));
  k_factor = ecValue/(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage);
}




void setup(){
  //Begin both serial communication
  Serial.begin(57600);
  Serial3.begin(57600);
  // Sensor TDS
  pinMode(TdsSensorPin,INPUT);
  // Sensor pH
  pinMode(pH_pin,INPUT);
  //sensor Temperature
  sensors.begin();   
  
  // LCD Display
  pinMode(pb_green,INPUT_PULLUP);
  pinMode(pb_blue,INPUT_PULLUP);
  pinMode(pb_yellow,INPUT_PULLUP);
  pinMode(pb_red,INPUT_PULLUP); 

  lcd.init();                      // initialize the lcd 
  lcd.backlight(); 
  delay(2000);
  menu();
  lcd.clear();

  Serial.begin(57600);
  lcd.setCursor(6,0);
  lcd.print("Welcome"); 
  delay(2000);
  lcd.clear();
}

  
void loop(){

  
  // Init coeficient
  neutralVoltage = readEEPROM(0);
  acidVoltage = readEEPROM(10);
  k_factor = readEEPROM(20);

  
  
  int var = 1; // Update data on LCD every 1 second
  while (var < 115 ) { // Update data on Cloud every 115 second
  //Sensor measurements  
  if ((millis() - lastTime) > timerDelay) {

  // SEND WATER LEVEL DATA
  float tinggiAir = ultrasonic.distanceRead();
  Serial.print("Jarak = ");
  Serial.print(tinggiAir);
  Serial.println("cm");
  delay(500);
  

  //Temperature Measurement
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
  Serial.print("temperature Reading ==> "); 
  Serial.print("Celsius temperature: ");
  Serial.println(temp);
  delay(500);

  // TDS Measurement
  
  analogValue = analogRead(TdsSensorPin);
  voltage = analogValue/adcRange*ref;
  ecValue=(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*k_factor;
  ecValue25  =  ecValue / (1.0+0.02*(temp-25.0));  //temperature compensation
  tdsValue = ecValue25 * TdsFactor;
  Serial.print("TDS Reading==> ");
  Serial.print(tdsValue,0);
  Serial.println(" ppm");
  delay(500);
  
  // pH Measurement
  analogValuepH = analogRead(pH_pin);
  voltagePH = analogValuepH/adcRange*ref;
  float slope = (7.0-4.0)/((neutralVoltage-1.5)/3.0 - (acidVoltage-1.5)/3.0);  // two point: (_neutralVoltage,7.0),(_acidVoltage,4.0)
  float intercept =  7.0 - slope*(neutralVoltage-1.5)/3.0;    
  phValue = slope*(voltagePH-1.5)/3.0+intercept;  //y = k*x + b
  Serial.print("pH Reading ==> ");
  Serial.print("temperature pH:");
  Serial.print(temp,1);
  Serial.print("^C  pH:");
  Serial.println(phValue,2);

  /*Serial.println (slope);
  Serial.println (intercept);
  Serial.println (phValue);
  */  
  
  lcd.setCursor(11,0);
  lcd.print("    ");
  lcd.setCursor(11,1);
  lcd.print("    ");  

  lcd.setCursor(11,2);
  lcd.print("    ");  

  lcd.setCursor(11,3);
  lcd.print("         ");
  
  lcd.setCursor(0,0);
  lcd.print("PPM       : "); 
  lcd.print(tdsValue,1);
  
  lcd.setCursor(0,1);
  lcd.print("Temp(degC): "); 
  lcd.print(temp);

  lcd.setCursor(0,2);
  lcd.print("pH        : "); 
  lcd.print(phValue,1);

  lcd.setCursor(0,3);
  lcd.print("Jarak  Air: "); 
  lcd.print(tinggiAir);

  /*lcd.setCursor(11,1);
  lcd.print("T:"); 
  lcd.print(temp,0);
  */
  
  lastTime = millis();
  }
  var++;  
  delay(1000);  
} 

  
  //Send Data to ESP
  kirim  = "";
  kirim += temp;
  kirim += ";";
  kirim += ultrasonic.distanceRead();
  kirim += ";";
  kirim += phValue;
  kirim += ";";
  kirim += tdsValue;
  Serial.println(kirim);  
  Serial3.println(kirim);
  
  if(Serial3.available()){
    String msg = "";
    while(Serial3.available()){
      msg += char(Serial3.read());
      delay(500);
    }
    Serial.println(msg);
  }  
  delay(1000);// This is important, need longer time if you send longer data 
}



// Menu Selection and Choice
void menu(){
menu:
          lcd.clear();
          while(1)
          {

          lcd.setCursor(0,0);
          lcd.print("      Main Menu");
          
          lcd.setCursor(0,2);
          lcd.print("1. Calibration");
          lcd.setCursor(0,3);
          lcd.print("2. Start");
          delay(100);
          
          ok = digitalRead(pb_green);
          up = digitalRead(pb_blue);
          down = digitalRead(pb_yellow);
          back = digitalRead(pb_red);
          if (ok == LOW) { delay(300); goto menu1; }
          if (up == LOW) { delay(300); return; }
          if (down == LOW) {  }
          if (back == LOW) {  }
          }

menu1:
          lcd.clear();
          while(1)
          {
          
          lcd.setCursor(0,0);
          lcd.print("1. pH Cal");
          
          lcd.setCursor(0,1);
          lcd.print("2. TDS Cal");
          
          lcd.setCursor(0,3);
          lcd.print("4. Back");
          
          delay(100);
          
          ok = digitalRead(pb_green); // merah
          up = digitalRead(pb_blue);
          down = digitalRead(pb_yellow);
          back = digitalRead(pb_red);
          if (ok == LOW) { delay(300); goto menu2; }
          if (up == LOW) { delay(300); goto menu4; }
          if (down == LOW) {  }
          if (back == LOW) { delay(300); goto menu; }
          }

menu2:
          lcd.clear();
          while(1)
          {

          float _neutralvoltage = readEEPROM(0);
          float _acidvoltage = readEEPROM(10);
          
          lcd.setCursor(0,0);
          lcd.print("Neutral : ");
          lcd.print(_neutralvoltage,1);
          lcd.print(" V");

          lcd.setCursor(0,1);
          lcd.print("Acid    : ");
          lcd.print(_acidvoltage,1);
          lcd.print(" V");
  
          lcd.setCursor(0,3);
          lcd.print("1. Change");
          lcd.setCursor(13,3);
          lcd.print("4. Back");
          
          delay(100);
          
          ok = digitalRead(pb_green); //merah
          up = digitalRead(pb_blue); //putih
          down = digitalRead(pb_yellow); // kuning
          back = digitalRead(pb_red); // hitam
          if (ok == LOW) { delay(300); goto menu3; }
          if (up == LOW) {  }
          if (down == LOW) {  }
          if (back == LOW) { delay(300); goto menu1; }
          }

menu3:

          lcd.clear();
          while(1)
          {

          voltagePH = analogRead(pH_pin)/1024.0*5;  // read the voltage

         
                    
          lcd.setCursor(0,0);
          lcd.print("Voltage : ");
          lcd.print(voltagePH,1);
          lcd.print(" V");
          
          lcd.setCursor(0,1);
          lcd.print("1. Save Neutral");

          lcd.setCursor(0,2);
          lcd.print("2. Save Acid");
          lcd.setCursor(0,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_green);
          up = digitalRead(pb_blue);
          down = digitalRead(pb_yellow);
          back = digitalRead(pb_red);
          if (ok == LOW) { 
            delay(300); 

            writeEEPROM(0, voltagePH);
            
            goto menu2; 
            }

          
          if (up == LOW) { 
            
            delay(300); 

            writeEEPROM(10, voltagePH);
            
            goto menu2; }
            
          if (down == LOW) {  }
          
          if (back == LOW) { delay(300); goto menu2; }
          }



menu4:
          lcd.clear();
          while(1)
          {

          k_factor = readEEPROM(20);

                    
          lcd.setCursor(0,0);
          lcd.print("K Value : ");
          lcd.print(k_factor);
          
  
          lcd.setCursor(0,3);
          lcd.print("1. Change");
          lcd.setCursor(13,3);
          lcd.print("4. Back");
          
          delay(100);
          
          ok = digitalRead(pb_green);
          up = digitalRead(pb_blue);
          down = digitalRead(pb_yellow);
          back = digitalRead(pb_red);
          if (ok == LOW) { delay(300); goto menu5; }
          if (up == LOW) {  }
          if (down == LOW) {  }
          if (back == LOW) { delay(300); goto menu1; }
          }


menu5:

          lcd.clear();
          while(1)
          {

                             
          lcd.setCursor(0,0);
          lcd.print("PPM   : ");
          lcd.print(tdsValue,0);
          
          lcd.setCursor(0,2);
          lcd.print("1. Set");

          lcd.setCursor(0,3);
          lcd.print("2. Up");

          lcd.setCursor(13,2);
          lcd.print("3. Down");
         

          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_green);
          up = digitalRead(pb_blue);
          down = digitalRead(pb_yellow);
          back = digitalRead(pb_red);
          if (ok == LOW) { 
            delay(300); 
           
            goto menu6; 
            }

          
          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  tdsValue = tdsValue+2.5; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); tdsValue = tdsValue-2.5; }
          if (back == LOW) { delay(300); goto menu4; }
          }

menu6:

          lcd.clear();
          while(1)
          {

          sensors.requestTemperatures();
          temp = sensors.getTempCByIndex(0);

          analogValue = analogRead(TdsSensorPin);
          voltage = analogValue/adcRange*ref;

          ecValue25 = tdsValue/TdsFactor;
          ecValue = ecValue25*(1.0+0.02*(temp-25.0));
          k = ecValue/(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage);
                    
          lcd.setCursor(0,0);
          lcd.print("Buffer   : ");
          lcd.print(tdsValue,0);
          lcd.print(" ppm");

          lcd.setCursor(0,1);
          lcd.print("k factor : ");
          lcd.print(k);
          
          lcd.setCursor(0,3);
          lcd.print("1. Save");
          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_green);
          up = digitalRead(pb_blue);
          down = digitalRead(pb_yellow);
          back = digitalRead(pb_red);
          if (ok == LOW) { 
            delay(200); 

            
            writeEEPROM(20, k);
            
            goto menu4; 
            }

          
          if (back == LOW) { delay(200); goto menu5; }
          }       
     

}
