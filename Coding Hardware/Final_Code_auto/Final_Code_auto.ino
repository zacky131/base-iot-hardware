// note: saat pompa on -> jangan kirim data

  ///////////////////////////////////// LIBRARY
#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <WiFiManager.h>
WiFiManager wm;
bool res;

#include <Preferences.h>
Preferences preferences;

#include <Wire.h>

//I2C LCD Library
#include <LiquidCrystal_I2C.h>

//DHT room Temp and Humi sensor
#include <DHT.h>

//DFRobot Library
#include "GravityTDS.h"
#include <OneWire.h> 
#include <DallasTemperature.h>
#include "DFRobot_PH.h"

//Ticker for interrupt mqtt message from thingsboard (SWITCH CONTROL)
#include <Ticker.h> 
Ticker periodicTicker;

///////////////////////////////////////////////////// FIRST SETUP
#define TOKEN "komunitashidroponik1"

//define relay pin
// relay_1 = Nutrisi A
// relay_2 = Nutrisi B
// relay_3 = Air
// relay_4 = ??
#define relay_1 18 //13
#define relay_2 19 //12
#define relay_3 23 //11
#define relay_4 5  //10

// Define distance sensor connnection
int TRIGPIN = 4;  //A1       //34 //A3
int ECHOPIN = 35; //A2

// Define connection for DFRobot Sensor
#define ONE_WIRE_BUS 14 //7 

// Define DHT sensor connection
#define DHTPIN 17 //4
#define DHTTYPE DHT21   // DHT 21  (AM2302)
DHT dht(DHTPIN, DHTTYPE);   //Initialize DHT sensor for normal 16mhz Arduino
// Push button
#define pb_red 13 // Red 9 
#define pb_green 12 // Yellow 8 
#define pb_yellow 27 // green 6 
#define pb_black 16 // black 5

#define TdsSensorPin 39 //A5
#define PH_PIN 36 //A4

// SDAPIN = 25; //2
// SCLPIN = 26; //3

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 (SDA and SCL pin) for a 16 chars and 2 line display

int ok = 0;
int up = 0;
int down = 0;
int back = 0;

// Floats to calculate distance
float duration, distance;

// Variable for hydroponic sensor 
float tdsValue, temperature, voltage, phVal;
// Calibration variable
float ppm_target, ppm=0;
float k_factor, k=0;
float analogValue, voltagePH, analogValuepH;
float ecValue, ecValue25;
float neutralVoltage, acidVoltage;
float slope, intercept, t, h;
  
  //Starting setting PPH and PH variable
  //Starting PH Value for setting
  float settingPH = 7.0;
  //Starting PPM Value for setting
  float settingPPM = 500.0;
  //Starting Distance Val for setting
  float settingDistance = 5.0;
  //Starting Temperature Val for setting
  float settingTemp = 25.0;

  //intrrupt time (send to internet) (minutes)
  int interrupt_time = 1;

  //delay time for check automation parameter (offline) (minutes)
  int delay_time_check = 1;
  
char thingsboardServer[] = "178.128.20.61";
WiFiClient wifi;
PubSubClient client(wifi);
int status = WL_IDLE_STATUS;

OneWire oneWire(ONE_WIRE_BUS); 
GravityTDS gravityTds;
DallasTemperature sensors(&oneWire);

DFRobot_PH ph;


///////////////////////////////////////////////////////// SETUP ///////////////////////////////////////
void setup() {
// Set output mode for all GPIO pins
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

  digitalWrite(relay_1, LOW);
  digitalWrite(relay_2, LOW);
  digitalWrite(relay_3, LOW);
  digitalWrite(relay_4, LOW);
  
  Serial.begin(115200);
  dht.begin();

  //Init EEPROM
  preferences.begin("my-app", false);
  
  sensors.begin();
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
  
  set_gpio_status(relay_1, 0); //a
  set_gpio_status(relay_2, 0); //b
  set_gpio_status(relay_3, 0); //air
  
    // LCD Display
  pinMode(pb_black,INPUT_PULLUP);
  pinMode(pb_green,INPUT_PULLUP);
  pinMode(pb_yellow,INPUT_PULLUP);
  pinMode(pb_red,INPUT_PULLUP); 
  
  lcd.init();                      // initialize the lcd 
  lcd.backlight(); 
  delay(1000);
  menu();
  //  preferences.end();
  lcd.clear();

  delay(1000);
//  lcd.setCursor(6,0);
//  lcd.print("Welcome"); 
//  delay(2000);

  neutralVoltage = preferences.getFloat("vPHneu", 0);
  acidVoltage = preferences.getFloat("vPHacid", 0); 
  k_factor = preferences.getFloat("kFact", 0);

  // both in minutes
  interrupt_time = preferences.getInt("sIntTime");
  delay_time_check = preferences.getInt("sTimeC");
//  auto_control();
  delay(10);
  
  
//  InitWiFi();
//  client.setServer( thingsboardServer, 1883 );
//  client.setCallback(on_message);

  //interupt
  // bisa ubah sampling time di sini
  // interrupt_time in minutes
//  periodicTicker.attach_ms(interrupt_time*60000, sendData_toServer);
}
///////////////////////////////////////////////////////// LOOP ///////////////////////////////////////

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  getNprintData();
  client.loop();
}

void auto_control() {
  settingDistance = preferences.getFloat("sDistanceVal", 0);
  settingPPM = preferences.getFloat("sPPMval", 0);
  auto_relay_control();
}

// nutrition a to nutrition b ratio. default is 1
float a_b_ratio = 1;


// relay_1 = Nutrisi A
// relay_2 = Nutrisi B
// relay_3 = Air
// relay_4 = ??

void auto_relay_control(){
  // perlu ada button buat ngestop? pake interrupt?
  // cek air, isi air
  getNprintData();
  while (distance/100 > settingDistance) {
    lcd.setCursor(16,0);
    lcd.print("AUTO");
    
    set_gpio_status(relay_3, 1); //air
    getNprintData();
    delay(100);
  }
  set_gpio_status(relay_3, 0); //air off
  
//  bool stopnutrients = false;
  while (tdsValue < settingPPM) {
    set_gpio_status(relay_1, 1); //a
    set_gpio_status(relay_2, 1); //b
    getNprintData();
    delay(100);
  };
  set_gpio_status(relay_1, 0); //a off
  set_gpio_status(relay_2, 0); //b off
    
  // delay in minutes
  delay_time_check = preferences.getInt("sTimeC");
  delay(delay_time_check*60000);
}

void get_distance(){
  //code for ultrasonic distance sensor
  // Set the trigger pin LOW for 2uS
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
 
  // Set the trigger pin HIGH for 20us to send pulse
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(20);
 
  // Return the trigger pin to LOW
  digitalWrite(TRIGPIN, LOW);
 
  // Measure the width of the incoming pulse
  duration = pulseIn(ECHOPIN, HIGH);
 
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  // Divide by 100 as we want centimeters
 
  distance = (duration / 2) * 3.43;
}

void get_ppm(){
  k_factor = preferences.getFloat("kFact", 0);
  //Getting PPM Value from TDS sensor
  analogValue = analogRead(TdsSensorPin);
  voltage = analogValue/4096*5.0;
  ecValue=(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*k_factor;
  ecValue25  =  ecValue / (1.0+0.02*(temperature-25.0));  //temperature compensation
  tdsValue = ecValue25 * 0.5;
  k = ecValue/(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage);
}

void getNprintData(){
   
    get_distance();
    // Delay before repeating measurement
    delay(100);

    
    //Getting data from sensor and send data to thingsboard
    sensors.requestTemperaturesByIndex(0);
    temperature = sensors.getTempCByIndex(0);     //get temperatureerature value from sensor

    get_ppm();

    //getting PH Value
    voltagePH = analogRead(PH_PIN)/4096.0*5;  // read the voltage
    slope =  (7.0-4.0)/((neutralVoltage-1.5)/3.0 - (acidVoltage-1.5)/3.0);  // two point: (_neutralVoltage,7.0),(_acidVoltage,4.0);
    intercept =   7.0 - slope*(neutralVoltage-1.5)/3.0;     
    phVal = slope*(voltagePH-1.5)/3.0+intercept;  //y = k*x + b
    
    // DHT sensor read
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    lcd.clear();
    lcd.setCursor(11,0);
    lcd.print("    ");
    lcd.setCursor(11,1);
    lcd.print("    ");  
  
    lcd.setCursor(11,2);
    lcd.print("    ");  
  
    lcd.setCursor(11,3);
    lcd.print("         ");
    
    lcd.setCursor(0,0);
    lcd.print("PPM:"); 
    lcd.print(tdsValue,0);
    
    lcd.setCursor(0,1);
    lcd.print("T1:"); 
    lcd.print(temperature);
    lcd.setCursor(8,1);
    lcd.print("  T2:"); 
    lcd.print(t);
  
    lcd.setCursor(0,2);
    lcd.print("PH:"); 
    lcd.print(phVal);
    lcd.setCursor(9,2);
    lcd.print("Hum: "); 
    lcd.print(h);
  
    lcd.setCursor(0,3);
    lcd.print("Jarak(cm): "); 
    float newDistance = distance/100;
    lcd.print(newDistance,1);
}

void sendData_toServer(){
  
 /////////////////////////////////////////// STRING TO JSON
    String packet = "";
    packet.concat(("{\"temperature\":"));
    packet.concat(temperature);
        
    packet.concat((",\"TDS\":"));
    packet.concat(tdsValue);
       
    packet.concat((",\"PH\":"));
    packet.concat(phVal);
  
    packet.concat((",\"Distance\":"));
    float newDistance = distance/100;
    packet.concat(newDistance);
  
    packet.concat((",\"Room Temperature\":"));
    packet.concat(t);
  
    packet.concat((",\"Room Humidity\":"));
    packet.concat(h);
    
    packet.concat("}");
    client.publish("v1/devices/me/telemetry", packet.c_str(), true); 

}
// The callback for when a PUBLISH message is received from the server.
//void on_message(const char* topic, byte* payload, unsigned int length) {
//
//  Serial.println("On message");
//
//  char json[length + 1];
//  strncpy (json, (char*)payload, length);
//  json[length] = '\0';
//
//  Serial.print("Topic: ");
//  Serial.println(topic);
//  Serial.print("Message: ");
//  Serial.println(json);
//
//  // Decode JSON request
//  StaticJsonBuffer<200> jsonBuffer;
//  JsonObject& data = jsonBuffer.parseObject((char*)json);
//
//  if (!data.success())
//  {
//    Serial.println("parseObject() failed");
//    return;
//  }
//
//  // Check request method
//  String methodName = String((const char*)data["method"]);
//
//  if (methodName.equals("getValue")) {
//    // Reply with GPIO status
//    String responseTopic = String(topic);
//    responseTopic.replace("request", "response");
//
//  } else if (methodName.equals("setValue")) {
//    // Update GPIO status and reply
//    set_gpio_status(data["params"]["pin"], data["params"]["enabled"]);
//    String responseTopic = String(topic);
//    responseTopic.replace("request", "response");
//  }
//}


void set_gpio_status(int pin, boolean enabled) {
  if (pin == relay_1) {
    // Output GPIOs state
    digitalWrite(relay_1, not(enabled ? HIGH : LOW));

  } 
  else if (pin == relay_2) {
    // Output GPIOs state
    digitalWrite(relay_2, not(enabled ? HIGH : LOW));

  } 
  else if (pin == relay_3) {
    // Output GPIOs state
    digitalWrite(relay_3, not(enabled ? HIGH : LOW));

  } 
  else if (pin == relay_4) {
    // Output GPIOs state
    digitalWrite(relay_4, not(enabled ? HIGH : LOW));

  }
}

void InitWiFi() {
    Serial.println("Connecting to AP ...");
  // attemperaturet to connect to WiFi network
/*
  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  digitalWrite(WiFi_LED_ON, HIGH);
*/  
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to AP ...");
  // attemperaturet to connect to WiFi network

  res = wm.autoConnect("ESPWiFiManager");
  while (!res) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  lcd.setCursor(19,0);
  lcd.print("1");
  
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Connected to AP");
      Serial.print("Connecting to ThingsBoard node ...");
      // Attemperaturet to connect (clientId, username, password)
      if (client.connect("Wemos D1 R32 Device", TOKEN, NULL) ) {
        Serial.println( "[DONE]" );
        // Subscribing to receive RPC requests
        client.subscribe("v1/devices/me/rpc/request/+");
        // Sending current GPIO status
        Serial.println("Sending current GPIO status ...");
        lcd.setCursor(19,0);
        lcd.print("1");
      } else {
        getNprintData();
        Serial.print( "[FAILED] [ rc = " );
        Serial.print( client.state() );
        Serial.println( " : retrying in 5 seconds]" );
        // Wait 5 seconds before retrying
        lcd.setCursor(19,0);
        lcd.print("0");
      }
   }
}


// Menu Selection and Choice
void menu(){
menu:
    set_gpio_status(relay_1, 0); //a
    set_gpio_status(relay_2, 0); //b
    set_gpio_status(relay_3, 0); //air
    lcd.clear();
    while(1)
    {

    lcd.setCursor(0,0);
    lcd.print("1. Calibration");          
    lcd.setCursor(0,1);
    lcd.print("2. Start Online");
    lcd.setCursor(0,2);
    lcd.print("3. Set Parameter");
    lcd.setCursor(0,3);
    lcd.print("4. Start Offline");
    delay(100);
    
    ok = digitalRead(pb_red);
    up = digitalRead(pb_yellow);
    down = digitalRead(pb_green);
    back = digitalRead(pb_black);
    
    if (ok == LOW) { delay(300); goto menu1; }
    if (up == LOW) { delay(300); return; }
    if (down == LOW) { delay(300); goto menu7;  }
    if (back == LOW) { delay(300); goto menuOffline;  }
    }   
          
menuOffline:
  lcd.clear();
  while(1){
    set_gpio_status(relay_1, 0); //a
    set_gpio_status(relay_2, 0); //b
    set_gpio_status(relay_3, 0); //air
    
    getNprintData();

    Serial.print(distance);
    lcd.setCursor(10,0);
    lcd.print("  1.A  2.M");
    
    ok = digitalRead(pb_red);
    up = digitalRead(pb_yellow);
    down = digitalRead(pb_green);
    back = digitalRead(pb_black);
    
    if (ok == LOW) { delay(300); goto menuAutoControl; }
//    if (ok == LOW) { delay(300); return; }
    if (up == LOW) { delay(300); goto menuManualControl; }
    if (down == LOW) { }
    if (back == LOW) { delay(300); goto menu; }
  }

menuAutoControl:
  lcd.clear();
  while(1){
    //read and show ppm
    auto_control();
    
    delay(100);
    
    ok = digitalRead(pb_red);
    if (ok == LOW) { };

    up = digitalRead(pb_yellow);
    if (up == LOW) { };
    
    down = digitalRead(pb_green);
    if (down == LOW) {  };

    back = digitalRead(pb_black);
    if (back == LOW) { delay(300); goto menuOffline; };
  }


menuManualControl:
  lcd.clear();
  while(1){
    //read and show ppm
    get_ppm();
    lcd.setCursor(0,0);
    lcd.print("                    ");
    
    lcd.setCursor(0,0);
    lcd.print("PPM: ");
    lcd.setCursor(5,0);
    lcd.print(tdsValue,0);

    //read and show water distance
    get_distance();
    lcd.setCursor(10,0);
    lcd.print("Dis: ");
    lcd.setCursor(14,0);
    lcd.print((distance/100),1);

    lcd.setCursor(0,1);
    lcd.print("1. Add nutrition");
    
    lcd.setCursor(0,2);
    lcd.print("2. Add water");
    
    lcd.setCursor(0,3);
    lcd.print("4. Back");
    
    delay(100);
    
    ok = digitalRead(pb_red);
    if (ok == LOW) { 
      delay(100);
      set_gpio_status(relay_1, 1); //a
      set_gpio_status(relay_2, 1); //b
    }
    else {
      delay(100);
      set_gpio_status(relay_1, 0); //a
      set_gpio_status(relay_2, 0); //b
    };

    up = digitalRead(pb_yellow);
    if (up == LOW) { 
      delay(100);
      set_gpio_status(relay_3, 1); //air
    } 
    else { 
      delay(100);
      set_gpio_status(relay_3, 0); //air
    };
    
    down = digitalRead(pb_green);
    if (down == LOW) {  };

    back = digitalRead(pb_black);
    if (back == LOW) { delay(300); goto menuOffline; };
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
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { delay(300); goto menu2; }
          if (up == LOW) { delay(300); goto menu4; }
          if (down == LOW) {  }
          if (back == LOW) { delay(300); goto menu; }
          }

menu2:
          lcd.clear();
          while(1)
          {
          
          float _neutralvoltage = preferences.getFloat("vPHneu", 0);
          float _acidvoltage = preferences.getFloat("vPHacid", 0);
          
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
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { delay(300); goto menu3; }
          if (up == LOW) {  }
          if (down == LOW) {  }
          if (back == LOW) { delay(300); goto menu1; }
          }

menu3:

          lcd.clear();
          while(1)
          {

          voltagePH = analogRead(PH_PIN)/4096.0*5;  // read the voltage

         
                    
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
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(300); 
            preferences.putFloat("vPHneu", voltagePH);
            
            goto menu2; 
            }

          
          if (up == LOW) { 
            
            delay(300); 

            preferences.putFloat("vPHacid", voltagePH);
            
            goto menu2; }
            
          if (down == LOW) {  }
          
          if (back == LOW) { delay(300); goto menu2; }
          }



menu4:
          lcd.clear();
          while(1)
          {

          k_factor = preferences.getFloat("kFact", 0);

                    
          lcd.setCursor(0,0);
          lcd.print("K Value : ");
          lcd.print(k_factor);
          
  
          lcd.setCursor(0,3);
          lcd.print("1. Change");
          lcd.setCursor(13,3);
          lcd.print("4. Back");
          
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
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
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(300); 
           
            goto menu6; 
            }

          
          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  tdsValue = tdsValue+10.0; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); tdsValue = tdsValue-10.0; }
          if (back == LOW) { delay(300); goto menu4; }
          }

menu6:

          lcd.clear();
          while(1)
          {

          sensors.requestTemperatures();
          temperature = sensors.getTempCByIndex(0);

          analogValue = analogRead(TdsSensorPin);
          voltage = analogValue/4096*5.0;
          
          ecValue25 = tdsValue/TdsFactor;
          ecValue = ecValue25*(1.0+0.02*(temperature-25.0));
          k = ecValue/(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage);
                       
          lcd.setCursor(0,0);
          lcd.print("Buffer   : ");
          lcd.print(tdsValue,1);
          lcd.print(" ppm");

          lcd.setCursor(0,1);
          lcd.print("k factor : ");
          lcd.print(k);
          
          lcd.setCursor(0,3);
          lcd.print("1. Save");
          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(200); 

            
            preferences.putFloat("kFact", k);
            
            goto menu4; 
            }

          
          if (back == LOW) { delay(200); goto menu5; }
          }     
            
menu7:
          lcd.clear();
          while(1)
          {
          
          lcd.setCursor(0,0);
          lcd.print("1.PH|PPM Set");
          
          lcd.setCursor(0,1);
          lcd.print("2.Jarak|Temp Set");

          lcd.setCursor(0,2);
          lcd.print("3.Check|Send Time");
          
          lcd.setCursor(0,3);
          lcd.print("4.Back");
          
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { delay(300); goto menu8; }
          if (up == LOW) { delay(300); goto menu9; }
          if (down == LOW) { delay(300); goto menu14; }
          if (back == LOW) { delay(300); goto menu; }
          }

menu8:
          lcd.clear();
          while(1)
          {
          
          lcd.setCursor(0,0);
          lcd.print("1.PH Set");
          
          lcd.setCursor(0,1);
          lcd.print("2.PPM Set");
          
          lcd.setCursor(0,3);
          lcd.print("4. Back");
          
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { delay(300); goto menu10; }
          if (up == LOW) { delay(300); goto menu11; }
          if (down == LOW) {  }
          if (back == LOW) { delay(300); goto menu7; }
          }

menu9:
          lcd.clear();
          while(1)
          {
          
          lcd.setCursor(0,0);
          lcd.print("1.Jarak Set");
          
          lcd.setCursor(0,1);
          lcd.print("2.Temperature Set");
          
          lcd.setCursor(0,3);
          lcd.print("4. Back");
          
          delay(100);

          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { delay(300); goto menu12; }
          if (up == LOW) { delay(300); goto menu13; }
          if (down == LOW) {  }
          if (back == LOW) { delay(300); goto menu7; }
          }
          
menu10:
          settingPH = preferences.getFloat("sPHval", 0);  
          lcd.clear();
          while(1)
          {       
          
//          float settingPHc = settingPH;      
          lcd.setCursor(0,0);
          lcd.print("PH   : ");
          lcd.print(settingPH,1);
          
          lcd.setCursor(0,2);
          lcd.print("1. Set");

          lcd.setCursor(0,3);
          lcd.print("2. Up");

          lcd.setCursor(13,2);
          lcd.print("3. Down");
         

          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(300); 
                      
            preferences.putFloat("sPHval", settingPH);
            goto menu8;
            }

          
          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  settingPH = settingPH + 0.1; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); settingPH = settingPH - 0.1; }
          if (back == LOW) { delay(300); goto menu8; }
          }

menu11:  
          settingPPM = preferences.getFloat("sPPMval", 0);  
          lcd.clear();
          while(1)
          {
          
//          float settingPPMc = settingPPM;    
                    
          lcd.setCursor(0,0);
          lcd.print("PPM   : ");
          lcd.print(settingPPM,1);
          
          lcd.setCursor(0,2);
          lcd.print("1. Set");

          lcd.setCursor(0,3);
          lcd.print("2. Up");

          lcd.setCursor(13,2);
          lcd.print("3. Down");
         

          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { 
            delay(300); 
                      
            preferences.putFloat("sPPMval", settingPPM);
            goto menu8;
            }

          
          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  settingPPM = settingPPM+10.0; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); settingPPM = settingPPM-10.0; }
          if (back == LOW) { delay(300); goto menu8; }
          }

menu12:  
          settingDistance = preferences.getFloat("sDistanceVal", 0);  
          lcd.clear();
          while(1)
          {       
          lcd.setCursor(0,0);
          lcd.print("Jarak (cm): ");
          lcd.print(settingDistance,1);
          
          lcd.setCursor(0,2);
          lcd.print("1. Set");

          lcd.setCursor(0,3);
          lcd.print("2. Up");

          lcd.setCursor(13,2);
          lcd.print("3. Down");
         

          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(300); 
                      
            preferences.putFloat("sDistanceVal", settingDistance);
            goto menu9;
            }

          
          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  settingDistance = settingDistance + 0.5; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); settingDistance = settingDistance - 0.5; }
          if (back == LOW) { delay(300); goto menu9; }
          }

menu13:  
          settingTemp = preferences.getFloat("sTempVal", 0);  
          lcd.clear();
          while(1)
          {       
          lcd.setCursor(0,0);
          lcd.print("Temperature(C): ");
          lcd.print(settingTemp,1);
          
          lcd.setCursor(0,2);
          lcd.print("1. Set");

          lcd.setCursor(0,3);
          lcd.print("2. Up");

          lcd.setCursor(13,2);
          lcd.print("3. Down");
         

          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(300); 
                      
            preferences.putFloat("sTempVal", settingTemp);
            goto menu9;
            }

          
          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  settingTemp = settingTemp + 0.5; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); settingTemp = settingTemp - 0.5; }
          if (back == LOW) { delay(300); goto menu9; }
          }

menu14:
          lcd.clear();
          while(1)
          {
          
          lcd.setCursor(0,0);
          lcd.print("1.Check Time");
          
          lcd.setCursor(0,1);
          lcd.print("2.To Internet Time");
          
          lcd.setCursor(0,3);
          lcd.print("4. Back");
          
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { delay(300); goto menu15; }
          if (up == LOW) { delay(300); goto menu16; }
          if (down == LOW) { delay(300); }
          if (back == LOW) { delay(300); goto menu7; }
          }
// check time
menu15:
          
          delay_time_check = preferences.getInt("sTimeC");
          lcd.clear();
          while(1)
          {       
          
          lcd.setCursor(0,0);
          lcd.print("Check Time (m): ");
          lcd.print(delay_time_check);
          
          lcd.setCursor(0,2);
          lcd.print("1. Set");

          lcd.setCursor(0,3);
          lcd.print("2. Up");

          lcd.setCursor(13,2);
          lcd.print("3. Down");
         

          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(300); 
            preferences.putInt("sTimeC", delay_time_check);
            goto menu14;
            }

          
          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  delay_time_check = delay_time_check + 1; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); delay_time_check = delay_time_check - 1; }
          if (back == LOW) { delay(300); goto menu14; }
          }

          
// send to internet delay time
menu16:
          interrupt_time = preferences.getInt("sIntTime");
          lcd.clear();
          while(1)
          {       
          
          lcd.setCursor(0,0);
          lcd.print("Send Time (m): ");
          lcd.print(interrupt_time);
          
          lcd.setCursor(0,2);
          lcd.print("1. Set");

          lcd.setCursor(0,3);
          lcd.print("2. Up");

          lcd.setCursor(13,2);
          lcd.print("3. Down");
         

          lcd.setCursor(13,3);
          lcd.print("4. Back");
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          if (ok == LOW) { 
            delay(300); 
            preferences.putInt("sIntTime", interrupt_time);
            goto menu14;
            }

          if (up == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      ");  interrupt_time = interrupt_time + 1; }
          if (down == LOW) { delay(50); lcd.setCursor(9,0); lcd.print("      "); interrupt_time = interrupt_time - 1; }
          if (back == LOW) { delay(300); goto menu14; }
          }
} 
