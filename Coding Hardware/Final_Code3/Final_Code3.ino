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
#define TOKEN "9NhrINrrAYd0Cmg1nT1l"

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
int DHTPIN = 17; //4
#define DHTTYPE DHT21   // DHT 21  (AM2302)
DHT dht(DHTPIN, DHTTYPE);   //Initialize DHT sensor for normal 16mhz Arduino
// Push button
#define pb_red 13 // Red 9 
#define pb_yellow 12 // Yellow 8 
#define pb_green 27 // green 6 
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
  Serial.begin(115200);
  dht.begin();  
  
  //Init EEPROM
  preferences.begin("my-app", false);
   
  sensors.begin();
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096);  //1024   for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
  
  // Set output mode for all GPIO pins
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

  digitalWrite(relay_1, HIGH);
  digitalWrite(relay_2, HIGH);
  digitalWrite(relay_3, HIGH);
  digitalWrite(relay_4, HIGH);

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
  lcd.setCursor(6,0);
  lcd.print("Welcome"); 
  delay(2000);

  neutralVoltage = preferences.getFloat("vPHneu", 0);
  acidVoltage = preferences.getFloat("vPHacid", 0); 
  k_factor = preferences.getFloat("kFact", 0);
   
  delay(10);
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  client.setCallback(on_message);

  //interupt
  // bisa ubah sampling time di sini
  periodicTicker.attach_ms(5000, sendData_toServer);
}
///////////////////////////////////////////////////////// LOOP ///////////////////////////////////////
void loop() {
  if (!client.connected()) {
    reconnect();
  }
    getNprintData();
    client.loop();
  
}

//initial values 0
float distance_min_5 = 0;
float distance_min_4 = 0;
float distance_min_3 = 0;
float distance_min_2 = 0;
float distance_min_1 = 0;
float avg_distance = 0;

// hapus distance kalo sus, rata rata kalo aman
void MA_distance(float new_distance){ // data baru setiap 5 detik
  if (new_distance < 2.5*avg_distance){ //kalo kejauhan diskip aja
    avg_distance = (distance_min_5 + distance_min_4 + distance_min_3 + distance_min_2 + distance_min_1 + new_distance)/6;
    distance_min_5 = distance_min_4;
    distance_min_4 = distance_min_3;
    distance_min_3 = distance_min_2;
    distance_min_2 = distance_min_1;
    distance_min_1 = new_distance;
  }
}

// automatic/manual switch belom


// relay_1 = Nutrisi A
// relay_2 = Nutrisi B
// relay_3 = Air
// relay_4 = ??
void relaycontrol(float avg_distance, int measured_ppm, int ppm_setting, float a_b_ratio){
  // jadiin parameter fungsi
  // int ppm_setting = 1200; //misal
  int max_ppm_setting = ppm_setting + 100;
  int min_ppm_setting = ppm_setting - 200; //1000
  int time = 5000; //in miliseconds
  // rata-rata distance (moving average)
  // float a_b_ratio = nutrisi_a / nutrisi_b
  int max_distance = 40; //airnya tinggal dikit
  int min_distance = 20; //airnya udah penuh

  if ((avg_distance > max_distance) || (measured_ppm < min_ppm_setting)){
    // avg_distance ngelewatin batas max_distance, atau ppm ngelewatin batas minimal
    while (avg_distance > min_distance){
      // selama masih belom sampe batas 20 cm
      while (measured_ppm < max_ppm_setting){
        // selama masih belom sampe ppm 1300
        // cek perlu lebih banyak nutrisi A ato B
        if (a_b_ratio < 1){
          // a<b, nyalain b lebih lama
          set_gpio_status(relay_1, 1); //Nutrisi A
          set_gpio_status(relay_2, 1); //Nutrisi B

          delay(floor(time*a_b_ratio));
          set_gpio_status(relay_1, 0);//a

          delay(time - floor(time*a_b_ratio));
          // set_gpio_status(relay_2, 0); //b
        }
        else if (a_b_ratio > 1){
          // a>b, nyalain a lebih lama
          set_gpio_status(relay_1, 1); //Nutrisi A
          set_gpio_status(relay_2, 1); //Nutrisi B

          delay(floor(time/a_b_ratio));
          set_gpio_status(relay_2, 0); //b

          delay(time - floor(time/a_b_ratio));
          // set_gpio_status(relay_1, 0); //a
        }
        else{
          // a=b, nyalain bareng
          set_gpio_status(relay_1, 1); //Nutrisi A
          set_gpio_status(relay_2, 1); //Nutrisi B
        }
        // turn on nutrisi A B sampai melebihi target, pisah relay (2 relay)
        // pake fungsi set_gpio_status(int pin, boolean enabled)
      }
      // turn on valve (air) jadiin boolean 0 atau 1
      set_gpio_status(relay_1, 0); //a
      set_gpio_status(relay_2, 0); //b
      set_gpio_status(relay_3, 1); //air
    }
  }
  // turn off all relay
  set_gpio_status(relay_1, 0);
  set_gpio_status(relay_2, 0);
  set_gpio_status(relay_3, 0);
  // set_gpio_status(relay_4, 0);
  
// setiap 5 menit sekali?
}


void getNprintData(){
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
    
    // Delay before repeating measurement
    delay(100);

    
    //Getting data from sensor and send data to thingsboard
    sensors.requestTemperaturesByIndex(0);
    temperature = sensors.getTempCByIndex(0);     //get temperatureerature value from sensor

    //Getting PPM Value from TDS sensor
    analogValue = analogRead(TdsSensorPin);
    voltage = analogValue/4096*5.0;
    ecValue=(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*k_factor;
    ecValue25  =  ecValue / (1.0+0.02*(temperature-25.0));  //temperature compensation
    tdsValue = ecValue25 * 0.5;
    k = ecValue/(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage);


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
    lcd.print(tdsValue);
    
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
void on_message(const char* topic, byte* payload, unsigned int length) {

  Serial.println("On message");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(json);

  // Decode JSON request
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject((char*)json);

  if (!data.success())
  {
    Serial.println("parseObject() failed");
    return;
  }

  // Check request method
  String methodName = String((const char*)data["method"]);

  if (methodName.equals("getValue")) {
    // Reply with GPIO status
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");

  } else if (methodName.equals("setValue")) {
    // Update GPIO status and reply
    set_gpio_status(data["params"]["pin"], data["params"]["enabled"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
  }
}


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
  while(1){
    getNprintData();
    delay(500);
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
          
          lcd.setCursor(0,3);
          lcd.print("4.Back");
          
          delay(100);
          
          ok = digitalRead(pb_red);
          up = digitalRead(pb_yellow);
          down = digitalRead(pb_green);
          back = digitalRead(pb_black);
          
          
          if (ok == LOW) { delay(300); goto menu8; }
          if (up == LOW) { delay(300); goto menu9; }
          if (down == LOW) {  }
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
}
