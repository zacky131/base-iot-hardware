///////////////////////////////////// LIBRARY
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

#include <EEPROM.h>

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
#define TOKEN "K3ZLw3PpqpcbKodz2xhg"


//define relay pin
#define relay_1 18 //13
#define relay_2 19 //12
#define relay_3 23 //11
#define relay_4 5  //10

// Define distance sensor connnection
#define TRIGPIN 12 //8
#define ECHOPIN 13 //9

// Define connection for DFRobot Sensor
#define ONE_WIRE_BUS 14 //7
#define TdsSensorPin 39 //A5

#define WiFi_LED_ON 27 //6

#define PH_PIN 36 //A4

// Define DHT sensor connection
#define DHTPIN 16 //5
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// Floats to calculate distance
float duration, distance;
// Variable for hydroponic sensor 
float tdsValue, temperature, voltage, phVal;

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
  sensors.begin();
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
  
  // Set output mode for all GPIO pins
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(WiFi_LED_ON, OUTPUT);

  digitalWrite(relay_1, HIGH);
  digitalWrite(relay_2, HIGH);
  digitalWrite(relay_3, HIGH);
  digitalWrite(relay_4, HIGH);
  
  delay(10);
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  client.setCallback(on_message);

  dht.begin();

  //interupt
  periodicTicker.attach_ms(5000, getNsend_data);
}
///////////////////////////////////////////////////////// LOOP ///////////////////////////////////////
void loop() {
  if (!client.connected()) {
    reconnect();
  }
    client.loop();
}
//
void getNsend_data(){
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
    // Divide by 1000 as we want millimeters
   
    distance = (duration / 2) * 0.343;
    
    // Delay before repeating measurement
    delay(100);
  
    //Getting data from sensor and send data to thingsboard
    sensors.requestTemperaturesByIndex(0);
    temperature = sensors.getTempCByIndex(0);     //get temperatureerature value from sensor
    Serial.println(temperature);
    gravityTds.setTemperature(sensors.getTempCByIndex(0));  // set the temperatureerature and execute temperatureerature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
      
    voltage = analogRead(PH_PIN)/4096.0*5000; //read ph sensor voltage
    phVal = ph.readPH(voltage, temperature);         //convert voltage to ph with temperatureerature compensation
  
    ph.calibration(voltage, temperature);              //calibration process by Serial Monitor
  
    // DHT sensor read
    float t = dht.readTemperature();
    float h = dht.readHumidity();
  
    
    String packet = "";
    packet.concat(("{\"temperature\":"));
    packet.concat(temperature);
        
    packet.concat((",\"TDS\":"));
    packet.concat(tdsValue);
       
    packet.concat((",\"PH\":"));
    packet.concat(phVal);
  
    packet.concat((",\"Distance\":"));
    packet.concat(distance);
  
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
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  Serial.println("Connecting to AP ...");
  // attemperaturet to connect to WiFi network

  bool res;
  res = wm.autoConnect("ESPWiFiManager");
  while (!res) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  digitalWrite(WiFi_LED_ON, HIGH);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard node ...");
    // Attemperaturet to connect (clientId, username, password)
    if (client.connect("ESP8266 Device", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      digitalWrite(WiFi_LED_ON, HIGH);
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      digitalWrite(WiFi_LED_ON, LOW);
    }
  }
}
