#include "GravityTDS.h"
#include <OneWire.h> 
#include <DallasTemperature.h>
 
#define ONE_WIRE_BUS 7
#define TdsSensorPin A1
 
OneWire oneWire(ONE_WIRE_BUS); 
GravityTDS gravityTds;
 
DallasTemperature sensors(&oneWire);
 
float tdsValue = 0;
 
void setup()
{
    Serial.begin(115200);
    sensors.begin();
    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on ESP
    gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
    gravityTds.begin();  //initialization
}
 
void loop()
{
    sensors.requestTemperatures();
 
    gravityTds.setTemperature(sensors.getTempCByIndex(0));  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
    //ecValue = gravityTds.getEcValue();

    //TDS serial print
    Serial.print("PPM: ");
    Serial.print(tdsValue,2);

    //TDS serial print
    /*Serial.print("EC: ");
    Serial.print(ecValue,2);
    */
    
    //Temp serial print
    Serial.print("Temperature is: "); 
    Serial.print(sensors.getTempCByIndex(0));
    
    delay(1500);
}
