/*
 * This program is property of SME Dehradun. for any query related to this program, contact us at www.smedehradn.com
 * if your want any soluiton related for any IoT Customized Boards and Sensor to www.nuttyengineer.com 
 */
// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPL31iBXa6-x"
#define BLYNK_TEMPLATE_NAME "wireless device"
#define BLYNK_DEVICE_NAME "wireless device"
#define BLYNK_FIRMWARE_VERSION "0.1.0"
#define BLYNK_PRINT Serial
#define USE_NODE_MCU_BOARD
#include "BlynkEdgent.h"
//#include "DHT.h"
//#define DHTPIN D2 
//#define DHTTYPE DHT11


float t, h;
int value;
float value1 = random(33,37);
float value2 = random(7.5,8);

void sendSensor()
{
  //h = dht.readHumidity();
  //t = dht.readTemperature();  
  Blynk.virtualWrite(V0, value1);
  Blynk.virtualWrite(V1, value2);
  Blynk.virtualWrite(V2, value);

}

void setup()
{
  pinMode(A0,INPUT);
  //pinMode(D0, INPUT);
  //pinMode(D1, INPUT);
  Serial.begin(9600);
  //dht.begin();
  BlynkEdgent.begin();
  delay(2000); 
  timer.setInterval(1000L, sendSensor); 
}

void loop() 
{
  value=analogRead(A0);
  //value1=analogRead(D0);
  //value2=analogRead(D1);
  BlynkEdgent.run();
  timer.run(); // Initiates SimpleTimer
}
