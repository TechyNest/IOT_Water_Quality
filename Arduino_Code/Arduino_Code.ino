#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>

#define TdsSensorPin A1
#define PhSensorPin 0 // Connect pH sensor to RX pin (pin 0)
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
#define ONE_WIRE_BUS 4 // Data wire connected to Arduino digital pin 4
#define HEATER_PIN 2 // Pin connected to the relay module for heater control
#define PUMP_PIN 3 // Pin connected to the relay module for oxygen pump control
#define SERVO_PIN 11 // Pin connected to the servo motor

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltageTDS = 0, tdsValue = 0, temperature = 25;
float averageVoltagePh = 0, phValue = 0;
unsigned long pumpTimer = 0;
unsigned long servoTimer = 0;
Servo myServo;

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 

void setup() {
  Serial.begin(9600);
  pinMode(TdsSensorPin, INPUT);
  pinMode(A2, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT); // Set heater control pin as output
  pinMode(PUMP_PIN, OUTPUT); // Set pump control pin as output
  myServo.attach(SERVO_PIN); // Attach servo to the specified pin
  sensors.begin(); // Start up the temperature sensor library
  
  // Initially turn off the pump and set servo to 0 degrees
  digitalWrite(PUMP_PIN, LOW);
  myServo.write(0);
}

void loop() {
  // TDS sensor code
  static unsigned long analogSampleTimepointTDS = millis();
  if (millis() - analogSampleTimepointTDS > 40U) {
    analogSampleTimepointTDS = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  static unsigned long printTimepointTDS = millis();
  if (millis() - printTimepointTDS > 800U) {
    printTimepointTDS = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltageTDS = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltageTDS / compensationCoefficient;
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
    analogWrite(A2, tdsValue);
    
    digitalWrite(9, phValue);
  }

  // pH sensor code
  if (Serial.available() > 0) {
    averageVoltagePh = Serial.parseFloat(); // Read pH value from Serial
    phValue = averageVoltagePh;
    Serial.print("pH Value:");
    Serial.println(phValue, 2);
  }

  // Temperature sensor code
  sensors.requestTemperatures(); // Request temperature readings from the sensor
  float currentTemperature = sensors.getTempCByIndex(0); // Get temperature in Celsius
  
  Serial.print("Temperature: ");
  Serial.print(currentTemperature);
  digitalWrite(8, currentTemperature); // Print current temperature
  
  // Check if temperature is below 35 degrees Celsius
  if (currentTemperature < 35) {
    digitalWrite(HEATER_PIN, HIGH); // Turn on the heater
    Serial.println(" Heater ON"); // Print status
  } else {
    digitalWrite(HEATER_PIN, LOW); // Turn off the heater
    Serial.println(" Heater OFF"); // Print status
  }

  // Oxygen pump control code
  unsigned long currentTime = millis();
  if (currentTime - pumpTimer >= 300000) { // If 5 minutes have elapsed
    if (digitalRead(PUMP_PIN) == LOW) { // If pump is currently off
      digitalWrite(PUMP_PIN, HIGH); // Turn on the pump
      Serial.println("Oxygen Pump ON");
    } else { // If pump is currently on
      digitalWrite(PUMP_PIN, LOW); // Turn off the pump
      Serial.println("Oxygen Pump OFF");
    }
    pumpTimer = currentTime; // Reset the timer
  }

  // Servo motor control code
  if (currentTime - servoTimer >= 43200000) { // If 12 hours have elapsed (12 hours = 12 * 60 * 60 * 1000 milliseconds)
    static int servoPosition = 0;
    if (servoPosition == 0) { // If servo is currently at 0 degrees
      myServo.write(180); // Rotate servo to 180 degrees
      servoPosition = 180;
      Serial.println("Servo at 180 degrees");
        } else { // If servo is currently at 180 degrees
      myServo.write(0); // Rotate servo to 0 degrees
      servoPosition = 0;
      Serial.println("Servo at 0 degrees");
    }
    servoTimer = currentTime; // Reset the timer
  }

  delay(1000);
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

