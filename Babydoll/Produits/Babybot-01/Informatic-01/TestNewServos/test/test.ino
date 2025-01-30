#include <Servo.h>

Servo myServo; // Create a Servo object

const int servoPin = 9; // Connect the servo signal pin to digital pin 9
const int delayTime = 15; // Delay time in milliseconds between updates

const int minPulseWidth = 500; // Minimum pulse width (in microseconds)
const int maxPulseWidth = 2500;

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

const int numSamples = 80; // 1 second worth of samples at 1ms intervals
float voltage[numSamples];
float current[numSamples];

void setup() {
  myServo.attach(servoPin, minPulseWidth, maxPulseWidth); // Attach the servo to the specified pin
  Serial.begin(9600); // Start serial communication for debugging
  Serial.println("Servo Test Initialized");

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  Serial.println("INA219 Initialized");
}

void loop() {


  myServo.write(0);

  // Serial print the results
  Serial.println("Voltage (V) | Current (mA)");
  for (int i = 0; i < numSamples; i++) {
    Serial.print(ina219.getBusVoltage_V(), 3); // 3 decimal places
    Serial.print(" V | ");
    Serial.print(ina219.getCurrent_mA(), 3); // 3 decimal places
    Serial.println(" mA");
  }
  myServo.write(180); 
    
  // Serial print the results
  Serial.println("Voltage (V) | Current (mA)");
  for (int i = 0; i < numSamples; i++) {
    Serial.print(ina219.getBusVoltage_V(), 3); // 3 decimal places
    Serial.print(" V | ");
    Serial.print(ina219.getCurrent_mA(), 3); // 3 decimal places
    Serial.println(" mA");
  }


}