#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 103  // Minimum pulse length count for 0° (adjust as needed)
#define SERVOMAX 512  // Maximum pulse length count for 180° (adjust as needed)

// Servo channels
#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 1
#define SERVO3_CHANNEL 2

const int oePin = 7;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Set the PWM frequency to 60 Hz for servos
  pinMode(oePin, OUTPUT);
  digitalWrite(oePin, LOW); // Start with outputs enabled
  Serial.println("Enter commands in format: <servo number> <angle>");
  Serial.println("Type 'enable' or 'disable' to control the OE pin");
}

void loop(){
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra whitespace
    
    if (input.equalsIgnoreCase("enable")) {
      digitalWrite(oePin, LOW); // Enable outputs
      Serial.println("Outputs enabled.");
    } else if (input.equalsIgnoreCase("disable")) {
      digitalWrite(oePin, HIGH); // Disable outputs
      Serial.println("Outputs disabled.");
    } else {
      // Parse servo number and angle from the input
      int servoNumber, angle;
      if (sscanf(input.c_str(), "%d %d", &servoNumber, &angle) == 2) {
        if (servoNumber >= 1 && servoNumber <= 3 && angle >= 0 && angle <= 180) {
          setServoAngle(servoNumber, angle);
          Serial.print("Set Servo ");
          Serial.print(servoNumber);
          Serial.print(" to ");
          Serial.print(angle);
          Serial.println(" degrees.");
        } else {
          Serial.println("Error: Servo number must be 1-3 and angle 0-180.");
        }
      } else {
        Serial.println("Invalid command. Format: <servo number> <angle>");
      }
    }
  }
}

// Function to set servo angle on a specified servo
void setServoAngle(int servoNumber, int angle) {
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  // Map servo number to channel
  int channel;
  switch (servoNumber) {
    case 1:
      channel = SERVO1_CHANNEL;
      break;
    case 2:
      channel = SERVO2_CHANNEL;
      break;
    case 3:
      channel = SERVO3_CHANNEL;
      break;
    default:
      return; // Invalid servo number
  }
  
  pwm.setPWM(channel, 0, pulseLength);
}
