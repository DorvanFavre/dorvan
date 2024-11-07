#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MatrixUtils.h"
#include <math.h>

// Initialize the PCA9685 driver
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x60);

#define PWM_MIN 109  // Minimum pulse length count for 0° (adjust as needed)
// #define SERVOMAX 512  // Maximum pulse length count for 180° (adjust as needed)
#define PWM_MAX 525

// min : 74
// half = env.316
// max : 559

// - pi/2: 119
// 0 : 317
// pi/2 = 525


// Fix parameters
const int oePin = 7;
const int N = 18; // Number of servo
const float L1 = 111.44;
const float L2 = 43.1;
const float H2 = -21.7;
const float L3 = 80;
const float L4 = 126.46;
const float H4 = -64.24;
float a4;

const int SERVO_CHANNEL[N] = {13, 14, 15, 5, 6, 7, 0, 1, 2, 0, 0, 0, 0, 0, 0, 2, 1, 0};
const int SERVO_OFFSET[N] = {0, 0, 15, 9, -16, 15, 0, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, -26}; // impulse width
const int SERVO_SIGN[N] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1};
float servo_min[18];
float servo_max[18];

// config

const float servoPeriod = 1.0 / 30.0; //s
const float velocity = 1.0; // rad/s
const float servoStep = velocity * servoPeriod; // rad

const float stepLength = 80; // mm
const float stepHeight = 50; // mm
const float stepPeriod = 1.0; // s

// private
float targetServoAngle[N];
float actualServoAngle[N];
int mode = 0;
unsigned long startTime;
unsigned long servoTime;

void setup() {

  Serial.begin(9600);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  pinMode(oePin, OUTPUT);
  digitalWrite(oePin, HIGH); // Start with outputs disable
  Serial.println("Enter commands in format: <servo number> <angle>");
  Serial.println("Type 'enable' or 'disable' to control the OE pin");

  a4 = sqrt(L4 * L4 + H4 * H4);

  for ( int i = 0; i < 6; i++) {
    servo_min[i * 3] = -0.75;
    servo_max[i * 3] = 0.75;
    servo_min[i * 3 + 1] = -0.5;
    servo_max[i * 3 + 1] = PI / 2;
    servo_min[i * 3 + 2] = -PI / 2;
    servo_max[i * 3 + 2] = 0.65;
  }


}



void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra whitespace

    // Commands
    if (input.equalsIgnoreCase("enable")) {
      digitalWrite(oePin, LOW); // Enable outputs
      Serial.println("Outputs enabled.");
    } else if (input.equalsIgnoreCase("disable")) {
      digitalWrite(oePin, HIGH); // Disable outputs
      Serial.println("Outputs disabled.");

    } else if (input.equalsIgnoreCase("init")) {
      digitalWrite(oePin, LOW); // Enable outputs
      Serial.println("Outputs enable, init.");
      for (int i = 0; i < 6; i++) {
        actualServoAngle[i * 3] = targetServoAngle[i * 3] = 0.0;
        actualServoAngle[i * 3 + 1] = targetServoAngle[i * 3 + 1] = PI / 2;
        actualServoAngle[i * 3 + 2] = targetServoAngle[i * 3 + 2] = 0.65;
        writeServo(i * 3, actualServoAngle[i * 3]);
        writeServo(i * 3 + 1, actualServoAngle[i * 3 + 1]);
        writeServo(i * 3 + 2, actualServoAngle[i * 3 + 2]);
      }
      for (int i = 0; i < N; i++) {

      }

    } else if (input.equalsIgnoreCase("calibrate")) {
      mode = 1;
      Serial.println("Calibration mode, pwm1 channel 11. ");
    } else if (input.equalsIgnoreCase("ik")) {
      mode = 3;
      Serial.println("IK mode. ");
    }
    else if (input.equalsIgnoreCase("sit")) {
      Serial.println("Small");
      for (int i = 0; i < 6; i++) {
        setServoAngle(i * 3 + 1, 1.57);
        setServoAngle(i * 3 + 2, 0.65);
      }

    }
    else if (input.equalsIgnoreCase("single")) {
      mode = 2;
      Serial.println("Single mode");
    }
    else if (mode == 3) {
      // Parse servo number and angle from the input
      float x, y, z;
      if (sscanf(input.c_str(), "%f %f %f", &x, &y, &z) == 3) {

        Serial.print("Ik x: ");
        Serial.print(x);
        Serial.print(", y: ");
        Serial.print(y);
        Serial.print(", z: ");
        Serial.println(z);
        servo_ik(0, x, y, z);

      } else {
        Serial.println("Invalid command. Format: <x> <y> <z>");
      }
    }
    else if (mode == 1) {
      Serial.println("calibration: ");
      int pulse_width;

      if (sscanf(input.c_str(), "%d", &pulse_width) == 1) {

        //setServoAngle(servoNumber, angle);
        Serial.print("Set pulse width");
        Serial.print(pulse_width);
        pwm1.setPWM(11, 0, pulse_width);
      }
      else {
        Serial.println("Invalid command. Format: <pulse width> ");
      }
    }




    else if (mode == 2) {
      // Play with individual servo
      int servoNumber;
      float angle;
      if (sscanf(input.c_str(), "%d %f", &servoNumber, &angle) == 2) {
        if (servoNumber >= 0 && servoNumber <= N - 1 && angle >= -PI / 2 && angle <= PI / 2) {
          setServoAngle(servoNumber, angle);
          //setServoAngle(servoNumber, angle);
          Serial.print("Set Servo ");
          Serial.print(servoNumber);
          Serial.print(" to ");
          Serial.print(angle);
          Serial.println(" rads.");
        } else {
          Serial.println("Error: Servo number must be 0-17 and angle -1.57 - 1.57");
        }
      } else {
        Serial.println("Invalid command. Format: <servo number> <angle>");
      }
    }


  }


  // run servo
  if (millis() - servoTime > servoPeriod*1000) {
    servoTime = millis();
    for (int i = 0; i < N; i++) {
      float error = targetServoAngle[i] - actualServoAngle[i];
      if (abs(error) > 0.001 ) {
        actualServoAngle[i] += getSign(error) * min(servoStep, abs(error));
        writeServo(i, actualServoAngle[i]);
      }
    }
  }




}

// Function to set servo angle on a specified servo
void writeServo(int n, float angle) {

  if (angle < servo_min[n] || angle > servo_max[n]) {
    Serial.println("Error: angle exceed constraints limit!");
  }
  else {
    int pulseLength = (int)mapFloat(angle, (-PI / 2) * SERVO_SIGN[n], (PI / 2) * SERVO_SIGN[n], PWM_MIN, PWM_MAX) + SERVO_OFFSET[n];
    Serial.println(pulseLength); // DEBUG
    if (n >= 0 && n <= 8) {
      pwm1.setPWM(SERVO_CHANNEL[n], 0, pulseLength);
    }
    else if (n >= 9 && n <= 17) {
      pwm2.setPWM(SERVO_CHANNEL[n], 0, pulseLength);
    }
    else {
      Serial.println("Error, servo number must be in range 0- 17");
    }
  }

}

void setServoAngle(int n, float angle) {
  targetServoAngle[n] = angle;
}



float getSign(float number) {
  if (number > 0) {
    return 1.0;   // Positive
  } else if (number < 0) {
    return -1.0;  // Negative
  } else {
    return 0.0;   // Zero
  }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void servo_ik(int l, float x, float y, float z) {
  float theta_1, theta_2, theta_3;
  if (ik(x, y, z, theta_1, theta_2, theta_3)) {
    theta_3 = theta_3 - PI / 2;
    Serial.print("Theta 1: ");
    Serial.println(theta_1);
    Serial.print("Theta 2: ");
    Serial.println(theta_2);
    Serial.print("Theta 3: ");
    Serial.println(theta_3);
    setServoAngle(6 * l, theta_1);
    setServoAngle(6 * l + 1, theta_2);
    setServoAngle(6 * l + 2, theta_3);
  }

}

int ik(float x, float y, float z, float & theta_1_, float & theta_2_, float & theta_3_) {
  float o = x - L1;
  float theta_1 = atan(y / o);
  Serial.print("theta_1: ");
  Serial.println(theta_1);

  float p = sqrt(o * o + y * y);
  Serial.print("p: ");
  Serial.println(p);

  float q = p - L2;
  float z2 = z - H2;
  float r = sqrt(q * q + z2 * z2);
  Serial.print("r: ");
  Serial.println(r);

  float phi_1 = atan(z2 / q);
  Serial.print("phi_1: ");
  Serial.println(phi_1);

  Serial.print("a4: ");
  Serial.println(a4);
  Serial.print("L3: ");
  Serial.println(L3);
  float phi_2 = acos((a4 * a4 - L3 * L3 - r * r) / (-2 * r * L3));
  Serial.print("Phi_2: ");
  Serial.println(phi_2);
  float theta_2 = phi_1 + phi_2;
  Serial.print("theta_2: ");
  Serial.println(theta_2);

  float phi_3 = acos((r * r - a4 * a4 - L3 * L3) / (-2 * a4 * L3));
  float phi_4 = abs(atan(H4 / L4));
  Serial.print("Phi_3: ");
  Serial.println(phi_3);
  Serial.print("Phi_4: ");
  Serial.println(phi_4);

  float theta_3 = PI - phi_4 - phi_3;
  Serial.print("theta_3: ");
  Serial.println(theta_3);

  if (isnan(theta_1) || isnan(theta_2) || isnan(theta_3)) {
    Serial.print("Angle is NAN");
    return 0;
  } else {
    theta_1_ = theta_1;
    theta_2_ = theta_2;
    theta_3_ = theta_3;
    return 1;
  }


}
