#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Kinematics.h"
#include "MatrixUtils.h"

// Initialize the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 103  // Minimum pulse length count for 0° (adjust as needed)
#define SERVOMAX 512  // Maximum pulse length count for 180° (adjust as needed)

// Servo channels
#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 1
#define SERVO3_CHANNEL 2
const int oePin = 7;
const int numServo = 3;
const int N = numServo;

// config
const float f = 30;
const float T = 1/f;
const float velocity = 30; // deg/s
const float servoStep = velocity*T;

// private
float targetServoAngle[numServo];
float actualServoAngle[numServo];

Kinematics kin(N);
MatrixUtils mat_utils;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Set the PWM frequency to 60 Hz for servos
  pinMode(oePin, OUTPUT);
  digitalWrite(oePin, HIGH); // Start with outputs disable
  Serial.println("Enter commands in format: <servo number> <angle>");
  Serial.println("Type 'enable' or 'disable' to control the OE pin");

  // Kinematic


  kin.add_joint_axis(0, 0,  1,  111.4, 0,    0);
  kin.add_joint_axis(0, 1,  0,  43.1, 0,    -21.7);
  kin.add_joint_axis(0, 1,  0, 80, 0, 0);

  
  kin.add_initial_end_effector_pose( 1, 0,  0, 361,
                                     0, 1,  0, 2,
                                     0, 0, 1, -86,
                                     0, 0,  0, 1);
  
}



void loop(){
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
      for(int i=0;i<numServo;i++){
        actualServoAngle[i]=90.0;
        targetServoAngle[i]=90.0;
        setServoAngle(i, actualServoAngle[i]);
      }
    }
     else if (input.equalsIgnoreCase("ik")) {
      Serial.println("Invert kinematic");

      float desired_transform[4][4] = {
        {1, 0,  0,     360},
        {0, 1,  0,      2},
        {0, 0, 1,     -86},
        {0, 0,  0,      1}
      };
  
      float jac[6][N];
      float jac_t[6][N];
      float AA_t[6][6];
      float A_tA[N][N];
      float pinv[N][6];
  
      float joint_angles_0[N] = {0, 0, 0};
      float joint_angles[N];
  
      kin.inverse((float*)desired_transform, (float*)jac, (float*)pinv, (float*)jac_t, (float*)AA_t, (float*)A_tA, joint_angles_0, 0.01, 0.001, 20, joint_angles);
      mat_utils.print_matrix(joint_angles, 1, N, "Joint angles");

      
    
    } else {
      // Parse servo number and angle from the input
      int servoNumber, angle;
      if (sscanf(input.c_str(), "%d %d", &servoNumber, &angle) == 2) {
        if (servoNumber >= 1 && servoNumber <= 3 && angle >= 10 && angle <= 170) {
          targetServoAngle[servoNumber-1] = angle;
          //setServoAngle(servoNumber, angle);
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
 

  // run servo
  for(int i =0;i<numServo;i++){
    float error = targetServoAngle[i] - actualServoAngle[i];
    if(abs(error) > 0.1 ){
      actualServoAngle[i] += getSign(error) * min(servoStep, abs(error));
      setServoAngle(i,actualServoAngle[i]);
    }
  }



  delay(T*1000);
}

// Function to set servo angle on a specified servo
void setServoAngle(int channel, float angle) {
  int pulseLength = map(angle, 10, 170, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulseLength);
}


int getSign(int number) {
    if (number > 0) {
        return 1;   // Positive
    } else if (number < 0) {
        return -1;  // Negative
    } else {
        return 0;   // Zero
    }
}
