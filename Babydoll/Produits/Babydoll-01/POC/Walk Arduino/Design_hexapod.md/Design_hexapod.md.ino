#include <Servo.h>

Servo myServo; // Create a Servo object
int potPin = A0; // Analog pin connected to the potentiometer

void setup() {
  myServo.attach(9); // Attach the servo to pin 9
}

void loop() {
  int potValue = analogRead(potPin); // Read the potentiometer value (0-1023)
  int angle = map(potValue, 0, 1023, 0, 180); // Map the value to a range of 0-180

  myServo.write(angle); // Set the servo position based on the potentiometer
  delay(15); // Small delay to smooth movement
}
