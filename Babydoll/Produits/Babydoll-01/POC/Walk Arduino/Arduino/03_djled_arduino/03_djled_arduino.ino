#include <SoftwareSerial.h>

#define RED 9
#define GREEN 10
#define BLUE 11

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial hc06(rxPin,txPin);

void setup() {

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  
  //Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("ENTER AT Commands:");
  //Initialize Bluetooth Serial Port
  hc06.begin(9600);
}

void loop() {
  //Write data from HC06 to Serial Monitor
  if (hc06.available()) {
    unsigned long top = micros();

    // read command
    String result;
    unsigned long startTime = millis();
    while(millis() - startTime < 3){
      if(hc06.available()){
        char c = hc06.read();
        result += c;
        startTime = millis();
      }
    }

    //Serial.println(micros() - top);
    // Decode message
    int r = 255 - result.substring(0,3).toInt();
    int g = 255 - result.substring(3,6).toInt();
    int b = 255 - result.substring(6,9).toInt();

    Serial.print(r) ;
    Serial.print(' ');
    Serial.print(g) ;
    Serial.print(' ');
    Serial.print(b) ;
    Serial.println(' ');


    // Light LEDs
    analogWrite(RED,r);
    analogWrite(GREEN,g);
    analogWrite(BLUE,b);
    
 }
}
