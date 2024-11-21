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

    // read command
    String result;
    unsigned long startTime = millis();
    while(millis() - startTime < 10){
      if(hc06.available()){
        char c = hc06.read();
        result += c;
        startTime = millis();
      }
    }

    Serial.print(result);


    // decode command
    if(result.startsWith("red0")){
      digitalWrite(RED,1);
    }
    else if(result.startsWith("red1")){
      digitalWrite(RED,0);
    }
    else if(result.startsWith("green0")){
      digitalWrite(GREEN,1);
    }
    else if(result.startsWith("green1")){
      digitalWrite(GREEN,0);
    }
    else if(result.startsWith("blue0")){
      digitalWrite(BLUE,1);
    }
    else if(result.startsWith("blue1")){
      digitalWrite(BLUE,0);
    }
    else{
      Serial.print("Command unknown");
    }
 }
  
  //Write from Serial Monitor to HC06
  if (Serial.available()) {
   hc06.write(Serial.read());
 }
}
String command(const char *toSend, unsigned long milliseconds) {
  String result;
  Serial.print("Sending: ");
  Serial.println(toSend);
  hc06.print(toSend);
  unsigned long startTime = millis();
  Serial.print(F("Received: "));
  while (millis() - startTime < milliseconds) {
    if (hc06.available()) {
      char c = hc06.read();
      Serial.write(c);
      result += c;  // append to the result string
    }
  }
  Serial.println();  // new line after timeout.
  return result;
}
