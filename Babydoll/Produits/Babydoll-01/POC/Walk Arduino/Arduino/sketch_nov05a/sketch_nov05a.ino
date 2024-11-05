

void setup() {
  //Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("ENTER AT Commands:");
  //Initialize Bluetooth Serial Port
  Serial1.begin(9600);
}

void loop() {
  //Write data from HC06 to Serial Monitor
  if (Serial1.available()) {
    Serial.write(Serial1.read());
 }
  
  //Write from Serial Monitor to HC06
  if (Serial.available()) {
   Serial1.write(Serial.read());
 }
}
