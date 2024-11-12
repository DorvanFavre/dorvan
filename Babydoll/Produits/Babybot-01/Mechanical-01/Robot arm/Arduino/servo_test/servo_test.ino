#include <Servo.h>

// Déclaration des servos
Servo servo1;
Servo servo2;
Servo servo3;

// Variables pour stocker la commande
String inputString = "";  // Chaîne reçue
bool stringComplete = false;  // Si la chaîne est complète

void setup() {
  // Initialisation des servos sur les pins correspondantes
  servo1.attach(9);   // Pin du servo 1
  servo2.attach(10);  // Pin du servo 2
  servo3.attach(11);  // Pin du servo 3
  
  // Initialisation de la communication série
  Serial.begin(9600);
  inputString.reserve(10);  // Réserve de la mémoire pour la chaîne de commande
}

void loop() {
  // Lorsque la chaîne est complète, la traiter
  if (stringComplete) {
    // Extrait le numéro du servo et l'angle
    int servoNum = inputString.charAt(1) - '0';  // Extrait le numéro du servo
    int angle = inputString.substring(3).toInt();  // Extrait l'angle
    
    // Sélectionne le bon servo et positionne-le à l'angle donné
    if (servoNum == 1) {
      servo1.write(angle);
      Serial.println("Servo 1 à l'angle : " + String(angle));
    } else if (servoNum == 2) {
      servo2.write(angle);
      Serial.println("Servo 2 à l'angle : " + String(angle));
    } else if (servoNum == 3) {
      servo3.write(angle);
      Serial.println("Servo 3 à l'angle : " + String(angle));
    } else {
      Serial.println("Numéro de servo invalide !");
    }

    // Réinitialise la chaîne
    inputString = "";
    stringComplete = false;
  }
}

// Fonction appelée chaque fois qu'un caractère est reçu par la console série
void serialEvent() {
  while (Serial.available()) {
    // Lit le caractère
    char inChar = (char)Serial.read();
    
    // Ajoute le caractère à la chaîne
    inputString += inChar;
    
    // Si un caractère de nouvelle ligne est reçu, la chaîne est complète
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
