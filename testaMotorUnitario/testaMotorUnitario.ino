#include <ESP32Servo.h>

Servo servoBase;  // Servo individual

void setup() {
  servoBase.attach(26);     
  servoBase.write(0);      
  
}

void loop() {
  
}


