#include <ESP32Servo.h>

Servo servoBase;  // Servo individual

void setup() {
  servoBase.attach(5);     // Teste com o GPIO 18 (mude se precisar)
  servoBase.write(0);      // Posição inicial
  
}

void loop() {
  
}
