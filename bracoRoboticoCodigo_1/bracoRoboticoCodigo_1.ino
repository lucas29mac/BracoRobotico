#include <ESP32Servo.h>

const int NUM_SERVOS = 7;
const int pinos[NUM_SERVOS] = {18, 19, 22, 23, 5, 25,26}; // GPIOs conectados aos servos
Servo servos[NUM_SERVOS];

void setup() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(pinos[i]);
    servos[i].write(40); // Posição inicial
  }
  delay(50); // Espera 1 segundo antes de começar o movimento
}

void loop() {

  for (int angulo = 0; angulo <= 60; angulo++) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i].write(angulo);
    }
    delay(100); // Movimento suave
  }

  delay(1000); // Pequena pausa

  
  for (int angulo = 60; angulo >= 0; angulo--) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i].write(angulo);
    }
    delay(100);
  }

  delay(1000); // Outra pausa
}
