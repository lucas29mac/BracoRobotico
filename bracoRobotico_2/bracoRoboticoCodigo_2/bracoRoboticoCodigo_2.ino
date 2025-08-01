#include <ESP32Servo.h>
#include <math.h>

Servo servoE1A;  // Servo base A
Servo servoE1B;  // Servo base B

int posA = 0;  // Posição atual do servo A
int posB = 0;  // Posição atual do servo B

void setup() {
  Serial.begin(9600);
  Serial.println("Digite comando no formato XY±xxx±yyy.");
  servoE1A.attach(22);  // GPIO do servo A
  servoE1B.attach(5);   // GPIO do servo B

  servoE1A.write(posA);
  servoE1B.write(posB);
}

void loop() {
  if (Serial.available() >= 11) {
    char cmd[12];
    for (int i = 0; i < 11; i++) {
      cmd[i] = Serial.read();
    }
    cmd[11] = '\0';  // Termina a string

    if (cmd[0] == 'X' && cmd[1] == 'Y' && cmd[10] == '.') {
      int sinalX = (cmd[2] == '-') ? -1 : 1;
      int x = sinalX * ((cmd[3] - '0') * 100 + (cmd[4] - '0') * 10 + (cmd[5] - '0'));

      int sinalY = (cmd[6] == '-') ? -1 : 1;
      int y = sinalY * ((cmd[7] - '0') * 100 + (cmd[8] - '0') * 10 + (cmd[9] - '0'));

      float theta = atan2(y, x) * 180.0 / PI;
      if (theta < 0) theta += 360;

      Serial.print("Coordenadas: X=");
      Serial.print(x);
      Serial.print(" Y=");
      Serial.print(y);
      Serial.print(" | Ângulo = ");
      Serial.println(theta);

      // Define os alvos para os servos
      int alvoA, alvoB;
      if (theta <= 180) {
        alvoA = theta;
        alvoB = 0;
      } else {
        alvoA = 180;
        alvoB = theta - 180;
      }

      // Movimento suave Servo A
      while (posA != alvoA) {
        if (posA < alvoA) posA++;
        else if (posA > alvoA) posA--;
        servoE1A.write(posA);
        delay(10);
      }

      // Movimento suave Servo B
      while (posB != alvoB) {
        if (posB < alvoB) posB++;
        else if (posB > alvoB) posB--;
        servoE1B.write(posB);
        delay(10);
      }

      // Limpa qualquer byte restante no buffer serial
      while (Serial.available()) Serial.read();
    }
  }
}
