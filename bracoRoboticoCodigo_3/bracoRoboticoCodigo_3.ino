#include <ESP32Servo.h>
#include <math.h>

// Servos da base
Servo servoE1A;
Servo servoE1B;

// Servos dos elos
Servo servoE2;  // Ombro
Servo servoE3;  // Cotovelo

// GPIOs
#define PIN_E1A 22
#define PIN_E1B 5
#define PIN_E2  26
#define PIN_E3  23

// Posições atuais
int posA = 0, posB = 0, posE2 = 90, posE3 = 90;

// Comprimento dos elos (em mm)
const float L1 = 100.0;
const float L2 = 100.0;

void setup() {
  Serial.begin(9600);
  Serial.println("Digite comando no formato: XY±xxx±yyy.");

  servoE1A.attach(PIN_E1A);
  servoE1B.attach(PIN_E1B);
  servoE2.attach(PIN_E2);
  servoE3.attach(PIN_E3);

  servoE1A.write(posA);
  servoE1B.write(posB);
  servoE2.write(posE2);
  servoE3.write(posE3);
}

void loop() {
  if (Serial.available() >= 11) {
    char cmd[12];
    for (int i = 0; i < 11; i++) {
      cmd[i] = Serial.read();
    }
    cmd[11] = '\0';

    if (cmd[0] == 'X' && cmd[1] == 'Y' && cmd[10] == '.') {
      int sinalX = (cmd[2] == '-') ? -1 : 1;
      int x = sinalX * ((cmd[3] - '0') * 100 + (cmd[4] - '0') * 10 + (cmd[5] - '0'));

      int sinalY = (cmd[6] == '-') ? -1 : 1;
      int y = sinalY * ((cmd[7] - '0') * 100 + (cmd[8] - '0') * 10 + (cmd[9] - '0'));

      // Ângulo da base
      float thetaBase = atan2(y, x) * 180.0 / PI;
      if (thetaBase < 0) thetaBase += 360;

      // Mover base com dois servos
      int alvoA, alvoB;
      if (thetaBase <= 180) {
        alvoA = thetaBase;
        alvoB = 0;
      } else {
        alvoA = 180;
        alvoB = thetaBase - 180;
      }

      // Validar alcance do braço
      float distancia = sqrt(x * x + y * y);
      if (distancia > (L1 + L2) || distancia < fabs(L1 - L2)) {
        Serial.println("Ponto fora do alcance do braço!");
        return;
      }

      // Cinemática inversa
      float D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
      float theta2 = acos(constrain(D, -1.0, 1.0));  // Cotovelo
      float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)); // Ombro

      // Conversão para graus
      int angE2 = round(theta1 * 180.0 / PI);  // Ombro
      int angE3 = round(theta2 * 180.0 / PI);  // Cotovelo

      // Ajustes de offset para os servos
      angE2 = constrain(90 + angE2, 0, 180);
      angE3 = constrain(angE3, 0, 180);

      Serial.print("X=");
      Serial.print(x);
      Serial.print(" Y=");
      Serial.print(y);
      Serial.print(" | Base=");
      Serial.print(thetaBase);
      Serial.print(" Ombro=");
      Serial.print(angE2);
      Serial.print(" Cotovelo=");
      Serial.println(angE3);

      // Movimento suave - Base
      while (posA != alvoA) {
        posA += (posA < alvoA) ? 1 : -1;
        servoE1A.write(posA);
        delay(5);
      }
      while (posB != alvoB) {
        posB += (posB < alvoB) ? 1 : -1;
        servoE1B.write(posB);
        delay(5);
      }

      // Movimento suave - Ombro
      while (posE2 != angE2) {
        posE2 += (posE2 < angE2) ? 1 : -1;
        servoE2.write(posE2);
        delay(5);
      }

      // Movimento suave - Cotovelo
      while (posE3 != angE3) {
        posE3 += (posE3 < angE3) ? 1 : -1;
        servoE3.write(posE3);
        delay(5);
      }

      // Limpa buffer serial
      while (Serial.available()) Serial.read();
    }
  }
}
