/*
  Código refatorado para um braço robótico de 4 eixos (4 DOF) no ESP32.
  Esta versão utiliza DOIS motores na base para uma rotação completa de 360 graus.

  - Eixo 1: Base com servoBase1 e servoBase2 (Giro Horizontal 0-360)
  - Eixo 2: Ombro (servoOmbro)
  - Eixo 3: Cotovelo (servoCotovelo)
  - Eixo 4: Pulso (comentado)

  O controle é feito via Serial com o comando: XYZ[+-]xxx[+-]yyy[+-]zzz.
  Exemplo: XYZ+000+000+050.
*/

#include <ESP32Servo.h>
#include <math.h>

// --- Configuração dos Servos e Pinos ---
Servo servoBase1;
Servo servoBase2;
Servo servoOmbro;
Servo servoCotovelo;

#define PIN_SERVO_BASE1     22
#define PIN_SERVO_BASE2     5
#define PIN_SERVO_OMBRO     23
#define PIN_SERVO_COTOVELO  26

const float comprimento_braco = 100.0;     // L1
const float comprimento_antebraco = 100.0; // L2
const float comprimento_punho = 10.0;

int posBase1 = 0, posBase2 = 0;
int posOmbro = 90, posCotovelo = 90;

void setup() {
  Serial.begin(9600);
  Serial.println("--- Braço Robótico 4-DOF (Base Dupla) Inicializado ---");
  Serial.println("Envie coordenadas no formato: XYZ+120+080+050.");

  servoBase1.attach(PIN_SERVO_BASE1);
  servoBase2.attach(PIN_SERVO_BASE2);
  servoOmbro.attach(PIN_SERVO_OMBRO);
  servoCotovelo.attach(PIN_SERVO_COTOVELO);

  resetServos();
}

void loop() {
  if (Serial.available() >= 16) {
    char cmd[17];
    Serial.readBytes(cmd, 16);
    cmd[16] = '\0';

    if (strncmp(cmd, "XYZ", 3) == 0 && cmd[15] == '.') {
      float targetX = ((cmd[3] == '-') ? -1 : 1) * ((cmd[4] - '0') * 100 + (cmd[5] - '0') * 10 + (cmd[6] - '0'));
      float targetY = ((cmd[7] == '-') ? -1 : 1) * ((cmd[8] - '0') * 100 + (cmd[9] - '0') * 10 + (cmd[10] - '0'));
      float targetZ = ((cmd[11] == '-') ? -1 : 1) * ((cmd[12] - '0') * 100 + (cmd[13] - '0') * 10 + (cmd[14] - '0'));

      Serial.print("Alvo Recebido: X="); Serial.print(targetX);
      Serial.print(" Y="); Serial.print(targetY);
      Serial.print(" Z="); Serial.println(targetZ);

      calculateAndMove(targetX, targetY, targetZ);
    }

    while (Serial.available()) Serial.read();
  }
}

void calculateAndMove(float x, float y, float z) {
  float r = sqrt(x * x + y * y);
  int targetBase1, targetBase2;


  // Se a distância horizontal for muito pequena (quase zero), mantém os motores da base nas posições atuais
  if (r < 1.0) {
      targetBase1 = posBase1;
      targetBase2 = posBase2;
  } else {
      // Calcula o ângulo theta no plano XY com base nas coordenadas X e Y (ângulo da base em graus)
      float theta = atan2(y, x) * 180.0 / PI;

      // Garante que o ângulo esteja sempre entre 0 e 360 graus
      if (theta < 0) theta += 360;

      // Se o ângulo estiver entre 0 e 180 graus, usa apenas o servoBase1
      if (theta <= 180) {
        targetBase1 = round(theta);
        targetBase2 = 0;
      } else {
        // Se o ângulo estiver entre 180 e 360 graus, mantém servoBase1 em 180 e usa servoBase2 para o restante
        targetBase1 = 180;
        targetBase2 = round(theta - 180);
      }
  }




  float anguloOmbro, anguloCotovelo, anguloPulsoInterno;

  float distancia = sqrt(r * r + z * z);

  if (distancia > (comprimento_braco + comprimento_antebraco) || distancia < fabs(comprimento_braco - comprimento_antebraco)) {
    Serial.println("ERRO: Ponto fora do alcance!");
    return;
  }

  // Use a lógica do código anterior para determinar qual função de cálculo chamar
  // Para simplificar, vamos usar Calc_Point_Adapted para todos os casos por enquanto,
  // já que a Special_Calc_Point_Adapted é para um caso específico (x1, y1 < comprimento_braco)
  // e a Calc_Point_Adapted lida com a lógica de ponto fora do alcance e chama Calc_Circle_Adapted.
  Calc_Point_Adapted(r, z, anguloOmbro, anguloCotovelo, anguloPulsoInterno);

  int targetOmbro = constrain(round(anguloOmbro), 0, 180);
  int targetCotovelo = constrain(round(anguloCotovelo), 0, 180);
  Serial.print(" -> Ângulos Calculados: Base1="); Serial.print(targetBase1);
  Serial.print(" Base2="); Serial.print(targetBase2);
  Serial.print("angulo Ombro="); Serial.print(anguloOmbro);
  Serial.print("angulo Cotovelo="); Serial.print(anguloCotovelo);
  Serial.print("angulo Pulso Interno="); Serial.println(anguloPulsoInterno);
  Serial.print("Distancia d="); Serial.println(distancia);
  Serial.print("target Ombro="); Serial.print(targetOmbro);
  Serial.print("target Cotovelo="); Serial.println(targetCotovelo);

  moveServosSmooth(targetBase1, targetBase2, targetOmbro, targetCotovelo);


}

void moveServosSmooth(int tBase1, int tBase2, int tOmbro, int tCotovelo) {
  unsigned long startTime = millis();

  while (posBase1 != tBase1 || posBase2 != tBase2 || posOmbro != tOmbro || posCotovelo != tCotovelo) {
    if (millis() - startTime > 15) {
      startTime = millis();

      if (posBase1 != tBase1) {
        posBase1 += (posBase1 < tBase1) ? 1 : -1;
        servoBase1.write(posBase1);
      }

      if (posBase2 != tBase2) {
        posBase2 += (posBase2 < tBase2) ? 1 : -1;
        servoBase2.write(posBase2);
      }

      if (posOmbro != tOmbro) {
        posOmbro += (posOmbro < tOmbro) ? 1 : -1;
        servoOmbro.write(posOmbro);
      }

      if (posCotovelo != tCotovelo) {
        posCotovelo += (posCotovelo < tCotovelo) ? 1 : -1;
        servoCotovelo.write(posCotovelo);
      }
    }
  }

  Serial.println("Movimento concluído.");
}

void resetServos() {
  Serial.println("Resetando servos para posição inicial...");
  moveServosSmooth(0, 0, 90, 90);
  delay(1000);
}





// Funções de cálculo de ângulos (adaptadas do código anterior)
void Special_Calc_Point_Adapted(float x1, float y1, float& angulo_ombro, float& angulo_cotovelo, float& angulo_punho) {
  float Hypot;
  float A, B, C;

  Hypot = sqrt(sq(x1) + sq(y1)) + comprimento_punho;
  A = comprimento_antebraco;
  B = Hypot;
  C = comprimento_braco;

  angulo_ombro = acos((sq(B) + sq(C) - sq(A)) / (2 * B * C)) * (180 / PI);
  angulo_cotovelo =  acos((sq(C) + sq(A) - sq(B)) / (2 * A * C)) * (180 / PI);
  angulo_punho = acos((sq(A) + sq(B) - sq(C)) / (2 * A * B)) * (180 / PI);

  angulo_ombro = angulo_ombro * (1 + (angulo_ombro / (angulo_cotovelo + angulo_punho)));
  if (angulo_ombro < 35) {
    angulo_ombro = 35;
  }
}

void Calc_Circle_Adapted(float BXValue, float BYValue, float& angulo_ombro, float& angulo_cotovelo, float& angulo_punho) {
  float Hypot;
  float A, B, C;

  Hypot = sqrt(sq(BXValue) + sq(BYValue));
  A = comprimento_antebraco;
  B = Hypot;
  C = comprimento_braco;

  angulo_cotovelo =  acos((sq(C) + sq(A) - sq(B)) / (2 * A * C)) * (180 / PI);
  angulo_punho = acos((sq(A) + sq(B) - sq(C)) / (2 * A * B)) * (180 / PI);
  angulo_punho = 180 - angulo_punho;

  angulo_ombro = acos((sq(B) + sq(C) - sq(A)) / (2 * B * C)) * (180 / PI);

  angulo_ombro = angulo_ombro * (1 + (angulo_ombro / (angulo_cotovelo + angulo_punho)));

  if (angulo_ombro < 35) {
    angulo_ombro = 35;
  }
}

void Calc_Point_Adapted(float x1, float y1, float& angulo_ombro, float& angulo_cotovelo, float& angulo_punho) {
  float Slope;
  float A_quad, B_quad, C_quad; // Renomeado para evitar conflito com A, B, C de cinemática inversa
  float NegXAnswer, NegYAnswer, PosXAnswer, PosYAnswer;

  // Equação para Slope
  Slope = (0.0 - y1) / (0.0 - x1); // Assuming x2=0.0, y2=0.0 from previous code

  // Equações para A, B, C da equação quadrática
  A_quad = 1 + sq(y1 / x1);
  B_quad = (-2 * x1) + (-2 * y1 * Slope);
  C_quad = sq(y1) + sq(x1) - sq(comprimento_punho);

  NegXAnswer = ((-1 * B_quad) - (sqrt(sq(B_quad) - (4 * A_quad * C_quad)))) / (2 * A_quad);
  NegYAnswer = Slope * NegXAnswer;

  if (!isnan(NegXAnswer)) {
    Calc_Circle_Adapted(NegXAnswer, NegYAnswer, angulo_ombro, angulo_cotovelo, angulo_punho);
  } else {
    PosXAnswer = ((-1 * B_quad) + (sqrt(sq(B_quad) - (4 * A_quad * C_quad)))) / (2 * A_quad);
    PosYAnswer = Slope * PosXAnswer;
    if (!isnan(PosXAnswer)) {
      Calc_Circle_Adapted(PosXAnswer, PosYAnswer, angulo_ombro, angulo_cotovelo, angulo_punho);
    }
  }
}

