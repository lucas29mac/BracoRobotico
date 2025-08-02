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

// Posições atuais dos servos
int posA = 90, posB = 0, posE2 = 90, posE3 = 90;

// Comprimento dos elos (em mm)
const float L1 = 100.0; // Comprimento do elo do ombro
const float L2 = 100.0; // Comprimento do elo do cotovelo

void setup() {
  Serial.begin(9600);
  Serial.println("Digite comando no formato: XYZ[+-]xxx[+-]yyy[+-]zzz.");
  Serial.println("Exemplo: XYZ+150+050+100.");

  servoE1A.attach(PIN_E1A);
  servoE1B.attach(PIN_E1B);
  servoE2.attach(PIN_E2);
  servoE3.attach(PIN_E3);

  // Define uma posição inicial para os servos
  servoE1A.write(posA);
  servoE1B.write(posB);
  servoE2.write(posE2);
  servoE3.write(posE3);
}

void loop() {
  // Verifica se o comando completo (16 caracteres + '.') foi recebido
  if (Serial.available() >= 16) {
    char cmd[17]; // 16 caracteres + caractere nulo
    Serial.readBytes(cmd, 16);
    cmd[16] = '\0'; // Adiciona o terminador de string

    // Valida o cabeçalho e o final do comando
    if (strncmp(cmd, "XYZ", 3) == 0 && cmd[15] == '.') {
      
      // Extrai as coordenadas X, Y, Z do comando
      int x = ((cmd[3] == '-') ? -1 : 1) * ((cmd[4] - '0') * 100 + (cmd[5] - '0') * 10 + (cmd[6] - '0'));
      int y = ((cmd[7] == '-') ? -1 : 1) * ((cmd[8] - '0') * 100 + (cmd[9] - '0') * 10 + (cmd[10] - '0'));
      int z = ((cmd[11] == '-') ? -1 : 1) * ((cmd[12] - '0') * 100 + (cmd[13] - '0') * 10 + (cmd[14] - '0'));

      // --- INÍCIO DA CINEMÁTICA INVERSA ---

      // 1. CÁLCULO DO ÂNGULO DA BASE (Servos E1A e E1B)
      // Usa as coordenadas X e Y para encontrar o ângulo de rotação no plano horizontal.
      float thetaBase = atan2(y, x) * 180.0 / PI;
      if (thetaBase < 0) {
        thetaBase += 360; // Garante que o ângulo esteja entre 0 e 360
      }

      // Mapeia o ângulo da base (0-360) para os dois servos que trabalham em conjunto (0-180 cada)
      int alvoA, alvoB;
      if (thetaBase <= 180) {
        alvoA = round(thetaBase);
        alvoB = 0;
      } else {
        alvoA = 180;
        alvoB = round(thetaBase - 180);
      }

      // 2. CÁLCULO DOS ÂNGULOS DO BRAÇO (Servos E2 e E3)
      // O braço opera em um plano vertical. A "distância horizontal" nesse plano é 'r'.
      float r = sqrt(pow(x, 2) + pow(y, 2));

      // A distância direta do eixo do ombro (origem) até a ponta da garra é 'd'.
      // Esta é a hipotenusa no plano vertical (r, z).
      float d = sqrt(pow(r, 2) + pow(z, 2));

      // Verificação de alcance: a distância 'd' pode ser alcançada pelos elos L1 e L2?
      if (d > (L1 + L2) || d < fabs(L1 - L2)) {
        Serial.println("ERRO: Ponto fora do alcance do braco!");
        while (Serial.available()) Serial.read(); // Limpa o buffer de entrada
        return; // Pula o resto do loop
      }

      // Cálculo do ângulo do Cotovelo (theta2) usando a Lei dos Cossenos no triângulo (L1, L2, d)
      float cosTheta2 = (pow(d, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
      cosTheta2 = constrain(cosTheta2, -1.0, 1.0); // Garante que o valor esteja no domínio de acos()
      float theta2_rad = acos(cosTheta2); // Ângulo do cotovelo em radianos

      // Cálculo do ângulo do Ombro (theta1)
      float alpha = atan2(z, r);
      float beta = atan2(L2 * sin(theta2_rad), L1 + L2 * cos(theta2_rad));
      float theta1_rad = alpha - beta; // Ângulo do ombro em radianos (relativo à horizontal)

      // Conversão dos ângulos (radianos) para graus de servo (0-180)
      // Ombro (E2): Mapeia o ângulo theta1 para o servo. 
      // Assumimos que 90 graus no servo corresponde ao braço na horizontal (theta1 = 0).
      // Se o braço sobe (theta1 > 0), o ângulo do servo diminui.
      int angE2 = constrain(round(90.0 - (theta1_rad * 180.0 / PI)), 0, 180);
      
      // Cotovelo (E3): Mapeia o ângulo theta2 diretamente.
      // Geralmente 0 é o braço dobrado e 180 esticado.
      int angE3 = constrain(round(theta2_rad * 180.0 / PI), 0, 180);

      // Imprime os valores calculados para depuração
      Serial.print("X="); Serial.print(x);
      Serial.print(" Y="); Serial.print(y);
      Serial.print(" Z="); Serial.print(z);
      Serial.print(" | Alvos: Base(A)="); Serial.print(alvoA);
      Serial.print(" Base(B)="); Serial.print(alvoB);
      Serial.print(" Ombro="); Serial.print(angE2);
      Serial.print(" Cotovelo="); Serial.println(angE3);

      // --- MOVIMENTO SUAVE DOS SERVOS ---
      
      // Movimento simultâneo (aproximado) de todos os servos até seus alvos
      while (posA != alvoA || posB != alvoB || posE2 != angE2 || posE3 != angE3) {
        if (posA != alvoA) {
          posA += (posA < alvoA) ? 1 : -1;
          servoE1A.write(posA);
        }
        if (posB != alvoB) {
          posB += (posB < alvoB) ? 1 : -1;
          servoE1B.write(posB);
        }
        if (posE2 != angE2) {
          posE2 += (posE2 < angE2) ? 1 : -1;
          servoE2.write(posE2);
        }
        if (posE3 != angE3) {
          posE3 += (posE3 < angE3) ? 1 : -1;
          servoE3.write(posE3);
        }
        delay(15); // Delay para controlar a velocidade do movimento
      }
      
      while (Serial.available()) Serial.read(); // Limpa o buffer de entrada após o comando ser executado
    }
  }
}