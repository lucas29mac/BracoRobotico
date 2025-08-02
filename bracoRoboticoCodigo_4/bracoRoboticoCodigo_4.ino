/*
  Código refatorado para um braço robótico de 4 eixos (4 DOF) no ESP32.
  Esta versão utiliza DOIS motores na base para uma rotação completa de 360 graus.

  - Eixo 1: Base com servoBase1 e servoBase2 (Giro Horizontal 0-360)
  - Eixo 2: Ombro (servoOmbro)
  - Eixo 3: Cotovelo (servoCotovelo)
  - Eixo 4: Pulso (comentado)

  O controle é feito via Serial com o comando: XYZ[+-]xxx[+-]yyy[+-]zzz.
  Exemplo: XYZ+100+000+050.
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

// Comprimento dos elos do braço em mm
const float braco = 100.0;     // L1 (Ombro ao Cotovelo)
const float antibraco = 100.0; // L2 (Cotovelo ao Pulso)

// Posições atuais dos servos (em graus)
int posBase1 = 0, posBase2 = 0;
int posOmbro = 0, posCotovelo = 0;

// Declaração das funções
void calculateAndMove(float x, float y, float z);
void moveServosSmooth(int tBase1, int tBase2, int ombro, int cotovelo);
void resetServos();

void setup() {
  Serial.begin(9600);
  Serial.println("--- Braço Robótico 4-DOF (Base Dupla) Inicializado ---");
  Serial.println("Envie coordenadas no formato: XYZ+120+080+050.");

  // Anexa os servos aos pinos
  servoBase1.attach(PIN_SERVO_BASE1);
  servoBase2.attach(PIN_SERVO_BASE2);
  servoOmbro.attach(PIN_SERVO_OMBRO);
  servoCotovelo.attach(PIN_SERVO_COTOVELO);

  // Move os servos para a posição inicial
  resetServos();
}

void loop() {
  // Verifica se há dados suficientes na Serial
  if (Serial.available() >= 16) {
    char cmd[17];
    Serial.readBytes(cmd, 16);
    cmd[16] = '\0'; // Finaliza a string

    // Verifica se o comando é válido (começa com "XYZ" e termina com ".")
    if (strncmp(cmd, "XYZ", 3) == 0 && cmd[15] == '.') {
      // Extrai as coordenadas X, Y, Z do comando
      float targetX = ((cmd[3] == '-') ? -1 : 1) * ((cmd[4] - '0') * 100 + (cmd[5] - '0') * 10 + (cmd[6] - '0'));
      float targetY = ((cmd[7] == '-') ? -1 : 1) * ((cmd[8] - '0') * 100 + (cmd[9] - '0') * 10 + (cmd[10] - '0'));
      float targetZ = ((cmd[11] == '-') ? -1 : 1) * ((cmd[12] - '0') * 100 + (cmd[13] - '0') * 10 + (cmd[14] - '0'));

      Serial.print("Alvo Recebido: X="); Serial.print(targetX);
      Serial.print(" Y="); Serial.print(targetY);
      Serial.print(" Z="); Serial.println(targetZ);

      // Calcula e move o braço para as coordenadas recebidas
      calculateAndMove(targetX, targetY, targetZ);
    }

    // Limpa o buffer da serial para evitar leitura de lixo
    while (Serial.available()) Serial.read();
  }
}

void calculateAndMove(float x, float y, float z) {
  
  // --- Cálculo do Ângulo da Base (Eixo 1) ---
 float theta = atan2(y, x) * 180.0 / PI;
      if (theta < 0) theta += 360;

  // Define os alvos para os servos
      int anguloBase1, anguloBase2;
      if (theta <= 180) {
        anguloBase1 = theta;
        anguloBase2 = 0;
      } else {
        anguloBase1 = 180;
        anguloBase2 = theta - 180;
      }

  // --- Cinemática Inversa para Ombro e Cotovelo (Eixos 2 e 3) ---
  float r = sqrt(x*x + y*y); // Projeção da distância no plano XY
  float d = sqrt(r*r + z*z); // Distância real do ombro ao alvo

  // Verifica se o ponto está ao alcance do braço
  if (d > braco + antibraco) {
    Serial.println("ERRO: Ponto fora de alcance!");
    return; // Aborta o movimento
  }

  // Cálculo do Ângulo do Cotovelo (usando Arcsin, otimizado para braco == antibraco)
  float senAngCotovelo = d / (2 * antibraco); // Fórmula corrigida
  float anguloCotovelo = 2 * asin(constrain(senAngCotovelo, -1.0, 1.0)) * 180.0 / PI;

  // Cálculo do Ângulo do Ombro
  float anguloAlvoRad = atan2(z, r); // Ângulo do alvo em radianos (usando 'r')
  float anguloInternoRad = acos(constrain((braco*braco + d*d - antibraco*antibraco) / (2 * braco * d), -1.0, 1.0)); // Ângulo interno em radianos
  float anguloOmbro = (anguloAlvoRad + anguloInternoRad) * 180.0 / PI; // Soma em radianos, depois converte

  // --- Preparação dos valores para os servos ---
  // Converte os ângulos calculados para inteiros e os restringe ao intervalo 0-180
  //int targeombro = constrain(round(anguloOmbro), 0, 180);
  //int targetCotovelo = constrain(round(anguloCotovelo), 0, 180); // Com o método 'asin', não precisa inverter

  // Imprime os ângulos calculados para depuração
  Serial.print(" -> Ângulos Calculados: Base1="); Serial.print(anguloBase1);
  Serial.print(" Angulo Base2="); Serial.print(anguloBase2);
  Serial.print(" Angulo Ombro="); Serial.print(anguloOmbro);
  Serial.print(" Angulo Cotovelo="); Serial.println(anguloCotovelo);

  // Envia os comandos de movimento para os servos
  int anguloOmbroCompensado = constrain(round(anguloOmbro), 0, 180);
  //int anguloCotoveloCompensado = constrain(round(anguloCotovelo), 0, 180);

  moveServosSmooth(anguloBase1, anguloBase2, anguloOmbroCompensado, anguloCotovelo);
}

void moveServosSmooth(int base1, int base2, int ombro, int cotovelo) {
  unsigned long startTime = millis();
  const int stepDelay = 150; // Delay entre cada passo do servo (em ms)

  // Continua o loop enquanto a posição atual de qualquer servo for diferente da posição alvo
  while (posBase1 != base1 || posBase2 != base2 || posOmbro != ombro || posCotovelo != cotovelo) {
    
    // Executa um passo a cada 'stepDelay' milissegundos
    if (millis() - startTime > stepDelay) {
      startTime = millis();

      // Move cada servo em um passo na direção do alvo
      if (posBase1 != base1) {
        posBase1 += (posBase1 < base1) ? 1 : -1;
        servoBase1.write(posBase1);
      }
      if (posBase2 != base2) {
        posBase2 += (posBase2 < base2) ? 1 : -1;
        servoBase2.write(posBase2);
      }
      if (posOmbro != ombro) {
        posOmbro += (posOmbro < ombro) ? 1 : -1;
        servoOmbro.write(round(180.0 - posOmbro));
      }
      if (posCotovelo != cotovelo) {
        posCotovelo += (posCotovelo < cotovelo) ? 1 : -1;
        servoCotovelo.write(posCotovelo);
      }
    }
  }

  //Serial.print("Posição Final Atingida: Base1="); Serial.print(posBase1);
  //Serial.print(" Base2="); Serial.print(posBase2);
  //Serial.print(" Ombro="); Serial.print(posOmbro);
  //Serial.print(" Cotovelo="); Serial.println(posCotovelo);
  Serial.println("Movimento concluído.");
}

void resetServos() {
  Serial.println("Resetando servos para posição inicial...");
  // Move os servos para uma posição de repouso segura
  moveServosSmooth(0, 0, 0, 0);
  delay(500); // Pequena pausa após o reset
}
