#include <QTRSensors.h>
QTRSensors qtr;

#define MOTOR_DIR_PIN 3 //pino PWM motor A
#define MOTOR_ESQ_PIN 5 //pino PWM motor B

#define IRD_PIN 8
#define IRE_PIN 12
#define NUM_LINHAS_PERCURSO_DIR 8

#define SENSOR_COUNT 8

#define PERIODO_AMOSTRAGEM 10
#define ULTIMOS_PIDS_SIZE 20 // 100 / PERIODO_AMOSTRAGEM;

#define VEL_MAX_LINEAR 255
#define VELMAX 255

// float KD = 4, KP = 8, erroLeitura;
// float KV = 1;
float KD = 4, KP = 9, erroLeitura;
float KV = 0.9;
// float KD = 4, KP = 10, erroLeitura;
// float KV = 0.6; // SUCESSO 23
float erro, erroAnterior = 0;
float ultimosPids[ULTIMOS_PIDS_SIZE] = {0};
int ultimosPidsIndex = -1;
float correcaoD, correcaoE, correcao;
int potenciaD = 0;
int potenciaE = 0;

float maxProporcao = 300;

uint16_t sensorValues[SENSOR_COUNT];

byte IRE = 0;
byte IRD = 0;

int countLinhasD = 0;

unsigned long previousMillis = 0, currentMillis = 0;

void setMotorD(int vel)
{
  analogWrite(MOTOR_DIR_PIN, vel);
}

void setMotorE(int vel)
{
  analogWrite(MOTOR_ESQ_PIN, vel);
}

void acionaMotores(float velLinear, float velRadial)
{
  correcaoD = velLinear - (velRadial * 0.5);
  correcaoE = velLinear + (velRadial * 0.5);

  potenciaD = constrain(correcaoD, 0, VELMAX);
  potenciaE = constrain(correcaoE, 0, VELMAX);

  // Serial.print(";   PID: ");
  // Serial.print(velRadial);
  // Serial.print(";   VelA: ");
  // Serial.print(correcaoD);
  // Serial.print(";   VelB: ");
  // Serial.print(correcaoE);
  // Serial.println(";");

  setMotorD(potenciaD);
  setMotorE(potenciaE);
}

void addItemUltimasProporcoes(float item)
{
  int numItems = ULTIMOS_PIDS_SIZE;
  int newIndex = (ultimosPidsIndex + 1) % numItems;
  ultimosPids[newIndex] = item;
  ultimosPidsIndex = newIndex;
}
float getMediaUltimasProporcoes()
{
  int numItems = ULTIMOS_PIDS_SIZE;
  float count = 0;
  for (int i = 0; i < numItems; i++)
  {
    count += ultimosPids[i];
  }
  return count / numItems;
}
void printUltimasProporcoes()
{
  int numItems = ULTIMOS_PIDS_SIZE;
  Serial.print("[");
  for (int i = 0; i < numItems; i++)
  {
    int index = (ultimosPidsIndex + i) % numItems;
    Serial.print(ultimosPids[index]);
    Serial.print(",");
  }
  Serial.print("]\n");
}

void setup()
{
  Serial.begin(9600);
  Serial.println("INICIO");
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_ESQ_PIN, OUTPUT);

  qtr.setTypeRC();
  const uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5, 2, 4};
  qtr.setSensorPins(sensorPins, SENSOR_COUNT);
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate(); //calibração do sensor;
  }
}

void loop()
{
  bool lastIRE = IRE;
  bool lastIRD = IRD;
  IRD = !digitalRead(IRD_PIN);
  IRE = !digitalRead(IRE_PIN);
  // Serial.print(IRD);
  // Serial.print(", ");
  // Serial.print(IRE);
  // Serial.print(", ");
  // Serial.print(lastIRD);
  // Serial.print(", ");
  // Serial.print(countLinhasD);
  // Serial.println();
  if (IRD && IRD != lastIRD)
  {
    countLinhasD++;
  }
  if (countLinhasD >= NUM_LINHAS_PERCURSO_DIR)
  {
    acionaMotores(255, 0);
    delay(800);
    acionaMotores(0, 0);
    Serial.println("FIM");
    exit(0);
  }
  currentMillis = millis();
  if (currentMillis - previousMillis >= PERIODO_AMOSTRAGEM)
  {
    previousMillis = currentMillis;
    int position = qtr.readLineWhite(sensorValues);
    // position: de 0 a 7000
    erro = 3500 - position;
    // erro: de -3500 a 3500
    erroLeitura = position / 3500.0 - 1.0;
    // erro_leitura: de -1 a 1
    // kp: de 1 a 10
    // kd: de 1 a 10
    float PID = KP * erroLeitura + KD * (erroLeitura - erroAnterior);
    // PID: da ordem 1 a 10 (vezes o original)
    erroAnterior = erroLeitura;

    float media = getMediaUltimasProporcoes();

    float compensacaoVelocidadeLinear = constrain(1.0 - (abs(media) / 255.0) * KV, 0, 1);
    float velocidadeLinear = VEL_MAX_LINEAR * compensacaoVelocidadeLinear;

    correcao = PID * velocidadeLinear;

    if (erroLeitura == 1 || erroLeitura == -1)
    {
      // Fora da pista
      if (media > 0)
      {
        // Saiu para a esquerda, precisa ir para a direita
        acionaMotores(velocidadeLinear, velocidadeLinear);
      }
      else
      {
        // Saiu para a direita, precisa ir para a esquerda
        acionaMotores(velocidadeLinear, -velocidadeLinear);
      }
    }
    else
    {
      acionaMotores(velocidadeLinear, correcao);
    }

    addItemUltimasProporcoes(potenciaD - potenciaE);

    // Serial.print("Sensor: ");
    // Serial.print(position);
    // Serial.print(" Erro: ");
    // Serial.print(erroLeitura);
    // Serial.print("; PID: ");
    // Serial.print(PID);
    // Serial.print("; Comp. Vel: ");
    // Serial.print(compensacaoVelocidadeLinear);
    // Serial.print("; VelA: ");
    // Serial.print(correcaoD);
    // Serial.print("; VelB: ");
    // Serial.print(correcaoE);
    // Serial.println(";");
  }
}
