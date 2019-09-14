#include <QTRSensors.h>
QTRSensors qtr;

#define motorA 10 //pino A1 ponte-h
#define motorB 5  //pino B2 ponte-h
#define IN1 9     //pino B1 ponte-h
#define IN2 8     //pino B1 ponte-h
#define IN3 7     //pino B1 ponte-h
#define IN4 6     //pino B1 ponte-h
#define STBY 3

#define SENSOR_COUNT 8

#define PERIODO_AMOSTRAGEM 10
#define ULTIMOS_PIDS_SIZE 20 // 100 / PERIODO_AMOSTRAGEM;

#define VEL_MAX_LINEAR 200
#define VELMAX 255

float KD = 5, KP = 9, erroLeitura;
// 5
float KV = 0.4;
// 0.5
float erro, erroAnterior = 0;
float ultimosPids[ULTIMOS_PIDS_SIZE] = {0};
int ultimosPidsIndex = -1;
float correcaoD, correcaoE, correcao;

float maxProporcao = 300;

uint16_t sensorValues[SENSOR_COUNT];

unsigned long previousMillis = 0, currentMillis = 0;

void setMotorD(int vel)
{
  if (vel >= 0)
  {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    analogWrite(motorA, vel);
  }
  else
  {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    analogWrite(motorA, -vel);
  }
}

void setMotorE(int vel)
{
  if (vel >= 0)
  {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
    analogWrite(motorB, vel);
  }
  else
  {
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
    analogWrite(motorB, -vel);
  }
}

void acionaMotores(float velLinear, float velRadial)
{
  correcaoD = velLinear - (velRadial * 0.5);
  correcaoE = velLinear + (velRadial * 0.5);

  correcaoD = constrain(correcaoD, -VELMAX, VELMAX);
  correcaoE = constrain(correcaoE, -VELMAX, VELMAX);

  // Serial.print(";   PID: ");
  // Serial.print(velRadial);
  // Serial.print(";   VelA: ");
  // Serial.print(correcaoD);
  // Serial.print(";   VelB: ");
  // Serial.print(correcaoE);
  // Serial.println(";");

  setMotorD(correcaoD);
  setMotorE(correcaoE);
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
  pinMode(motorA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(motorB, OUTPUT);

  qtr.setTypeRC();
  const uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5, 2, 4};
  qtr.setSensorPins(sensorPins, SENSOR_COUNT);
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate(); //calibração do sensor;
  }

  digitalWrite(STBY, 1);
}

void loop()
{
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

    addItemUltimasProporcoes(PID);
    float media = getMediaUltimasProporcoes();

    float compensacaoVelocidadeLinear = constrain(1.0 - (abs(media) / 100.0) * KV, 0, 1);
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
