//itineráio:
// - identificar a velocidade máxima e a velocidade mínima
// - testar leitura do sensor
// - identificar as constantes do PID

#include <QTRSensors.h>
QTRSensors qtr;

#define motorA 10 //pino A1 ponte-h
#define IN1 9     //pino B1 ponte-h
#define IN2 8     //pino B1 ponte-h
#define IN3 7     //pino B1 ponte-h
#define IN4 6     //pino B1 ponte-h
#define STBY 3
#define motorB 5 //pino B2 ponte-h

#define ULTIMAS_PROPORCOES_COUNT 15

float KD = 500, KI = 50, KP = 130, ERRO_LEITURA, INTEGRAL, INTEGRAL_ANTERIOR = 0;
float ERRO, ERRO_ANTERIOR = 0;
float ULTIMAS_PROPORCOES[ULTIMAS_PROPORCOES_COUNT] = {0};
int ULTIMAS_PROPORCOES_INDEX = -1;
float VELOCIDADE_MIN_A, VELOCIDADE_MIN_B, VELOCIDADE_A, VELOCIDADE_B, VELOCIDADE;
float VEL_MAX_LINEAR = 150;
float VELMAX = 250;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

unsigned long previousMillis = 0, currentMillis = 0;

void setMotorA(int vel)
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

void setMotorB(int vel)
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
  VELOCIDADE_A = velLinear - (velRadial * 0.5);
  VELOCIDADE_B = velLinear + (velRadial * 0.5);

  VELOCIDADE_A = constrain(VELOCIDADE_A, -VELMAX, VELMAX);
  VELOCIDADE_B = constrain(VELOCIDADE_B, -VELMAX, VELMAX);

  Serial.print(";   PID: ");
  Serial.print(velRadial);
  Serial.print(";   VelA: ");
  Serial.print(VELOCIDADE_A);
  Serial.print(";   VelB: ");
  Serial.print(VELOCIDADE_B);
  Serial.println(";");

  setMotorA(VELOCIDADE_A);
  setMotorB(VELOCIDADE_B);
}

void setup()
{
  pinMode(motorA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(motorB, OUTPUT);

  qtr.setTypeRC();
  uint8_t sensorPins[] = {A0, A1, A2, A3, A4, A5, 2, 4};
  qtr.setSensorPins(sensorPins, SensorCount);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate(); //calibração do sensor;
  }
  Serial.begin(9600);

  digitalWrite(STBY, 1);

  VELOCIDADE_MIN_A = 80;
  VELOCIDADE_MIN_B = 82; // VELOICIDADE CERTA E 72
}

void addItemUltimasProporcoes(float item)
{
  int numItems = ULTIMAS_PROPORCOES_COUNT;
  int newIndex = (ULTIMAS_PROPORCOES_INDEX + 1) % numItems;
  ULTIMAS_PROPORCOES[newIndex] = item;
  ULTIMAS_PROPORCOES_INDEX = newIndex;
}
float getMediaUltimasProporcoes()
{
  int numItems = ULTIMAS_PROPORCOES_COUNT;
  float count = 0;
  for (int i = 0; i < numItems; i++)
  {
    count += ULTIMAS_PROPORCOES[i];
  }
  return count / numItems;
}
void printUltimasProporcoes()
{
  int numItems = ULTIMAS_PROPORCOES_COUNT;
  Serial.print("[");
  for (int i = 0; i < numItems; i++)
  {
    int index = (ULTIMAS_PROPORCOES_INDEX + i) % numItems;
    Serial.print(ULTIMAS_PROPORCOES[index]);
    Serial.print(",");
  }
  Serial.print("]\n");
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis >= 50)
  {
    previousMillis = currentMillis;
    int position = qtr.readLineWhite(sensorValues);
    ERRO = 3500 - position;
    ERRO_LEITURA = position / 3500.0 - 1.0;
    INTEGRAL = KI * (ERRO_LEITURA + ERRO_ANTERIOR);
    +INTEGRAL_ANTERIOR;
    INTEGRAL_ANTERIOR = INTEGRAL;
    VELOCIDADE = KP * ERRO_LEITURA + KD * (ERRO_LEITURA - ERRO_ANTERIOR) + INTEGRAL;
    ERRO_ANTERIOR = ERRO_LEITURA;

    addItemUltimasProporcoes(VELOCIDADE);
    float media = getMediaUltimasProporcoes();

    float compensacaoVelocidade = constrain(1.0 - abs(media) / 1000.0, 0, 1);
    float velocidade = VEL_MAX_LINEAR * compensacaoVelocidade;

    if (ERRO_LEITURA == 1 || ERRO_LEITURA == -1)
    {
      // Fora da pista
      if (media > 0)
      {
        // Saiu para a esquerda, precisa ir para a direita
        acionaMotores(velocidade, 400);
      }
      else
      {
        // Saiu para a direita, precisa ir para a esquerda
        acionaMotores(velocidade, -400);
      }
    }
    else
    {
      acionaMotores(velocidade, VELOCIDADE);
    }

    /*Serial.print("Sensor: ");
      Serial.print(position);
      Serial.print("    Erro: ");
      Serial.print(ERRO_LEITURA);
      Serial.print(";   PID: ");
      Serial.print(VELOCIDADE);
      Serial.print(";   VelA: ");
      Serial.print(VELOCIDADE_A);
      Serial.print(";   VelB: ");
      Serial.print(VELOCIDADE_B);
      Serial.println(";");*/
  }
}
