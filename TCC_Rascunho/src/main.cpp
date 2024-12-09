///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <math.h>

////////////////////////// VARIÁVEIS //////////////////////////
#define PI 3.1415

const int SAMPLES = 1000;  // 200 amostras (2 segundos se amostra a cada 10 ms)
int sampleIndex = 0;

float refSinData[SAMPLES];  // Sinal de referência (seno)
float cosData[SAMPLES];     // Sinal cosseno controlado
float phaseData[SAMPLES];   // Diferença de fase calculada

float frequency = 10.0;     // Frequência inicial do cosseno controlado
float phaseSetpoint = 180.0; // Diferença de fase desejada

// PID
float Kp = 0.7, Ki = 0.05, Kd = 0.01;
float phaseError = 0, lastError = 0, integral = 0, derivative = 0;

volatile bool timerFlag = false;
bool collectingData = false;
hw_timer_t *timer = NULL;

// Variáveis para detecção de cruzamento por zero
unsigned long lastZeroCrossingRef = 0;
unsigned long lastZeroCrossingCos = 0;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
void IRAM_ATTR onTimer();
void Print();

float SenoRef(int index);
float CossenoControlado(int index, float freq);
float PIDControl(float error);

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);

  // Configuração do timer
  timer = timerBegin(0, 80, true);   // Prescaler 80 -> 1 tick = 1µs
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true); // 10ms
  timerAlarmDisable(timer);

  Serial.println("Digite 'START' para iniciar a coleta.");
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("START")) {
      if (!collectingData) {
        sampleIndex = 0;
        collectingData = true;
        timerFlag = false;
        timerAlarmEnable(timer);
        Serial.println("Coleta iniciada...");
      } else {
        Serial.println("Já coletando dados.");
      }
    } else if (command.equalsIgnoreCase("NEW")) {
      if (!collectingData) {
        sampleIndex = 0;
        Serial.println("Pronto para nova coleta, digite 'START'.");
      } else {
        Serial.println("Coleta em andamento.");
      }
    } else {
      Serial.println("Comando não reconhecido ('START' ou 'NEW').");
    }
  }

  if (collectingData && timerFlag) {
    timerFlag = false;

    float sRef = SenoRef(sampleIndex);
    float cCtrl = CossenoControlado(sampleIndex, frequency);

    refSinData[sampleIndex] = sRef;
    cosData[sampleIndex] = cCtrl;

    // Detectar cruzamentos por zero do seno de referência (neg->pos)
    if (sampleIndex > 0) {
      if (sRef >= 0 && refSinData[sampleIndex - 1] < 0) {
        lastZeroCrossingRef = micros();
      }
    }

    // Detectar cruzamentos por zero do cosseno controlado (neg->pos)
    if (sampleIndex > 0) {
      if (cCtrl >= 0 && cosData[sampleIndex - 1] < 0) {
        lastZeroCrossingCos = micros();
        // Quando o cosseno cruza zero, calcula a diferença de fase
        unsigned long timeDiff = lastZeroCrossingCos - lastZeroCrossingRef;
        // Período do seno de referência = 1 s = 1.000.000 µs
        float phaseDiff = (timeDiff / 1000000.0) * 360.0;
        if (phaseDiff > 360) phaseDiff = fmod(phaseDiff, 360.0);

        phaseData[sampleIndex] = phaseDiff;

        // Ajuste pelo PID
        phaseError = phaseSetpoint - phaseDiff;
        frequency += PIDControl(phaseError);
        if(frequency < 1.0) frequency = 1.0;
        if (frequency > 10.0) frequency = 10.0;
      }
    }

    sampleIndex++;

    if (sampleIndex >= SAMPLES) {
      collectingData = false;
      timerAlarmDisable(timer);
      Print();
    }
  }
}

///////////////////////// FUNÇÕES ///////////////////////////
void IRAM_ATTR onTimer() {
  timerFlag = true;
}

void Print() {
  Serial.println("Leituras:");
  for (int i = 0; i < SAMPLES; i++) {
    Serial.print("Amostra ");
    Serial.print(i);
    Serial.print(": RefSeno: ");
    Serial.print(refSinData[i], 4);
    Serial.print(" CosCtrl: ");
    Serial.print(cosData[i], 4);
    Serial.print(" Fase: ");
    Serial.println(phaseData[i], 4);
  }
}

float SenoRef(int index) {
  // Frequência do seno = 1 Hz, t = index * 0.01s
  float t = index * 0.01;
  return sin(2 * PI * 1.0 * t);
}

float CossenoControlado(int index, float freq) {
  float t = index * 0.01;
  return cos(2 * PI * freq * t);
}

float PIDControl(float error) {
  integral += error;
  derivative = error - lastError;
  lastError = error;
  return (Kp * error + Ki * integral + Kd * derivative);
}
