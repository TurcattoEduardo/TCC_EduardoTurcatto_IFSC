///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

////////////////////////// VARIÁVEIS //////////////////////////
// Configurações do MPU-9250
const int MPU9250_ADDR = 0x68;
const int SMPLRT_DIV = 0x19;
const int CONFIG = 0x1A;
const int ACCEL_CONFIG = 0x1C;
const int ACCEL_XOUT_H = 0x3B;

// Vetor para armazenar leituras do acelerômetro
const int SAMPLES = 1000; // 10ms por amostra, total 1s
float accelData[SAMPLES][3]; // X, Y, Z em mm/s²
int sampleIndex = 0;

// Configuração do Encoder
const int encoderPinA = 14;
const int encoderPinB = 32;
volatile long encoderPulsesA = 0; // Contador de pulsos do encoder
volatile long lastEncoderPulses = 0; // Pulsos anteriores para calcular RPS
float currentRPS = 0; // Rotações por segundo
const int pulsesPerRevolution = 1000;

// Offsets do acelerômetro
float offsetX = 0.0;
float offsetY = 0.0;
float offsetZ = 0.0;

uint8_t Ctrl_Vel = 75;
volatile long Pulse_Counter = 0;

// Estados e controle
volatile bool timerFlag = false; // Indica que a ISR foi disparada
bool collectingData = false; // Indica se a coleta está ativa
SemaphoreHandle_t dataMutex; // Mutex para proteger o vetor

// Timer de interrupção
hw_timer_t *timer = NULL;

// Constante do filtro IIR
float alpha = 0.1; // Ajuste entre 0 e 1 para suavização (menor = mais suave)

// Valores filtrados anteriores
float filteredX = 0.0, filteredY = 0.0, filteredZ = 0.0;

// PWM
#define Ctrl_Motor_Pin_1 27
#define Ctrl_Motor_Pin_2 33

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
void setupMPU9250();
void calibrateMPU9250();
void readMPU9250(float &ax, float &ay, float &az);
void readAndFilterMPU(float &ax, float &ay, float &az); // Nova função com filtro
void IRAM_ATTR onTimer(); // Função de interrupção
float calculateRPS(); // Função para calcular RPS
void convertAndPrintData();

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configuração do sensor
  setupMPU9250();
  calibrateMPU9250(); // Calibração do acelerômetro

  // Configuração do encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), []() { encoderPulsesA++;Pulse_Counter++; }, FALLING);

  // Inicialização do mutex
  dataMutex = xSemaphoreCreateMutex();

  // Configuração do timer
  timer = timerBegin(0, 80, true); // Prescaler de 80 para 1 MHz (1 µs por tick)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true); // Dispara a cada 10 ms
  timerAlarmDisable(timer); // Desativa inicialmente

  // PWM
  pinMode(Ctrl_Motor_Pin_1, OUTPUT);
  pinMode(Ctrl_Motor_Pin_2, OUTPUT);

  analogWrite(Ctrl_Motor_Pin_1,0);
  analogWrite(Ctrl_Motor_Pin_2,0);

  Serial.println("Digite 'START' para iniciar a coleta de dados.");
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
        timerAlarmEnable(timer); // Inicia o timer
        Serial.println("Coleta de dados iniciada...");
        analogWrite(Ctrl_Motor_Pin_1,Ctrl_Vel);
      } else {
        Serial.println("Já está coletando dados. Aguarde o término.");
      }
    } else if (command.equalsIgnoreCase("NEW")) {
      if (!collectingData) {
        sampleIndex = 0;
        Serial.println("Pronto para uma nova coleta. Digite 'START' para começar.");
      } else {
        Serial.println("Coleta em andamento. Aguarde o término.");
      }
    } else {
      Serial.println("Comando não reconhecido. Use 'START' ou 'NEW'.");
    }
  }

  if (collectingData && timerFlag) {
    timerFlag = false; // Limpa o flag da ISR

    float ax, ay, az;
    readAndFilterMPU(ax, ay, az); // Leitura com filtro IIR aplicado

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      accelData[sampleIndex][0] = ax;
      accelData[sampleIndex][1] = ay;
      accelData[sampleIndex][2] = az;
      xSemaphoreGive(dataMutex);
    }

    // Calcula o RPS
    currentRPS = calculateRPS();
    encoderPulsesA = 0;

    sampleIndex++;

    if (sampleIndex >= SAMPLES) {
      collectingData = false;
      timerAlarmDisable(timer); // Para o timer
      analogWrite(Ctrl_Motor_Pin_1,0);
      convertAndPrintData();
    }
  }
}

///////////////////////// FUNÇÕES ///////////////////////////
void setupMPU9250() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); // Acorda o sensor
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x00); // Sem filtro
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x00); // ±2g
  Wire.endTransmission();
}

void calibrateMPU9250() {
  analogWrite(Ctrl_Motor_Pin_1,0);
  int16_t rawX, rawY, rawZ;
  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 6, true);

    rawX = (Wire.read() << 8) | Wire.read();
    rawY = (Wire.read() << 8) | Wire.read();
    rawZ = (Wire.read() << 8) | Wire.read();

    sumX += rawX;
    sumY += rawY;
    sumZ += rawZ;
    delay(10);
  }

  offsetX = sumX / samples;
  offsetY = sumY / samples;
  offsetZ = (sumZ / samples) - 16384;
}

void readMPU9250(float &ax, float &ay, float &az) {
  int16_t rawX, rawY, rawZ;

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);

  rawX = (Wire.read() << 8) | Wire.read();
  rawY = (Wire.read() << 8) | Wire.read();
  rawZ = (Wire.read() << 8) | Wire.read();

  ax = rawX - offsetX;
  ay = rawY - offsetY;
  az = rawZ - offsetZ;

  const float scale = 2.0 / 32768.0;
  const float gToMMs2 = 9806.65;

  ax *= scale * gToMMs2;
  ay *= scale * gToMMs2;
  az *= scale * gToMMs2;
}

void readAndFilterMPU(float &ax, float &ay, float &az) {
  readMPU9250(ax, ay, az);
  filteredX = alpha * ax + (1 - alpha) * filteredX;
  filteredY = alpha * ay + (1 - alpha) * filteredY;
  filteredZ = alpha * az + (1 - alpha) * filteredZ;

  ax = filteredX;
  ay = filteredY;
  az = filteredZ;
}

void IRAM_ATTR onTimer() {
  timerFlag = true;
}

float calculateRPS() {
  return (encoderPulsesA / (float)pulsesPerRevolution) / 0.01;
}

void convertAndPrintData() {
  Serial.println("Leitura de acelerômetro (mm/s²) e velocidade (RPS):");
  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
    for (int i = 0; i < SAMPLES; i++) {
      Serial.print("Amostra ");
      Serial.print(i);
      Serial.print(": X= ");
      Serial.print(accelData[i][0], 2);
      Serial.print(" mm/s², Y= ");
      Serial.print(accelData[i][1], 2);
      Serial.print(" mm/s², Z= ");
      Serial.print(accelData[i][2], 2);
      Serial.print(" mm/s², RPS= ");
      Serial.println(currentRPS, 4);
    }
    xSemaphoreGive(dataMutex);
  }
}
