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
const int SAMPLES = 100; // 10ms por amostra, total 1s
int16_t accelData[SAMPLES][3]; // X, Y, Z
int sampleIndex = 0;

// Configuração do Encoder
const int encoderPinA = 14;
const int encoderPinB = 32;
volatile long encoderPulses = 0; // Contador de pulsos do encoder
volatile long lastEncoderPulses = 0; // Pulsos anteriores para calcular RPS
float currentRPS = 0; // Rotações por segundo
const int pulsesPerRevolution = 1000;

// Estados e controle
volatile bool timerFlag = false; // Indica que a ISR foi disparada
bool collectingData = false; // Indica se a coleta está ativa
SemaphoreHandle_t dataMutex; // Mutex para proteger o vetor

// Timer de interrupção
hw_timer_t *timer = NULL;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
void setupMPU9250();
void readMPU9250(int16_t &x, int16_t &y, int16_t &z);
void IRAM_ATTR onTimer(); // Função de interrupção
void IRAM_ATTR encoderISR_A(); // Interrupção para o pino A
void IRAM_ATTR encoderISR_B(); // Interrupção para o pino B
float calculateRPS(); // Função para calcular RPS
void convertAndPrintData();

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configuração do sensor
  setupMPU9250();

  // Configuração do encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR_B, CHANGE);

  // Inicialização do mutex
  dataMutex = xSemaphoreCreateMutex();

  // Configuração do timer
  timer = timerBegin(0, 80, true); // Prescaler de 80 para 1 MHz (1 µs por tick)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true); // Dispara a cada 10 ms
  timerAlarmDisable(timer); // Desativa inicialmente

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
        lastEncoderPulses = encoderPulses; // Reseta os pulsos acumulados
        timerAlarmEnable(timer); // Inicia o timer
        Serial.println("Coleta de dados iniciada...");
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

    int16_t x, y, z;
    readMPU9250(x, y, z);

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      accelData[sampleIndex][0] = x;
      accelData[sampleIndex][1] = y;
      accelData[sampleIndex][2] = z;
      xSemaphoreGive(dataMutex);
    }

    // Calcula o RPS
    currentRPS = calculateRPS();

    sampleIndex++;

    if (sampleIndex >= SAMPLES) {
      collectingData = false;
      timerAlarmDisable(timer); // Para o timer
      convertAndPrintData();
    }
  }
}

///////////////////////// FUNÇÕES ///////////////////////////
void setupMPU9250() {
  // Configuração de Sample Rate Divider
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(SMPLRT_DIV);
  Wire.write(99); // 100 Hz (1 kHz / (1 + 99))
  Wire.endTransmission();

  // Configuração do filtro DLPF
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(CONFIG);
  Wire.write(2); // 92 Hz de banda passante
  Wire.endTransmission();

  // Configuração da escala do acelerômetro
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0); // ±2g
  Wire.endTransmission();
}

void readMPU9250(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);

  x = (Wire.read() << 8) | Wire.read();
  y = (Wire.read() << 8) | Wire.read();
  z = (Wire.read() << 8) | Wire.read();
}

void IRAM_ATTR onTimer() {
  timerFlag = true; // Apenas seta o flag
}

void IRAM_ATTR encoderISR_A() {

  encoderPulses++;
  
}

void IRAM_ATTR encoderISR_B() {
 
  encoderPulses++;

}

float calculateRPS() {
  long currentPulses = encoderPulses;
  long pulseDifference = currentPulses - lastEncoderPulses;
  lastEncoderPulses = currentPulses;
  return (pulseDifference / (float)pulsesPerRevolution) / 0.01; // 0.01s = 10ms
}

void convertAndPrintData() {
  const float scale = 2.0 / 32768.0; // Conversão para g, considerando ±2g

  Serial.println("Leitura de acelerômetro (g) e velocidade (RPS):");
  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
    for (int i = 0; i < SAMPLES; i++) {
      float ax = accelData[i][0] * scale;
      float ay = accelData[i][1] * scale;
      float az = accelData[i][2] * scale;

      Serial.print("Amostra ");
      Serial.print(i);
      Serial.print(": X=");
      Serial.print(ax, 4);
      Serial.print("g, Y=");
      Serial.print(ay, 4);
      Serial.print("g, Z=");
      Serial.print(az, 4);
      Serial.print("g, RPS=");
      Serial.println(currentRPS, 4);
    }
    xSemaphoreGive(dataMutex);
  }
}
