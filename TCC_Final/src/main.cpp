///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

////////////////////////// VARIÁVEIS //////////////////////////
// Configurações do MPU-9250
const int MPU9250_ADDR = 0x68;
const int SMPLRT_DIV = 0x19;
const int CONFIG = 0x1A;
const int ACCEL_CONFIG = 0x1C;
const int ACCEL_XOUT_H = 0x3B;

// Offsets do acelerômetro
float offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

// Variáveis auxiliares do acelerômetro
float FiltredAy = 0.0, Alpha = 0.1;
float AccelCurrentValue = 0.0;
float AccelLastValue = 0.0;
unsigned long AccelLastZeroCrossingPos = 0;
unsigned long AccelLastZeroCrossingNeg = 0;

// Configuração do Encoder
const int encoderPinA = 14;
volatile long encoderPulses = 0; // Contador de pulsos do encoder
const int pulsesPerRevolution = 1000; // Pulsos por rotação

// Variaveis auxiliares encoder
float CurrentRPS = 0.0;
long lastPulseCount = 0;
float EncCurrentValue = 0;

float EncLastValue = 0;
float EncLastZeroCrossingPos = 0, EncLastZeroCrossingNeg = 0;

// Controle de Frequencia
float FreqCurrentError = 0, FreqLastError = 0, FreqCurrentControl = 0, FreqLastControl = 0;
const float FreqKp = 1, FreqKi = 1;
float FreqRef = 0;

// Controle de Phase
float PhaseCurrentError = 0, PhaseLastError = 0, PhaseCurrentControl = 0, PhaseLastControl = 0;
const float PhaseKp = 1, PhaseKi = 1;
float PhaseRef = 180;
float CurrentPhase = 0;

// Controle de tempo e flags
hw_timer_t *velocityTimer = NULL; // Timer para controle de velocidade
hw_timer_t *accelerometerTimer = NULL; // Timer para leitura do acelerômetro
hw_timer_t *phaseControlTimer = NULL; // Timer para controle de fase
volatile bool velocityFlag = false;
volatile bool accelerometerFlag = false;
volatile bool phaseControlFlag = false;
volatile bool systemRunning = false; // Estado do sistema

// Motor
#define Ctrl_Motor_A 33
#define Ctrl_Motor_H 27

// Mutex para proteger os dados compartilhados
SemaphoreHandle_t dataMutex;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
void setupMPU9250();
void calibrateMPU9250();
void readMPU9250(float &ax, float &ay, float &az);
void FreqControl();
void CalculateRPS();
void EncSin();
void ZeroCrossing();
void ApllyMotorControl(float DutyCicle);
void calculatePhaseControl();
void CalculatePhaseDifference();
void IRAM_ATTR onVelocityTimer();
void IRAM_ATTR onAccelerometerTimer();
void IRAM_ATTR onPhaseControlTimer();
void handleUserCommands();

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configuração do sensor
  setupMPU9250();
  calibrateMPU9250();

  // Configuração do encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), []() { encoderPulses++; }, RISING);

  // Configuração dos timers
  velocityTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(velocityTimer, &onVelocityTimer, true);
  timerAlarmWrite(velocityTimer, 1000, true); // 1ms
  timerAlarmDisable(velocityTimer);

  accelerometerTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(accelerometerTimer, &onAccelerometerTimer, true);
  timerAlarmWrite(accelerometerTimer, 10000, true); // 10ms
  timerAlarmDisable(accelerometerTimer);

  phaseControlTimer = timerBegin(2, 80, true);
  timerAttachInterrupt(phaseControlTimer, &onPhaseControlTimer, true);
  timerAlarmWrite(phaseControlTimer, 200000, true); // 200ms
  timerAlarmDisable(phaseControlTimer);

  // Inicialização do mutex
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("Erro ao criar mutex!");
    while (1); // Travar o sistema se não puder criar o mutex
  }

  pinMode(Ctrl_Motor_A, OUTPUT);
  pinMode(Ctrl_Motor_H, OUTPUT);
  analogWrite(Ctrl_Motor_A, 0);
  analogWrite(Ctrl_Motor_H, 0);

  Serial.println("Sistema iniciado. Digite 'START' para iniciar ou 'STOP' para pausar.");
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  handleUserCommands(); // Verifica os comandos do usuário

  if (systemRunning) {
    if (velocityFlag) {
      velocityFlag = false;

      // Proteção com mutex
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        FreqControl();
        xSemaphoreGive(dataMutex);
      }
    }

    if (accelerometerFlag) {
      accelerometerFlag = false;

      // Proteção com mutex
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        float ax, ay, az;
        readMPU9250(ax,ay,az);
        AccelCurrentValue = ay;
        EncSin();
        ZeroCrossing();
        xSemaphoreGive(dataMutex);
      }
    }

    if (phaseControlFlag) {
      phaseControlFlag = false;

      // Proteção com mutex
      if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        calculatePhaseControl();
        xSemaphoreGive(dataMutex);
      }
    }
  }
}

///////////////////////// FUNÇÕES ///////////////////////////
// Mpu 9250
void setupMPU9250() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(SMPLRT_DIV);
  Wire.write(0x68); // 100 Hz (1 kHz / (1 + 99))
  Wire.endTransmission();

  // Configuração do filtro DLPF
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(CONFIG);
  Wire.write(0); // 92 Hz de banda passante
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C); // Configuração do intervalo de aceleração
  Wire.write(0x00); // ±2g
  Wire.endTransmission();
}

void calibrateMPU9250() {
  analogWrite(Ctrl_Motor_A, 0);
  analogWrite(Ctrl_Motor_H, 0);
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
  offsetZ = (sumZ / samples) - 16384; // Ajuste para compensar a gravidade
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

  ay = rawY - offsetY;

  FiltredAy = Alpha * ay + (1 - Alpha) * FiltredAy;

  ay = FiltredAy;

  const float scale = 2.0 / 32768.0;
  const float gToMMs2 = 9806.65;

  ay *= scale * gToMMs2;
}

// Controle de Frequência
void FreqControl() {
  CalculateRPS();
  
  FreqCurrentError = FreqRef - CurrentRPS;

  FreqCurrentControl = FreqLastControl + FreqLastError*FreqKp + FreqLastError*FreqKi;

  FreqLastControl = FreqCurrentControl;
  FreqLastError = FreqCurrentError;

  ApllyMotorControl(FreqCurrentControl);
}

void CalculateRPS(){
  long pulseDifference = encoderPulses - lastPulseCount;
  CurrentRPS = (pulseDifference / (float)pulsesPerRevolution) / 0.001; // 0.001s = 1ms
  lastPulseCount = encoderPulses;
}

void EncSin(){
  float angle = (float)(encoderPulses%pulsesPerRevolution)/pulsesPerRevolution*2.0*PI;
  EncCurrentValue = sin(angle);
}

void calculatePhaseControl() {
  CalculatePhaseDifference();

  PhaseCurrentError = PhaseRef - CurrentPhase;

  PhaseCurrentControl = PhaseLastControl + PhaseCurrentError*PhaseKp - PhaseLastError*PhaseKi;

  PhaseLastControl = PhaseCurrentControl;
  PhaseLastError = PhaseCurrentError;

  ApllyMotorControl(PhaseCurrentControl+FreqCurrentControl);
}

void CalculatePhaseDifference(){
  if (AccelLastZeroCrossingNeg>0 && EncLastZeroCrossingPos >0){
    // Calcula o intervalo de tempo entre cruzamentos
    long timeDifference = AccelLastZeroCrossingNeg - EncLastZeroCrossingPos;

    // Converte o tempo em diferença de fase
    CurrentPhase = (timeDifference / (1000.0 / CurrentRPS)) * 360.0;

    // Normaliza para o intervalo [-180, 180]
    while (CurrentPhase > 180.0) CurrentPhase -= 360.0;
    while (CurrentPhase < -180.0) CurrentPhase += 360.0;

  } else if(AccelLastZeroCrossingPos > 0 && EncLastZeroCrossingNeg > 0 ){
    // Calcula o intervalo de tempo entre cruzamentos
    long timeDifference = AccelLastZeroCrossingPos - EncLastZeroCrossingNeg;

    // Converte o tempo em diferença de fase
    CurrentPhase = (timeDifference / (1000.0 / CurrentRPS)) * 360.0;

    // Normaliza para o intervalo [-180, 180]
    while (CurrentPhase > 180.0) CurrentPhase -= 360.0;
    while (CurrentPhase < -180.0) CurrentPhase += 360.0;
  }

  Serial.print("Diferença de fase: ");
  Serial.print(CurrentPhase);
  Serial.println(" graus");
}

void ZeroCrossing(){
  float currentTime = millis();

  if(AccelLastValue >= 0 && AccelCurrentValue < 0){
    AccelLastZeroCrossingNeg = currentTime;
    
  }else if (AccelLastValue <= 0 && AccelCurrentValue > 0){
    AccelLastZeroCrossingPos = currentTime;
  }

  if(EncCurrentValue >= 0 && EncLastValue < 0){
    EncLastZeroCrossingNeg = currentTime;
  }else if (EncCurrentValue <= 0 && EncLastValue > 0){
    EncLastZeroCrossingPos = currentTime;
  }

  EncLastValue = EncCurrentValue;
  AccelLastValue = AccelCurrentValue;
}

void ApllyMotorControl(float DutyCicle){
  analogWrite(Ctrl_Motor_A, constrain(DutyCicle, 0, 255));
}

void IRAM_ATTR onVelocityTimer() {
  velocityFlag = true;
}

void IRAM_ATTR onAccelerometerTimer() {
  accelerometerFlag = true;
}

void IRAM_ATTR onPhaseControlTimer() {
  phaseControlFlag = true;
}

// Controle de interações com o unsuário
void handleUserCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("START")) {
      if (!systemRunning) {
        systemRunning = true;
        timerAlarmEnable(velocityTimer);
        timerAlarmEnable(accelerometerTimer);
        timerAlarmEnable(phaseControlTimer);
        Serial.println("Sistema iniciado.");
      } else {
        Serial.println("Sistema já está em execução.");
      }
    } else if (command.equalsIgnoreCase("STOP")) {
      if (systemRunning) {
        systemRunning = false;
        timerAlarmDisable(velocityTimer);
        timerAlarmDisable(accelerometerTimer);
        timerAlarmDisable(phaseControlTimer);
        Serial.println("Sistema pausado.");
      } else {
        Serial.println("Sistema já está pausado.");
      }
    } else {
      Serial.println("Comando não reconhecido. Use 'START' ou 'STOP'.");
    }
  }
}
