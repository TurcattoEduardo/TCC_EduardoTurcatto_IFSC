///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <arduinoFFT.h>

////////////////////////// VARIÁVEIS //////////////////////////
// Configurações do MPU-9250
const int MPU9250_ADDR = 0x68;
const int SMPLRT_DIV = 0x19;
const int CONFIG = 0x1A;
const int ACCEL_CONFIG = 0x1C;
const int ACCEL_XOUT_H = 0x3B;

// Vetor para armazenar leituras do acelerômetro e sinal senoidal
const int SAMPLES = 64; // Total de amostras por teste
float accelData[SAMPLES][3]; // X, Y, Z em mm/s²
float sinData[SAMPLES]; // Sinal senoidal
volatile int sampleIndex = 0;
float filteray=0;
float alpha = 0.1;

// Offsets do acelerômetro
float offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

// Configuração do Encoder
const int encoderPinA = 14;
volatile long encoderPulses = 0; // Contador de pulsos do encoder
const int pulsesPerRevolution = 1000; // Pulsos por rotação
float garfoAmplitude = 5.0; // Amplitude do sinal do garfo escocês (ajustar conforme modelo)

// Timer e controle de interrupção
hw_timer_t *timer = NULL;
volatile bool timerFlag = false; // Flag setada pela interrupção
bool collectingData = false; // Controle do estado de coleta

// Motor
#define Ctrl_Motor_A 33
#define Ctrl_Motor_H 27

// Configuração para FFT
#define SAMPLES 64 // Mesmo número de amostras
#define SAMPLING_FREQUENCY 100.0 // Frequência de amostragem em Hz

// Vetores para FFT
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY); // Instância da classe com tipo double

// Controle
float e_ant = 0;
float c_ant = 0;
float Kp = 0.08;
float Ki = 0.004;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
void setupMPU9250();
void calibrateMPU9250();
void readMPU9250(float &ax, float &ay, float &az);
void reconstructSinSignal(int index);
void printCollectedData();
void IRAM_ATTR onTimer(); // Função de interrupção
void calculatePhaseDifference();
void Controle(double erro);

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

  // Configuração do timer para interrupção a cada 10 ms
  timer = timerBegin(0, 80, true); // Prescaler de 80 para 1 MHz (1 µs por tick)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true); // Dispara a cada 10 ms
  timerAlarmDisable(timer); // Inicialmente desativado

  pinMode(Ctrl_Motor_A,OUTPUT);
  pinMode(Ctrl_Motor_H,OUTPUT);

  analogWrite(Ctrl_Motor_A,0);
  analogWrite(Ctrl_Motor_H,0);

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
        encoderPulses = 0;
        analogWrite(Ctrl_Motor_A,65);
        timerAlarmEnable(timer); // Inicia o timer
        Serial.println("Coleta de dados iniciada...");
      } else {
        Serial.println("Coleta já está em andamento. Aguarde o término.");
      }
    } else if (command.equalsIgnoreCase("NEW")) {
      if (!collectingData) {
        Serial.println("Pronto para uma nova coleta. Digite 'START' para começar.");
      } else {
        Serial.println("Coleta em andamento. Aguarde o término.");
      }
    } else {
      Serial.println("Comando não reconhecido. Use 'START' ou 'NEW'.");
    }
  }

  if (collectingData && timerFlag) {
    timerFlag = false; // Limpa a flag da interrupção

    if (sampleIndex < SAMPLES) {
      float ax, ay, az;
      readMPU9250(ax, ay, az); // Leitura do acelerômetro

      accelData[sampleIndex][0] = ax;
      accelData[sampleIndex][1] = ay;
      accelData[sampleIndex][2] = az;

      reconstructSinSignal(sampleIndex); // Reconstrói o sinal senoidal

      sampleIndex++;
    } else {
      //collectingData = false;
      timerAlarmDisable(timer); // Para o timer
      //Serial.println("Coleta de dados concluída.");
      //analogWrite(Ctrl_Motor_A,0);
      //printCollectedData();
      calculatePhaseDifference();
      sampleIndex = 0;
      timerAlarmEnable(timer);
    }
  }
}

///////////////////////// FUNÇÕES ///////////////////////////
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
  analogWrite(Ctrl_Motor_A,0);
  analogWrite(Ctrl_Motor_H,0);
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
  float alpha = 0.1;

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

  filteray = alpha*ay + (1-alpha)*filteray;

  ay = filteray;

  const float scale = 2.0 / 32768.0;
  const float gToMMs2 = 9806.65;

  ax *= scale * gToMMs2;
  ay *= scale * gToMMs2;
  az *= scale * gToMMs2;
}

void reconstructSinSignal(int index) {
  float angle = (float)(encoderPulses % pulsesPerRevolution) / pulsesPerRevolution * 2.0 * PI;
  sinData[index] = garfoAmplitude * sin(angle);
}

void printCollectedData() {
  Serial.println("Dados coletados (Acelerômetro e Sinal Senoidal):");
  for (int i = 0; i < SAMPLES; i++) {
    Serial.print("Amostra ");
    Serial.print(i);
    Serial.print(": Accel X= ");
    Serial.print(accelData[i][0], 2);
    Serial.print(" mm/s², Accel Y= ");
    Serial.print(accelData[i][1], 2);
    Serial.print(" mm/s², Accel Z= ");
    Serial.print(accelData[i][2], 2);
    Serial.print(" mm/s², Sinal Senoidal= ");
    Serial.println(sinData[i], 2);
  }
}

void IRAM_ATTR onTimer() {
  timerFlag = true;
}

void calculatePhaseDifference() {
  // Preenchendo os vetores com os dados coletados
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = accelData[i][1]; // Usa o eixo Y do acelerômetro
    vImag[i] = 0.0; // Parte imaginária zerada
  }

  // Realizando a FFT do acelerômetro
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // Identificando a frequência dominante
  int dominantIndex = FFT.majorPeak();
  double freqAcelerometro = (double)dominantIndex * SAMPLING_FREQUENCY / SAMPLES;
  double phaseAcelerometro = atan2(vImag[dominantIndex], vReal[dominantIndex]);

  // Fazendo a mesma coisa para o sinal reconstruído
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = sinData[i]; // Sinal senoidal reconstruído
    vImag[i] = 0.0;
  }


  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  dominantIndex = FFT.majorPeak();
  double freqReconstruido = (double)dominantIndex * SAMPLING_FREQUENCY / SAMPLES;
  double phaseReconstruido = atan2(vImag[dominantIndex], vReal[dominantIndex]);

  // Calculando a diferença de fase
  double phaseDifference = phaseAcelerometro - phaseReconstruido;

  // Normalizando para o intervalo [-pi, pi]
  if (phaseDifference > PI) {
    phaseDifference -= 2 * PI;
  } else if (phaseDifference < -PI) {
    phaseDifference += 2 * PI;
  }

  // Convertendo para graus
  double phaseDifferenceDegrees = phaseDifference * (180.0 / PI);
  Controle(phaseDifferenceDegrees);

  // Imprimindo os resultados
  /*Serial.print("Frequência do Acelerômetro: ");
  Serial.print(freqAcelerometro);
  Serial.println(" Hz");
  Serial.print("Frequência do Reconstruído: ");
  Serial.print(freqReconstruido);
  Serial.println(" Hz");
  Serial.print("Diferença de Fase: ");
  Serial.print(phaseDifferenceDegrees);*/
  Serial.println(" graus");
}

void Controle(double erro){
  float e = erro-180;

  float c = c_ant + (Kp*e) - (Ki*e_ant);

  c_ant = c;
  e_ant = e;

  if (c<0){
    c=0;
  }

  if (c>180){
    c=180;
  }

  Serial.println(c);

  analogWrite(Ctrl_Motor_A,c);

}