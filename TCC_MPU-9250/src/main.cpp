/* Informações Gerais
Código desenvolvido para realizar leituras de aceleração com o sensor MPU-9250

O presente código conta com a configuração e preparo do sensor MPU-9250 sem a utilização
de bibliotecas, realizando a leitura através de uma interrupção para garantir um taxa
de amostragem fixa, além de demonstrar a configuração do barraemnto do I2C em velocidade
máxima. Para esse código foi utilizado um microcontrolador ESP-32-DEVKIT-V1 (DOIT).

Esse código foi desenvolvido para as atividas de TCC do curso de ENG. Controle e Automação
IFSC - Chapecó 

Desenvolvido por: Eduardo Turcatto
Versão: b1.1.0.v1.0
Data: 22/11/2024
*/
#include <Wire.h>
#include <Arduino.h>

#define MPU9250_ADDR 0x68
#define GYRO_XOUT_H 0x43 
#define GYRO_CONFIG 0x1B

#define VECTOR_SIZE 100 // Tamanho do vetor de dados
#define FILTER_SIZE 10 // Tamanho do filtro de média móvel (quantas leituras manter no buffer)
#define SAMPLE_INTERVAL 10000 // Intervalo de amostragem em microssegundos (10ms)

float gyroZData[VECTOR_SIZE]; // Vetor para armazenar os dados de rotação no eixo Z (giroscópio)
float gyroZFiltered = 0.0; // Giroscópio filtrado em Z
float gyroScale = 131.0; // Sensibilidade para ±250°/s (131 LSB/°/s)
float offsetZ = -0.16; // Offset do giroscópio Z

int dataIndex = 0; // Índice do vetor de dados
bool collectData = false; // Flag para controlar quando coletar dados
bool testInProgress = false; // Flag para saber se o teste está em andamento

hw_timer_t *timer = NULL; // Timer para a interrupção
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Mecanismo para sincronizar interrupções

volatile bool readData = false; // Flag para indicar quando ler os dados

// Buffer para o filtro de média móvel simples
float filterBuffer[FILTER_SIZE];
int filterIndex = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  readData = true; // Sinaliza que a leitura dos dados deve ser feita
  portEXIT_CRITICAL_ISR(&timerMux);
}

void printAndResetTest();
void readGyroData(int16_t *data);
float applyMovingAverageFilter(float input);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configura o MPU-9250
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B); // Registrador de gerenciamento de energia
  Wire.write(0x00); // Ativa o sensor
  Wire.endTransmission();

  // Configura o alcance do giroscópio para ±250°/s
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00); // ±250°/s
  Wire.endTransmission();

  // Configuração do temporizador para interrupção a cada 10ms (10000 microssegundos)
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 microsegundo)
  timerAttachInterrupt(timer, &onTimer, true); // Chama a função onTimer quando o timer dispara
  timerAlarmWrite(timer, SAMPLE_INTERVAL, true); // Interrupção a cada 10ms
  timerAlarmEnable(timer); // Habilita o temporizador

  Serial.println("Digite 'start' para iniciar o teste.");
}

void loop() {
  // Verifica comandos recebidos via Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove espaços ou quebras de linha extras

    if (command == "start" && !testInProgress) {
      Serial.println("Iniciando coleta de dados...");
      collectData = true;
      testInProgress = true; // Marca o teste como em andamento
      dataIndex = 0; // Reinicia o índice
    } else if (command == "stop") {
      Serial.println("Coleta de dados interrompida.");
      collectData = false;
    }
  }

  // Realiza a leitura do giroscópio se solicitado
  if (collectData) {
    if (readData) {
      // A interrupção sinalizou que devemos realizar a leitura
      portENTER_CRITICAL(&timerMux);
      readData = false; // Reseta a flag
      portEXIT_CRITICAL(&timerMux);

      int16_t rawData[3];
      readGyroData(rawData);

      // Converte os valores brutos para °/s e aplica os offsets
      float rawZ = rawData[2] / gyroScale - offsetZ;

      // Aplica o filtro de média móvel simples (SMA) no valor Z
      gyroZFiltered = applyMovingAverageFilter(rawZ);

      // Armazena o valor filtrado no vetor
      gyroZData[dataIndex] = gyroZFiltered;

      dataIndex++;

      // Verifica se o vetor foi preenchido
      if (dataIndex >= VECTOR_SIZE) {
        collectData = false; // Interrompe a coleta
        Serial.println("Vetor de dados cheio.");
        printAndResetTest(); // Exibe os dados e reinicia para o próximo teste
      }
    }
  }
}

void readGyroData(int16_t *data) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(GYRO_XOUT_H); // Registrador do giroscópio X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true); // 6 bytes para X, Y, Z

  data[0] = (Wire.read() << 8) | Wire.read(); // X
  data[1] = (Wire.read() << 8) | Wire.read(); // Y
  data[2] = (Wire.read() << 8) | Wire.read(); // Z
}

// Função do filtro de média móvel simples (SMA)
float applyMovingAverageFilter(float input) {
  // Substitui o valor mais antigo no buffer pelo novo valor
  filterBuffer[filterIndex] = input;

  // Calcula a média do buffer
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += filterBuffer[i];
  }

  // Atualiza o índice do buffer (circular)
  filterIndex = (filterIndex + 1) % FILTER_SIZE;

  return sum / FILTER_SIZE;
}

// Função para imprimir os dados e reiniciar o teste
void printAndResetTest() {
  Serial.println("Dados coletados:");

  // Exibe os dados no vetor
  for (int i = 0; i < VECTOR_SIZE; i++) {
    Serial.printf("Z: %.2f\n", gyroZData[i]);
  }

  // Pergunta ao usuário se deseja iniciar um novo teste
  Serial.println("Digite 'start' para iniciar um novo teste.");

  // Aguarda o usuário para reiniciar o teste
  testInProgress = false;
  collectData = false; // Interrompe a coleta de dados
}
