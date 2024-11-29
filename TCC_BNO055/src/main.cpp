///////////////////////// BIBLIOTECAS /////////////////////////
#include <Wire.h>
#include <Arduino.h>
////////////////////////// VARIÁVEIS //////////////////////////
// Configurações do sensor
#define BNO055_ADDRESS 0x28  // Endereço I2C do BNO055
#define GYRO_X_LSB 0x14      // Registrador para o giroscópio no eixo X
#define GYRO_Y_LSB 0x16      // Registrador para o giroscópio no eixo Y
#define GYRO_Z_LSB 0x18      // Registrador para o giroscópio no eixo Z

// Variáveis para armazenar as taxas de rotação
float gyroData[3];  // Para armazenar as taxas de rotação (X, Y, Z)

// Variável para controlar a interrupção
hw_timer_t *timer = NULL;
volatile bool dataReady = false; // Flag para indicar que os dados estão prontos

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
void IRAM_ATTR onTimer();               // Função de interrupção do timer
void initBNO055();                      // Função para inicializar o BNO055
void writeRegister(uint8_t reg, uint8_t value); // Função para escrever no registrador
void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length); // Função para ler do registrador
void readGyroscope();                   // Função para ler os dados do giroscópio

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);  // Inicializa o monitor serial

  // Inicializa o barramento I2C
  Wire.begin();

  // Configura o sensor BNO055 (Modo NDOF)
  initBNO055();

  // Configura o temporizador de interrupção para 10ms
  timer = timerBegin(0, 80, true); // Prescaler de 80 para 1 MHz
  timerAttachInterrupt(timer, &onTimer, true); // Chama a função onTimer na interrupção
  timerAlarmWrite(timer, 10000, true); // 10ms de intervalo
  timerAlarmEnable(timer); // Habilita o temporizador
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  // Verifica se os dados estão prontos a cada 10ms
  if (dataReady) {
    dataReady = false;  // Reseta a flag

    // Lê as taxas de rotação do giroscópio
    readGyroscope();

    // Imprime as taxas de rotação do giroscópio (X, Y, Z)
    Serial.print("Gyro X: ");
    Serial.print(gyroData[0]);
    Serial.print(" | Gyro Y: ");
    Serial.print(gyroData[1]);
    Serial.print(" | Gyro Z: ");
    Serial.println(gyroData[2]);
  }
}

///////////////////////// FUNÇÕES ///////////////////////////
// Função de interrupção chamada a cada 10ms
void IRAM_ATTR onTimer() {
  dataReady = true; // Seta a flag indicando que os dados estão prontos
}

// Função para inicializar o sensor BNO055
void initBNO055() {
  delay(10);

  // Coloca o sensor no modo CONFIG para alterar configurações
  writeRegister(0x3D, 0x00); // OPR_MODE para CONFIG_MODE
  delay(10);

  // Habilita o cristal externo para maior precisão
  writeRegister(0x3F, 0x80); // SYS_TRIGGER para ativar o cristal externo
  delay(10);

  // Coloca o sensor em NDOF (Sensor de orientação com todos os sensores ativados)
  writeRegister(0x3D, 0x0C); // OPR_MODE para NDOF
  delay(20);

  // Configuração das unidades para graus por segundo (para o giroscópio)
  writeRegister(0x3B, 0x00); // UNIT_SEL para graus
  delay(10);
}

// Função para ler os dados do giroscópio (X, Y, Z)
void readGyroscope() {
  uint8_t buffer[6];  // Buffer para armazenar os 6 bytes (2 para cada eixo)

  // Lê os dados de giroscópio (X, Y, Z)
  readRegisters(GYRO_X_LSB, buffer, 6);

  // Converte os dados brutos de giroscópio para taxas de rotação (em graus por segundo)
  gyroData[0] = ((int16_t)(buffer[1] << 8 | buffer[0])) / 16.0; // Gyro X
  gyroData[1] = ((int16_t)(buffer[3] << 8 | buffer[2])) / 16.0; // Gyro Y
  gyroData[2] = ((int16_t)(buffer[5] << 8 | buffer[4])) / 16.0; // Gyro Z
}

// Função para ler múltiplos registradores a partir do BNO055
void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);  // Mantenha o barramento ativo para leitura
  Wire.requestFrom(BNO055_ADDRESS, length);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
}

// Função para escrever um valor em um registrador do BNO055
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
