/* Informações Gerais
Código desenvolvido para realizar leituras de aceleração com o sensor MPU-9250

O presente código conta com a configuração e preparo do sensor MPU-9250 sem a utilização
de bibliotecas, realizando a leitura através de uma interrupção para garantir um taxa
de amostragem fixa, além de demonstrar a configuração do barraemnto do I2C em velocidade
máxima. Para esse código foi utilizado um microcontrolador ESP-32-DEVKIT-V1 (DOIT).

Esse código foi desenvolvido para as atividas de TCC do curso de ENG. Controle e Automação
IFSC - Chapecó 

Desenvolvido por: Eduardo Turcatto
Versão: b1.0.0.v0.1
Data: 22/11/2024
*/
///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>
#include <Wire.h>

////////////////////////// VARIAVEIS //////////////////////////
//----------------------------------------------------------//
// Configuração I2C
#define MPU_SDA 21
#define MPU_SCL 22

// Configuração do Sensor
#define MPU_Address 0x68
#define MPU_Acc_Config 0x1C
#define MPU_Acc_Xout 0X3B

// Variáveis
volatile bool bFlagSensorRead = false;
int iVetorSize = 2000;
int Index = 0;
float *MPUAccX, *MPUAccY, *MPUAccZ;
bool bFlagRunnig = false;

//----------------------------------------------------------//
// Timer
hw_timer_t *t_MPUTimer = NULL;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
//----------------------------------------------------------//
// Sensor
void MPU_Inicialize();
void MPU_Read(float &AccX, float &AccY, float &AccZ);

//----------------------------------------------------------//
// Timer
void IRAM_ATTR MPU_Interrupt();

//////////////////////////// SETUP ///////////////////////////
void setup() {
  Serial.begin(115200);
  //----------------------------------------------------------//
  // I2C
  Wire.begin(MPU_SDA,MPU_SCL);
  MPU_Inicialize();

  Serial.println("Sensor incializado!");

  MPUAccX = new float[iVetorSize];
  MPUAccY = new float[iVetorSize];
  MPUAccZ = new float[iVetorSize];

  //----------------------------------------------------------//
  // Timer
  t_MPUTimer = timerBegin(0,80,true);
  timerAttachInterrupt(t_MPUTimer, &MPU_Interrupt, true);
  timerAlarmWrite(t_MPUTimer, 10, true);
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  if (!bFlagRunnig){
    Serial.println("Deseja iniciar a coleta de dados ? [Y/N]");

    while (Serial.available() == 0){

    }

    if (Serial.available() > 0){
      String strComand = Serial.readStringUntil('\n');
      strComand.trim();

      if (strComand.equalsIgnoreCase("Y")){
        Serial.println("Iniciando coleta de dados, aguarde!");
        bFlagRunnig = true;
        timerAlarmEnable(t_MPUTimer);
      }
      else if (strComand.equalsIgnoreCase("N")){
        Serial.println("Código encerrado, press reset para reiniciar!");
        delay(500000);
        return;
      }
    }
  }

  if (bFlagSensorRead){
    timerAlarmDisable(t_MPUTimer);
    bFlagSensorRead = false;
    if (Index < iVetorSize){
      float ax,ay,az;
      MPU_Read(ax,ay,az);

      MPUAccX[Index] = ax;
      MPUAccY[Index] = ay;
      MPUAccZ[Index] = az;
      Index++;
      timerAlarmEnable(t_MPUTimer);
    }else{
      Serial.println("Coleta finalizada. Imprimindo dados na tela . . .");
      delay(500);
      for (int i = 0; i < iVetorSize; i++){
        Serial.print(MPUAccX[i]/16384.0);
        Serial.print(" ");
        Serial.print(MPUAccY[i]/16384.0);
        Serial.print(" ");
        Serial.println(MPUAccZ[i]/16384.0);
      }
      Serial.println("Deseja Realizar nova coleta de dados ? [Y/N]");

      while (Serial.available() == 0){

      }

      if (Serial.available() > 0){
        String strComand = Serial.readStringUntil('\n');
        strComand.trim();

        if (strComand.equalsIgnoreCase("Y")){
          Index = 0;
          Serial.println("Iniciando coleta de dados! Aguarde . . .");
          timerAlarmEnable(t_MPUTimer);
        }
        else if (strComand.equalsIgnoreCase("N")){
          Serial.println("Código encerrado, presione reset para reiniciar!");
          delay(500000);
          return;
        }
      }
    }

  }
  
}

///////////////////////// FUNÇÕES ///////////////////////////
//----------------------------------------------------------//
// Timer
void IRAM_ATTR MPU_Interrupt(){
  bFlagSensorRead = true;
}

// Sensor
void MPU_Read(float &AccX, float &AccY, float &AccZ){
  Wire.beginTransmission(MPU_Address);
  Wire.write(MPU_Acc_Xout);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_Address,6);
  if (Wire.available()==6){
    AccX = (Wire.read() << 8 | Wire.read());
    AccY = (Wire.read() << 8 | Wire.read());
    AccZ = (Wire.read() << 8 | Wire.read());
  }
}

void MPU_Inicialize(){
  Wire.begin(MPU_SDA, MPU_SCL);

    // Wake up do sensor
    Wire.beginTransmission(MPU_Address);
    Wire.write(0x6B); // Registro de Power Management
    Wire.write(0x00); // Wake-up
    Wire.endTransmission();

    // Configurar a escala do acelerômetro (±2g)
    Wire.beginTransmission(MPU_Address);
    Wire.write(MPU_Acc_Config);
    Wire.write(0x00); // ±2g
    Wire.endTransmission();

}