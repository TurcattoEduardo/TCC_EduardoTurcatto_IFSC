/* Informações Gerais
Código desenvolvido para indetificação de dinâmicas com o microcontrolador ESP-32-DEVKIT-V1

O presente código conta com o ascionamento de dois motor CC utilizando o drive LM298P, onde 
um motor utiliza-se das saídasDAC do microcontroladro e o outro utiliza saídas PWM padrões
configuradas em 8 bits.
Além de contar uma leitura de um sensor MPU-9250 no seu barramento I2C principal e também
a leitura dos canais A e B de um encoder conectado a um dos motores.

Esse código foi desenvolvido para as atividas de TCC do curso de ENG. Controle e Automação
IFSC - Chapecó 

Desenvolvido por: Eduardo Turcatto
Versão: b1.0.1v1
Data: 22/11/2024
*/
///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>

////////////////////////// VARIAVEIS //////////////////////////
//----------------------------------------------------------//
// Pinos Encoders
#define Enc_Canal_A_Pin 14
#define Enc_Canal_B_Pin 32

// Constantes encoder
volatile const double Enc_Pulse_Relv = 400;
volatile const double Motor_Reduc = 3;

// Variáveis Encoder
volatile long Enc_A_Pulse = 0;
volatile long Enc_B_Pulse = 0;
volatile float Enc_RPS = 0;

//----------------------------------------------------------//
// Pinos dos motores
#define Ctrl_Motor_Pin_Hor 25
#define Ctrl_Motor_Pin_Ant 26 
#define Exc_Motor_Pin_Hor 27
#define Exc_Motor_Pin_Ant 33 

// Configuração do PWM
#define Pwm_Freq 5000
#define Pwm_Resulution 8
#define Pwm_Chanel_Hor 0
#define Pwm_Chanel_Ant 1

// Variáveis do motor
volatile float Motor_RPS = 0;

// Matriz de configuração dos motores
volatile uint8_t Motors_Config[2][2];

/*
Motor 1 : Sentido de Giro | Velocidade
Motor 2 : Sentido de Giro | Velocidade
*/
//----------------------------------------------------------//
// Variáveis de Configuração
volatile int iMotorSelect = 0;
volatile int iDirection = 0;
volatile int iDutyCicle = 0;
//----------------------------------------------------------//
// Variáveis de pré definição
volatile int iPreDirection = 0;
volatile int iPreDutyCicle = 128;
//----------------------------------------------------------//
// Flags do Programa
volatile bool bFlagMotorTest = false;
volatile bool bFlagConfig = false;
volatile bool bFlagEnc = false;
volatile bool bFlagOperation = false;

//----------------------------------------------------------//
// Timers

hw_timer_t *t_EncTimer = NULL;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
//----------------------------------------------------------//
void Config();
void Inicialize();
void MotorTest();
void CalcRPS();

//----------------------------------------------------------//
// Encoder
void Enc_Canal_A_Counter();
void Enc_Canal_B_Counter();
void IRAM_ATTR Enc_Interrupt();

//////////////////////////// SETUP ///////////////////////////y
void setup() {
    Serial.begin(115200);

    //----------------------------------------------------------//
    // Configuração dos PWM
    ledcSetup(Pwm_Chanel_Ant, Pwm_Freq, Pwm_Resulution);
    ledcAttachPin(Exc_Motor_Pin_Ant,Pwm_Chanel_Ant);
    ledcSetup(Pwm_Chanel_Hor, Pwm_Freq, Pwm_Resulution);
    ledcAttachPin(Exc_Motor_Pin_Hor,Pwm_Chanel_Hor);

    // Motor pinMode
    pinMode(Ctrl_Motor_Pin_Ant, OUTPUT);
    pinMode(Ctrl_Motor_Pin_Hor, OUTPUT);
    pinMode(Exc_Motor_Pin_Ant, OUTPUT);
    pinMode(Exc_Motor_Pin_Hor, OUTPUT);

    //----------------------------------------------------------//
    // Encoder
    pinMode(Enc_Canal_A_Pin, INPUT_PULLUP);
    pinMode(Enc_Canal_B_Pin, INPUT_PULLUP);

    //----------------------------------------------------------//
    // Timers
    t_EncTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(t_EncTimer, &Enc_Interrupt, true);
    timerAlarmWrite(t_EncTimer, 100000, true);

    //----------------------------------------------------------//
    Inicialize();

    timerAlarmEnable(t_EncTimer);
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
    //----------------------------------------------------------//
    Config();

    //----------------------------------------------------------//
    // Inciiar Teste
    Serial.println("Deseja Testar configuração dos motores ? [Y/N]");

    while(Serial.available() == 0){

    }

    if (Serial.available() > 0){
    String strComand = Serial.readStringUntil('\n');
    strComand.trim();

    if (strComand.equalsIgnoreCase("Y")){
      bFlagMotorTest = true;
      Serial.println("Iniciando Testes. . .");
      delay(500);
    }
    else if (strComand.equalsIgnoreCase("N")){
      bFlagMotorTest = false;
      Serial.println("Deseja alterar as configurações ? [Y/N]");

        while(Serial.available() == 0){

        }

      if (strComand.equalsIgnoreCase("Y")){
        bFlagConfig = true;
        Serial.println("Iniciando configuração Manual . . .");
        delay(500);
        }
        else if (strComand.equalsIgnoreCase("N")){
        bFlagConfig = false;
        delay(1000);
        return;
        }
      return;
    }
  }
  //----------------------------------------------------------//
  MotorTest();

  while(bFlagOperation){
    if (bFlagEnc){
        timerAlarmDisable(t_EncTimer);
        bFlagEnc = false;
        detachInterrupt(Enc_Canal_A_Pin);
        detachInterrupt(Enc_Canal_B_Pin);

        CalcRPS();

        Enc_A_Pulse = 0;
        Enc_B_Pulse = 0;

        attachInterrupt(digitalPinToInterrupt(Enc_Canal_A_Pin), Enc_Canal_A_Counter, FALLING);
        attachInterrupt(digitalPinToInterrupt(Enc_Canal_B_Pin), Enc_Canal_B_Counter, FALLING);
        timerAlarmEnable(t_EncTimer);
    }

  }
}

///////////////////////// FUNÇÕES ///////////////////////////
//----------------------------------------------------------//
void Config(){
    while(bFlagConfig){
        //----------------------------------------------------------//
        // Seleção do motor
        Serial.println("Escolha qual Motor deseja Configurar: [0 - Controle | 1 - Excitação]");
        while (Serial.available() == 0){
            
        }

        if(Serial.available() > 0){
            String strMotorSelect = Serial.readStringUntil('\n');
            strMotorSelect.trim();

            iMotorSelect = strMotorSelect.toInt();

            if(iMotorSelect == 0 || iMotorSelect == 1){
                Serial.print("Motor Selecionado: ");
                Serial.println(iMotorSelect);
            } else{
                Serial.print("Motor não encontrado!");
                return;
            }
            
        }
        //----------------------------------------------------------//
        // Seleção do sentido de giro
        Serial.println("Escolha qual direção o motor deve girar: [0 - Horário | 1 - Anti Horário]");
        while (Serial.available() == 0){
        
        }

        if(Serial.available() > 0){
        String strDirection = Serial.readStringUntil('\n');
        strDirection.trim();

        iDirection = strDirection.toInt();

        if (iDirection >= 1){
            iDirection = 1;
        }

        Motors_Config[iMotorSelect][0] = iDirection;
        
        }
        //----------------------------------------------------------//
        // Seleção da Velocidade
        Serial.println("Informe o Novo Valor para o PWM: (0 até 255)");
        while (Serial.available() == 0){

        }

        if(Serial.available() > 0){
            String strPWMComand = Serial.readStringUntil('\n');
            strPWMComand.trim();

            iDutyCicle = strPWMComand.toInt();

            if (iDutyCicle > 255){
                Serial.println("Valor muito alto limitado em 255!");
                iDutyCicle = 255;
            }else if (iDutyCicle < 0){
                Serial.println("Valor muito baixo limitado em 0!");
                iDutyCicle = 0;
            }

            Motors_Config[iMotorSelect][1] = iDutyCicle;
        }
        //----------------------------------------------------------//
        // Validar Configurações
        Serial.println("A configuração Atual é: ");
        Serial.print("Motor de controle | Sentido de giro: ");
        if (Motors_Config[0][0] == 0){
            Serial.print("Horário");
        }else{
            Serial.print("Antihorário");
        }
        Serial.print(" | DutyCicle: ");
        Serial.println(Motors_Config[0][1]);

        Serial.print("Motor de excitação | Sentido de giro: ");
        if (Motors_Config[1][0] == 0){
            Serial.print("Horário");
        }else{
            Serial.print("Antihorário");
        }
        Serial.print(" | DutyCicle: ");
        Serial.println(Motors_Config[1][1]);

        Serial.println("Deseja alterar as configurações ? [Y/N]");
        while(Serial.available() == 0){

        }
        if (Serial.available() > 0){
            String strComand = Serial.readStringUntil('\n');
            strComand.trim();

            if (strComand.equalsIgnoreCase("Y")){
            bFlagConfig = true;
            }
            else if (strComand.equalsIgnoreCase("N")){
            bFlagConfig = false;
            }
        }
    }
}

void Inicialize(){
    Serial.println("Iniciando o programa. . . ");
    Serial.print("Deseja configurar os motores manualmente ? [Y/N]");
    while (Serial.available() == 0){

    }

    if (Serial.available() > 0){
        String strComand = Serial.readStringUntil('\n');
        strComand.trim();

        if (strComand.equalsIgnoreCase("Y")){
        bFlagConfig = true;
        Serial.println("Iniciando configuração Manual . . .");
        delay(500);
        }
        else if (strComand.equalsIgnoreCase("N")){
        bFlagConfig = false;
        Serial.println("Aplicando Configuração de pré-definição!");
        Motors_Config [0][0] = iPreDirection;
        Motors_Config [1][0] = iPreDirection;
        Motors_Config [0][1] = iPreDutyCicle;
        Motors_Config [1][1] = iPreDutyCicle;

        Serial.println("A configuração Atual é: ");
        Serial.print("Motor de controle | Sentido de giro: ");
        if (Motors_Config[0][0] == 0){
            Serial.print("Horário");
        }else{
            Serial.print("Antihorário");
        }
        Serial.print(" | DutyCicle: ");
        Serial.println(Motors_Config[0][1]);

        Serial.print("Motor de excitação | Sentido de giro: ");
        if (Motors_Config[1][0] == 0){
            Serial.print("Horário");
        }else{
            Serial.print("Antihorário");
        }
        Serial.print(" | DutyCicle: ");
        Serial.println(Motors_Config[1][1]);
        }
    }
}

void MotorTest(){
    while (bFlagMotorTest){
        Serial.println("Aplicando Configuração aos Motores. . .");
        delay(500);
        int iContador = 0;
        attachInterrupt(digitalPinToInterrupt(Enc_Canal_A_Pin), Enc_Canal_A_Counter, FALLING);
        attachInterrupt(digitalPinToInterrupt(Enc_Canal_B_Pin), Enc_Canal_B_Counter, FALLING);
        //----------------------------------------------------------//
        // Motor de controle
        if (Motors_Config[0][0] == 0){
            dacWrite(Ctrl_Motor_Pin_Ant,0);
            dacWrite(Ctrl_Motor_Pin_Hor,Motors_Config[0][1]);
        }else if (Motors_Config[0][0] == 1){
            dacWrite(Ctrl_Motor_Pin_Hor,0);
            dacWrite(Ctrl_Motor_Pin_Ant,Motors_Config[0][1]);
        }
        //----------------------------------------------------------//
        // Motor de Excitação
        if (Motors_Config[1][0] == 0){
            ledcWrite(Pwm_Chanel_Ant,0);
            ledcWrite(Pwm_Chanel_Hor,Motors_Config[1][1]);
        }else if (Motors_Config[1][0] == 1){
            ledcWrite(Pwm_Chanel_Hor,0);
            ledcWrite(Pwm_Chanel_Ant,Motors_Config[1][1]);
        }
        delay(500);
        Serial.println("Deseja alterar as configurações? [Y/N]");

        while (Serial.available() == 0){
            if (bFlagEnc){
                timerAlarmDisable(t_EncTimer);
                bFlagEnc = false;
                detachInterrupt(Enc_Canal_A_Pin);
                detachInterrupt(Enc_Canal_B_Pin);
                iContador++;
                CalcRPS();

                Enc_A_Pulse = 0;
                Enc_B_Pulse = 0;

                attachInterrupt(digitalPinToInterrupt(Enc_Canal_A_Pin), Enc_Canal_A_Counter, FALLING);
                attachInterrupt(digitalPinToInterrupt(Enc_Canal_B_Pin), Enc_Canal_B_Counter, FALLING);
                timerAlarmEnable(t_EncTimer);
            }
            if (iContador >= 10){
                iContador = 0;
                Serial.print("Velocidade do eixo: ");
                Serial.print(Enc_RPS);
                Serial.println(" RPS");
                Serial.print("Velocidade do Motor de controle: ");
                Serial.print(Motor_RPS);
                Serial.println(" RPS");
            }
        }
        
        if (Serial.available() > 0){
            detachInterrupt(Enc_Canal_A_Pin);
            detachInterrupt(Enc_Canal_B_Pin);
            String strComand = Serial.readStringUntil('\n');
            strComand.trim();

            if (strComand.equalsIgnoreCase("Y")){
                bFlagConfig = true;
                Serial.println("Retornando para configurações . . .");
                delay(500);
                bFlagMotorTest = false;
            }
            else if (strComand.equalsIgnoreCase("N")){
                ledcWrite(Pwm_Chanel_Ant,0);
                ledcWrite(Pwm_Chanel_Hor,0);
                dacWrite(Ctrl_Motor_Pin_Ant,0);
                dacWrite(Ctrl_Motor_Pin_Hor,0);
                bFlagConfig = false;
                Serial.println("Teste Finalizado!");
                delay(500);
            }

        }
    }
}

void CalcRPS(){
    Enc_RPS = 10*(Enc_A_Pulse/Enc_Pulse_Relv);
    Motor_RPS = 10*((Enc_A_Pulse*3)/Enc_Pulse_Relv);
}

//----------------------------------------------------------//
// Encoder
void Enc_Canal_A_Counter(){
    Enc_A_Pulse++;
}

void Enc_Canal_B_Counter(){
    Enc_B_Pulse++;
}

void IRAM_ATTR Enc_Interrupt(){
    bFlagEnc = true;
}