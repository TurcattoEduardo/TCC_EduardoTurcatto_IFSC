///////////////////////// BIBLIOTECAS /////////////////////////
#include <Arduino.h>

////////////////////////// VARIAVEIS //////////////////////////
//----------------------------------------------------------//
// Pinos Encoders
#define Enc_Canal_A_Pin 14
#define Enc_Canal_B_Pin 32

// Constantes encoder
volatile const double Enc_Pulse_Relv = 1000;
volatile const double Motor_Reduc = 3;

// Variáveis Encoder
volatile long Enc_A_Pulse = 0;
volatile long Enc_B_Pulse = 0;
volatile float Enc_RPS = 0;
volatile bool bFlagEnc = false;

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
uint8_t DutyCicle = 40;

//----------------------------------------------------------//
// Timers

hw_timer_t *t_EncTimer = NULL;

//////////////////// DECLARAÇÃO DE FUNÇÕES ///////////////////
//----------------------------------------------------------//
void CalcRPS();

//----------------------------------------------------------//
// Encoder
void Enc_Canal_A_Counter();
void Enc_Canal_B_Counter();
void IRAM_ATTR Enc_Interrupt();

//////////////////////////// SETUP ///////////////////////////
void setup() {
    Serial.begin(115200);

    //----------------------------------------------------------//
    // Configuração dos PWM
    ledcSetup(Pwm_Chanel_Ant, Pwm_Freq, Pwm_Resulution);
    ledcAttachPin(Exc_Motor_Pin_Ant, Pwm_Chanel_Ant);
    ledcSetup(Pwm_Chanel_Hor, Pwm_Freq, Pwm_Resulution);
    ledcAttachPin(Exc_Motor_Pin_Hor, Pwm_Chanel_Hor);

    // Motor pinMode
    pinMode(Ctrl_Motor_Pin_Ant, OUTPUT);
    pinMode(Ctrl_Motor_Pin_Hor, OUTPUT);

    //----------------------------------------------------------//
    // Encoder
    pinMode(Enc_Canal_A_Pin, INPUT_PULLUP);
    pinMode(Enc_Canal_B_Pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(Enc_Canal_A_Pin), Enc_Canal_A_Counter, RISING);
    attachInterrupt(digitalPinToInterrupt(Enc_Canal_B_Pin), Enc_Canal_B_Counter, RISING);

    //----------------------------------------------------------//
    // Timers
    t_EncTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(t_EncTimer, &Enc_Interrupt, true);
    timerAlarmWrite(t_EncTimer, 10000, true);
    timerAlarmEnable(t_EncTimer);
}

/////////////////////////// LOOP ////////////////////////////
void loop() {
  //----------------------------------------------------------//
  ledcWrite(Pwm_Chanel_Ant, DutyCicle);

  if (bFlagEnc) {
    bFlagEnc = false; // Reseta a flag
    CalcRPS();
    Serial.printf("PWM em: %d | Velocidade do eixo: %.2f RPS\n", DutyCicle, Enc_RPS);
  }
}

///////////////////////// FUNÇÕES ///////////////////////////
//----------------------------------------------------------//
void CalcRPS() {
  Enc_RPS = 100 * (Enc_A_Pulse / Enc_Pulse_Relv); // Velocidade em RPS
  Enc_A_Pulse = 0; // Reseta o contador após o cálculo
}

//----------------------------------------------------------//
// Encoder
void Enc_Canal_A_Counter() {
  Enc_A_Pulse++;
}

void Enc_Canal_B_Counter() {
  Enc_B_Pulse++;
}

void IRAM_ATTR Enc_Interrupt() {
  bFlagEnc = true;
}
