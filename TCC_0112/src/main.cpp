#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mySensor;

const int sampleInterval = 10;
const int sampleCount = 100;
volatile bool collectData = false;
volatile int dataIndex = 0;
float accelZData[sampleCount];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!mySensor.begin()) {
    Serial.println("Falha ao inicializar o sensor MPU6050");
    while (1);
  }
  
  timerBegin(0, 80, true);
  timerAttachInterrupt(0, onTimer, true);
  timerAlarmWrite(0, sampleInterval * 1000, true);
  timerAlarmEnable(0);
  
  Serial.println("Aguardando comando para iniciar...");
  while (!Serial.available()) {}
  Serial.println("Iniciando coleta de dados...");
}

void loop() {
  if (collectData) {
    collectData = false;
    sensors_event_t a, g, temp;
    mySensor.getEvent(&a, &g, &temp);
    accelZData[dataIndex] = a.acceleration.z;
    dataIndex++;

    if (dataIndex >= sampleCount) {
      timerAlarmDisable(0);
      Serial.println("Dados coletados:");
      for (int i = 0; i < sampleCount; i++) {
        Serial.print("Aceleração Z[" + String(i) + "]: ");
        Serial.println(accelZData[i], 4);
      }

      Serial.println("Digite 'start' para reiniciar.");
      while (Serial.available() == 0) {}
      String command = Serial.readString();
      if (command == "start") {
        dataIndex = 0;
        timerAlarmEnable(0);
        Serial.println("Coleta reiniciada...");
      }
    }
  }
}

void IRAM_ATTR onTimer() {
  noInterrupts();
  collectData = true;
  interrupts();
}
