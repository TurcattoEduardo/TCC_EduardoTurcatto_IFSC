#include "MessageFormatter.h"

// Instância da biblioteca
MessageFormatter formatter;

// Variável para controlar envio de dados
bool sendData = false;

void setup() {
    Serial.begin(9600);
    Serial.println("ESP32 pronto. Aguarde o comando START.");
}

void loop() {
    // Verifica se recebeu um comando
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove espaços em branco extras

        if (command == "START") {
            sendData = true;
            Serial.println("Comando START recebido. Enviando dados...");
        } else if (command == "STOP") {
            sendData = false;
            Serial.println("Comando STOP recebido. Parando envio de dados...");
        }
    }

    // Envia os dados somente se o comando START foi recebido
    if (sendData) {
        // Gera valores aleatórios
        int x = random(-100, 100);
        int y = random(-100, 100);
        int z = random(-100, 100);
        int rps = random(0, 50);
        float encsin = random(0, 10000) / 10000.0;

        // Cria e envia mensagens formatadas
        String mpuMessage = formatter.createMessage("MPU", "X=" + String(x) + " Y=" + String(y) + " Z=" + String(z));
        String rpsMessage = formatter.createMessage("RPS", String(rps));
        String encsinMessage = formatter.createMessage("ENCSIN", String(encsin, 4));

        Serial.println(mpuMessage);
        delay(500);

        Serial.println(rpsMessage);
        delay(500);

        Serial.println(encsinMessage);
        delay(500);
    }
}
