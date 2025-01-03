#include <Arduino.h>
#include "SerialProtocol.h"

#define MAX_BUFFER_SIZE 256
uint8_t buffer[MAX_BUFFER_SIZE];

// Estados
bool is_sending = false;

void setup() {
    Serial.begin(115200); // Inicializa a comunicação serial
    randomSeed(analogRead(0)); // Configura a semente para números aleatórios
}

void send_data(uint8_t prefix, uint8_t identifier, const void *payload, uint8_t payload_size) {
    int message_length = encapsulate_message(prefix, identifier, payload, payload_size, buffer);
    if (message_length > 0) {
        Serial.write(buffer, message_length);
    }
}

void process_command(const uint8_t *buffer, uint8_t buffer_size) {
    Message msg;
    if (decode_message(buffer, buffer_size, &msg)) {
        if (msg.identifier == ID_CON) {
            if (strncmp((char *)msg.payload, "start", msg.payload_size) == 0) {
                is_sending = true;
                Serial.println("Iniciando envio de dados.");
            } else if (strncmp((char *)msg.payload, "stop", msg.payload_size) == 0) {
                is_sending = false;
                Serial.println("Parando envio de dados.");
            }
        }
    }
}

void loop() {
    // Processa comandos recebidos
    if (Serial.available()) {
        uint8_t received_buffer[MAX_BUFFER_SIZE];
        int bytes_read = Serial.readBytes(received_buffer, sizeof(received_buffer));
        process_command(received_buffer, bytes_read);
    }

    // Envia dados apenas se o envio estiver ativo
    if (is_sending) {
        float accel_x = random(-100, 100) / 10.0;
        float accel_y = random(-100, 100) / 10.0;
        float accel_z = random(-100, 100) / 10.0;
        uint16_t rps = random(0, 1000);
        uint16_t enc = random(0, 500);

        uint8_t payload_acc[12];
        memcpy(payload_acc, &accel_x, sizeof(float));
        memcpy(payload_acc + 4, &accel_y, sizeof(float));
        memcpy(payload_acc + 8, &accel_z, sizeof(float));
        send_data(PREFIX_DATA, ID_ACC, payload_acc, sizeof(payload_acc));

        uint8_t payload_rps[2];
        memcpy(payload_rps, &rps, sizeof(uint16_t));
        send_data(PREFIX_DATA, ID_RPS, payload_rps, sizeof(payload_rps));

        uint8_t payload_enc[2];
        memcpy(payload_enc, &enc, sizeof(uint16_t));
        send_data(PREFIX_DATA, ID_ENC, payload_enc, sizeof(payload_enc));

        delay(1000); // Envia os dados a cada 1 segundo
    }
}
