#include <Arduino.h>
#include "SerialProtocol.h"
#include <stdlib.h> // Para geração de números aleatórios

// Buffer para envio e recepção
uint8_t send_buffer[256];
uint8_t receive_buffer[256];

void generate_random_data(float *data, int size) {
    for (int i = 0; i < size; i++) {
        data[i] = random(0, 1000) / 100.0; // Gera números float aleatórios entre 0.00 e 10.00
    }
}

void setup() {
    Serial.begin(115200); // Inicializa o monitor serial
    Serial.println("Iniciando geração de mensagens...");
}

void loop() {
    // Gera dados aleatórios
    float random_data[3];
    generate_random_data(random_data, 3);

    // Encapsula a mensagem com os dados gerados
    int message_size = encapsulate_message(PREFIX_DATA, ID_ACC, random_data, sizeof(random_data), send_buffer);

    if (message_size > 0) {
        // Exibe a mensagem encapsulada no formato hexadecimal
        Serial.print("Mensagem Encapsulada (HEX): ");
        for (int i = 0; i < message_size; i++) {
            Serial.printf("%02X ", send_buffer[i]);
        }
        Serial.println();

        // Decodifica a mensagem para simular a recepção
        Message msg;
        if (decode_message(send_buffer, message_size, &msg)) {
            Serial.println("Mensagem Decodificada:");
            Serial.printf("Prefixo: 0x%02X, Identificador: 0x%02X, Tamanho do Payload: %d\n",
                          msg.prefix, msg.identifier, msg.payload_size);

            // Exibe o payload decodificado
            float *decoded_data = (float *)msg.payload;
            for (int i = 0; i < msg.payload_size / sizeof(float); i++) {
                Serial.printf("Valor %d: %.2f\n", i + 1, decoded_data[i]);
            }
        } else {
            Serial.println("Erro ao decodificar a mensagem.");
        }
    } else {
        Serial.println("Erro ao encapsular a mensagem.");
    }

    delay(1000); // Aguarda 1 segundo antes de gerar outra mensagem
}
