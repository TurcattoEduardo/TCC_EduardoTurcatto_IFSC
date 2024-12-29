#include "SerialProtocol.h"
#include <string.h> // Para memcpy

// Implementação da função para calcular o CRC
uint16_t calculate_crc(const uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // Polinômio usado no CRC-16
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Implementação da função para encapsular mensagens
int encapsulate_message(uint8_t prefix, uint8_t identifier, const void *payload, uint8_t payload_size, uint8_t *buffer) {
    if (payload_size > MAX_PAYLOAD_SIZE) {
        return -1; // Erro: tamanho do payload excede o limite
    }

    uint8_t index = 0;

    // Adiciona o prefixo
    buffer[index++] = prefix;

    // Adiciona o identificador
    buffer[index++] = identifier;

    // Adiciona o tamanho do payload
    buffer[index++] = payload_size;

    // Copia o payload para o buffer
    memcpy(&buffer[index], payload, payload_size);
    index += payload_size;

    // Calcula o CRC
    uint16_t crc = calculate_crc(buffer, index);

    // Adiciona o CRC ao buffer
    buffer[index++] = crc & 0xFF;        // Byte menos significativo
    buffer[index++] = (crc >> 8) & 0xFF; // Byte mais significativo

    return index; // Retorna o tamanho total da mensagem
}

// Implementação da função para decodificar mensagens
bool decode_message(const uint8_t *buffer, uint8_t buffer_size, Message *msg) {
    if (buffer_size < (HEADER_SIZE + CRC_SIZE)) {
        return false; // Erro: mensagem muito curta
    }

    uint8_t index = 0;

    // Lê o prefixo
    msg->prefix = buffer[index++];

    // Lê o identificador
    msg->identifier = buffer[index++];

    // Lê o tamanho do payload
    msg->payload_size = buffer[index++];

    if (msg->payload_size > MAX_PAYLOAD_SIZE || 
        (HEADER_SIZE + msg->payload_size + CRC_SIZE) != buffer_size) {
        return false; // Erro: tamanho inválido
    }

    // Copia o payload
    memcpy(msg->payload, &buffer[index], msg->payload_size);
    index += msg->payload_size;

    // Lê o CRC recebido
    uint16_t received_crc = buffer[index++];
    received_crc |= (buffer[index++] << 8);

    // Calcula o CRC do restante da mensagem
    uint16_t calculated_crc = calculate_crc(buffer, buffer_size - CRC_SIZE);

    // Verifica o CRC
    if (received_crc != calculated_crc) {
        return false; // Erro: CRC inválido
    }

    // Tudo validado
    msg->crc = received_crc;
    return true;
}
