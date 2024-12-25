#include "MessageFormatter.h"

// Construtor
MessageFormatter::MessageFormatter() {}

// Método para calcular o CRC
uint8_t MessageFormatter::calculateCRC(const String &data) {
    uint8_t crc = 0;
    for (size_t i = 0; i < data.length(); i++) {
        crc ^= data[i];  // XOR simples para cada caractere
    }
    return crc;
}

// Método para criar mensagem formatada
String MessageFormatter::createMessage(const String &tag, const String &data) {
    uint8_t crc = calculateCRC(data);  // Calcula o CRC do dado
    return "[" + tag + "] " + data + "[" + String(crc) + "]";
}
