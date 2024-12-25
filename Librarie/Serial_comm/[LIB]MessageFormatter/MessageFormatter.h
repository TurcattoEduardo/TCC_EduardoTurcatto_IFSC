#ifndef MESSAGEFORMATTER_H
#define MESSAGEFORMATTER_H

#include <Arduino.h>

class MessageFormatter {
public:
    // Construtor
    MessageFormatter();

    // Método para criar mensagem formatada
    String createMessage(const String &tag, const String &data);

private:
    // Método para calcular o CRC
    uint8_t calculateCRC(const String &data);
};

#endif
