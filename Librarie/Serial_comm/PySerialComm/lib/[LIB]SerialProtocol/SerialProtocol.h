#ifndef SERIALPROTOCOL_H
#define SERIALPROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// Definições de constantes
#define CRC_SIZE 2            // Tamanho do CRC em bytes
#define MAX_PAYLOAD_SIZE 20   // Tamanho máximo do payload (ex.: 3 floats)
#define HEADER_SIZE 3         // Tamanho do cabeçalho (prefixo + identificador + tamanho)

// Identificadores de prefixos
#define PREFIX_DATA 0x01  // %SGL
#define PREFIX_CMD  0x02  // %CMD
#define PREFIX_ERR  0x03  // %ERR
#define PREFIX_INF  0x04  // %INF

// Identificadores de dados (%ACC%, %RPS%, %ENC%)
#define ID_ACC 0xA1  // Acelerômetro
#define ID_RPS 0xB1  // Velocidade (RPS)
#define ID_ENC 0xB2  // Posição do encoder

// Identificadores de comandos (%CON%, %EXC%)
#define ID_CON 0xC1  // Configuração
#define ID_EXC 0xC2  // Execução

// Identificadores de erros (%VLL%, %EXC%, %TMP%)
#define ID_ERR_VLL 0xD1  // Erro de valor
#define ID_ERR_EXC 0xD2  // Erro de execução
#define ID_ERR_TMP 0xD3  // Erro de tempo

// Identificadores de informações (%EXC%, %DBG%)
#define ID_INF_EXC 0xE1  // Execução
#define ID_INF_DBG 0xE2  // Debug

// Estrutura da mensagem
typedef struct {
    uint8_t prefix;                // Prefixo (ex.: %SGL, %CMD)
    uint8_t identifier;            // Identificador (ex.: %ACC%, %RPS%)
    uint8_t payload_size;          // Tamanho do payload em bytes
    uint8_t payload[MAX_PAYLOAD_SIZE]; // Dados do payload (binário)
    uint16_t crc;                  // CRC para validação
} Message;

// Funções da biblioteca

/**
 * @brief Encapsula dados em uma mensagem formatada.
 * 
 * @param prefix Prefixo da mensagem (ex.: PREFIX_DATA, PREFIX_CMD).
 * @param identifier Identificador da mensagem (ex.: ID_ACC, ID_RPS).
 * @param payload Ponteiro para os dados do payload.
 * @param payload_size Tamanho dos dados do payload.
 * @param buffer Buffer de saída para a mensagem formatada.
 * @return int Tamanho total da mensagem formatada ou -1 em caso de erro.
 */
int encapsulate_message(uint8_t prefix, uint8_t identifier, const void *payload, uint8_t payload_size, uint8_t *buffer);

/**
 * @brief Decodifica uma mensagem recebida.
 * 
 * @param buffer Dados recebidos (mensagem completa).
 * @param buffer_size Tamanho do buffer recebido.
 * @param msg Estrutura onde a mensagem decodificada será armazenada.
 * @return bool true se a mensagem foi decodificada com sucesso, false caso contrário.
 */
bool decode_message(const uint8_t *buffer, uint8_t buffer_size, Message *msg);

/**
 * @brief Calcula o CRC para um conjunto de dados.
 * 
 * @param data Dados para os quais calcular o CRC.
 * @param length Tamanho dos dados.
 * @return uint16_t CRC calculado.
 */
uint16_t calculate_crc(const uint8_t *data, uint8_t length);

#endif
