#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS

#include <stdint.h>
#include <stddef.h>

#ifndef NUMBER_OF_SCALES
#define NUMBER_OF_SCALES 1  // Default to 1 if not defined
#endif

#pragma pack(push, 1)
typedef struct
{
    uint8_t coil_PesoCommand;
    uint8_t coil_TareCommand;
    uint8_t coil_CalibCommand;
    uint8_t coil_LastCommandSuccess;
    uint8_t coil_PresenceStatus;
    uint8_t coil_CellStatus;
    uint8_t coil_AdcsStatus;
} coil_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint16_t holding_cell1MS;
    uint16_t holding_cell1LS;
    uint16_t holding_cell2MS;
    uint16_t holding_cell2LS;
    uint16_t holding_cell3MS;
    uint16_t holding_cell3LS;
    uint16_t holding_cell4MS;
    uint16_t holding_cell4LS;
    uint16_t holding_pesoTotMS;
    uint16_t holding_pesoTotLS;
    uint16_t holding_pesoCalibMS;
    uint16_t holding_pesoCalibLS;
    uint16_t connection_dummy;
    uint16_t diagnostic;
} holding_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint16_t input_test;
} input_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t discrete_test;
} discrete_reg_params_t;
#pragma pack(pop)

// Definisce la dimensione massima possibile per il numero di scale
#define MAX_SCALES 6

holding_reg_params_t holding_reg_params[MAX_SCALES];
input_reg_params_t input_reg_params;
coil_reg_params_t coil_reg_params[MAX_SCALES];
discrete_reg_params_t discrete_reg_params;

// Parametri di comunicazione
#define MB_PORT_NUM 2       // Numero di porta UART usata per la connessione Modbus
#define MB_SLAVE_ADDR 1     // Indirizzo del dispositivo nella rete Modbus
#define MB_DEV_SPEED 9600   // Velocità di comunicazione della UART
#define CONFIG_MB_COMM_MODE_RTU 1
#define CONFIG_MB_UART_TXD RS485_TX
#define CONFIG_MB_UART_RXD RS485_RX
#define CONFIG_MB_UART_RTS RS485_EN

// Indirizzi iniziali per ogni tipo di parametro Modbus
#define HOLD_OFFSET(scale, field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1) + (scale) * sizeof(holding_reg_params_t) / 2)
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_COILS_START(scale) ((scale) * sizeof(coil_reg_params_t))
#define MB_REG_HOLDING_START(scale) (HOLD_OFFSET(scale, holding_cell1MS))

// Parametri del protocollo Modbus
#define MB_PAR_INFO_GET_TOUT 10  // Timeout per ottenere le informazioni del parametro
#define MB_CHAN_DATA_MAX_VAL 6
#define MB_CHAN_DATA_OFFSET 0.2f
#define MB_READ_MASK (MB_EVENT_INPUT_REG_RD | MB_EVENT_HOLDING_REG_RD | MB_EVENT_DISCRETE_RD | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK (MB_READ_MASK | MB_WRITE_MASK)

#endif // _DEVICE_PARAMS