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
    uint8_t coil_Config;
    uint8_t status_input;
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
    uint16_t count;
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

extern holding_reg_params_t holding_reg_params;
extern input_reg_params_t input_reg_params;
extern coil_reg_params_t coil_reg_params;
extern discrete_reg_params_t discrete_reg_params;

//DEFINIZIONE PARAMETRI COMUNICAZIONE
#define MB_PORT_NUM     2   // Number of UART port used for Modbus connection
#define MB_SLAVE_ADDR   1     // The address of device in Modbus network
#define MB_DEV_SPEED    9600  // The communication speed of the UART
#define CONFIG_MB_COMM_MODE_RTU 1
#define CONFIG_MB_UART_TXD RS485_TX
#define CONFIG_MB_UART_RXD RS485_RX
#define CONFIG_MB_UART_RTS RS485_EN
 
//DEFINIZIONE START ADDRESS PER OGNI TIPO DI PARAMETRO MODBUS
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) >> 1))
#define MB_REG_COILS_START                  (0x0000)
#define MB_REG_HOLDING_START                (HOLD_OFFSET(holding_cell1MS))

//DEFINIZIONE PARAMETRI PROTOCOLLO MODBUS
#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_CHAN_DATA_MAX_VAL                (6)
#define MB_CHAN_DATA_OFFSET                 (0.2f)
#define MB_READ_MASK                        (MB_EVENT_INPUT_REG_RD \
                                                | MB_EVENT_HOLDING_REG_RD \
                                                | MB_EVENT_DISCRETE_RD \
                                                | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR \
                                                | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)

#endif // !defined(_DEVICE_PARAMS)