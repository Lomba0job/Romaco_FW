/**
 * @file main.cpp
 * @description 
 * @date 15/07/2024
 * @author Lombardi Michele 
 * @copyright Nanolever 
 * @version alpha 0.0.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "device_params.h"
#include "modbus_params.h"  // Include la tua libreria Modbus

// Definisce l'ID dello slave Modbus
#define MB_SLAVE_ADDR 1

// Definisce il numero di porta UART e i parametri di comunicazione
#define MB_PORT_NUM 2
#define MB_DEV_SPEED 9600
#define MB_PARITY MB_PARITY_NONE

// Tag per il logging
static const char *TAG = "MODBUS_SLAVE";

void app_main(void) {
    // Configurazione della porta UART per la comunicazione Modbus
    uart_config_t uart_config = {
        .baud_rate = MB_DEV_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = MB_PARITY,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configura i parametri UART
    uart_param_config(MB_PORT_NUM, &uart_config);

    // Imposta i pin UART
    uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD, CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);

    // Installa il driver UART
    ESP_ERROR_CHECK(uart_driver_install(MB_PORT_NUM, 256, 256, 10, NULL, 0));

    // Inizializza il controller Modbus
    void *handler = NULL;
    mb_communication_info_t comm_info = {
        .comm_mode = MB_MODE_RTU,
        .slave_addr = MB_SLAVE_ADDR,
        .port = MB_PORT_NUM,
        .baudrate = MB_DEV_SPEED,
        .parity = MB_PARITY,
    };

    mb_register_area_descriptor_t reg_area_holding = {
        .type = MB_PARAM_HOLDING,
        .start_offset = 0,
        .address = (void *)&holding_reg_params[0],
        .size = sizeof(holding_reg_params),
    };

    mb_register_area_descriptor_t reg_area_coils = {
        .type = MB_PARAM_COIL,
        .start_offset = 0,
        .address = (void *)&coil_reg_params[0],
        .size = sizeof(coil_reg_params),
    };

    mb_register_area_descriptor_t reg_area_number_of_scales = {
        .type = MB_PARAM_HOLDING,
        .start_offset = MAX_SCALES * sizeof(holding_reg_params_t),  // Offset per il registro del numero di scale
        .address = (void *)&number_of_scales,
        .size = sizeof(number_of_scales),
    };

    // Inizializza il controller Modbus
    ESP_ERROR_CHECK(mbcontroller_init(&comm_info, &handler));

    // Imposta le aree dei registri
    ESP_ERROR_CHECK(mbcontroller_set_descriptor(handler, &reg_area_holding));
    ESP_ERROR_CHECK(mbcontroller_set_descriptor(handler, &reg_area_coils));
    ESP_ERROR_CHECK(mbcontroller_set_descriptor(handler, &reg_area_number_of_scales));

    // Avvia il controller Modbus
    ESP_ERROR_CHECK(mbcontroller_start(handler));

    // Ciclo principale
    while (1) {
        // Qui puoi aggiornare i tuoi holding_reg_params e coil_reg_params se necessario

        // Log di debug
        ESP_LOGI(TAG, "Modbus slave in esecuzione");

        // Ritardo
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Ferma il controller Modbus (questa linea non verrà mai raggiunta in questo esempio)
    ESP_ERROR_CHECK(mbcontroller_stop(handler));
    ESP_ERROR_CHECK(mbcontroller_destroy(handler));
}