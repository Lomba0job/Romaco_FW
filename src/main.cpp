#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "mbcontroller.h"
#include "modbus_params.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "DAISY_PINOUT.hpp"
#include "HX711.hpp"
#include "COND.hpp"
#include "ALGO.hpp"
#include "PERSISTENCE.hpp"

#define MB_SLAVE_ADDR 1
#define MB_PORT_NUM 2
#define MB_DEV_SPEED 9600
#define MB_PARITY MB_PARITY_NONE
#define sample_freq 5
#define times 5
#define buffer 10
#define wait_time 1
#define discharge_time 1
#define err_perc 10

static const char *SLAVE_TAG = "MODBUS_SLAVE";
static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;
int err = 0;

extern "C" void app_main(void) {
    init_Daisy();
    esp_log_level_set(SLAVE_TAG, ESP_LOG_INFO);

    uart_config_t uart_config = {
        .baud_rate = MB_DEV_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = MB_PARITY,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(MB_PORT_NUM, &uart_config);
    uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD, CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_driver_install(MB_PORT_NUM, 256, 256, 10, NULL, 0));

    void *handler = NULL;
    mb_communication_info_t comm_info = {
        .mode = MB_MODE_RTU,
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
        .start_offset = MAX_SCALES * sizeof(holding_reg_params_t),
        .address = (void *)&number_of_scales,
        .size = sizeof(number_of_scales),
    };

    ESP_ERROR_CHECK(mbcontroller_init(&comm_info, &handler));
    ESP_ERROR_CHECK(mbcontroller_set_descriptor(handler, &reg_area_holding));
    ESP_ERROR_CHECK(mbcontroller_set_descriptor(handler, &reg_area_coils));
    ESP_ERROR_CHECK(mbcontroller_set_descriptor(handler, &reg_area_number_of_scales));
    ESP_ERROR_CHECK(mbcontroller_start(handler));

    int32_t cell1_data = 0, cell2_data = 0, cell3_data = 0, cell4_data = 0, pesoTot_data = 0;
    int32_t pesoDX = 0, pesoSX = 0, pesoCalib = 0, imbalanceLimit = 0, maximumLimit = 0;
    uint16_t storeID = 0, workplaceID = 0;
    bool prevTare = 0, prevCalib = 0, checkCells = 0;
    char stato = 'A';
    int wei = 0;
    time_t tStart, tStart_check;
    holding_reg_params.diagnostic = 0;
    holding_reg_params.connection_dummy = 1;

    HX711 *cond1 = new HX711();
    cond1->init(ADC1_DATA, ADC1_CLOCK);
    cond1->set_gain(eGAIN_64);
    cond1->set_algo(new Combination(new Mediana(buffer), new MediaMobile(5)));
    HX711 *cond2 = new HX711();
    cond2->init(ADC1_DATA, ADC1_CLOCK);
    cond2->set_gain(eGAIN_32);
    cond2->set_algo(new Combination(new Mediana(buffer), new MediaMobile(5)));
    HX711 *cond3 = new HX711();
    cond3->init(ADC2_DATA, ADC2_CLOCK);
    cond3->set_gain(eGAIN_64);
    cond3->set_algo(new Combination(new Mediana(buffer), new MediaMobile(5)));
    HX711 *cond4 = new HX711();
    cond4->init(ADC2_DATA, ADC2_CLOCK);
    cond4->set_gain(eGAIN_32);
    cond4->set_algo(new Combination(new Mediana(buffer), new MediaMobile(5)));

    COND *bilancia = new COND();
    HX711 **conds = (HX711 **)malloc(sizeof(HX711 *) * 4);
    conds[0] = cond1;
    conds[1] = cond2;
    conds[2] = cond3;
    conds[3] = cond4;
    bilancia->init(conds, 4);
    ALGO *algo = new Dummy();
    bilancia->set_algo(algo);
    SPIFFS *mem = new SPIFFS();

    int o = 0;

    while (1) {
        pesoTot_data = bilancia->get_units(1);
        o = o + 1;
        cell1_data = bilancia->get_last_units(0);
        cell2_data = bilancia->get_last_units(1);
        cell3_data = bilancia->get_last_units(2);
        cell4_data = bilancia->get_last_units(3);
        pesoDX = cell1_data + cell2_data;
        pesoSX = cell3_data + cell4_data;

        portENTER_CRITICAL(&param_lock);
        coil_reg_params.coil_PresenceStatus = !gpio_get_level(avvio_lettura);
        portEXIT_CRITICAL(&param_lock);

        if (coil_reg_params.coil_TareCommand == 1) {
            if (prevTare == 0) {
                coil_reg_params.coil_LastCommandSuccess = 0;
                bilancia->tare(times * 2);
                printf("Tare Command requested!\n");
                prevTare = 1;
                coil_reg_params.coil_LastCommandSuccess = 1;
            }
        } else prevTare = 0;

        if (coil_reg_params.coil_CalibCommand == 1) {
            if (prevCalib == 0) {
                coil_reg_params.coil_LastCommandSuccess = 0;
                pesoCalib = holding_reg_params.holding_pesoCalibMS * (2 ^ 16) + holding_reg_params.holding_pesoCalibLS;
                bilancia->calib(pesoCalib, times * 2);
                printf("Calib Command requested!: %f\n", bilancia->get_calfact());
                prevCalib = 1;
                coil_reg_params.coil_LastCommandSuccess = 1;
            }
        } else prevCalib = 0;

        if (coil_reg_params.coil_CalibCommand == 1) {
            if (stato == 'A') {
                printf("richiesto il peso...\n");
                tStart = time(NULL);
                o = 0;
                stato = 'B';
            } else if (stato == 'B') {
                if (difftime(time(NULL), tStart) > wait_time) {
                    printf("Tempo di attesa passato :\n");
                    printf("Numero misure fatte: %d\n", o);
                    wei = (int)pesoTot_data;
                    printf("Valore i: %d", wei);

                    portENTER_CRITICAL(&param_lock);
                    holding_reg_params.holding_cell1LS = uint16_t (cell1_data);
                    holding_reg_params.holding_cell1MS = cell1_data >> 16;
                    holding_reg_params.holding_cell2LS = uint16_t(cell2_data);
                    holding_reg_params.holding_cell2MS = cell2_data >> 16;
                    holding_reg_params.holding_cell3LS = uint16_t(cell3_data);
                    holding_reg_params.holding_cell3MS = cell3_data >> 16;
                    holding_reg_params.holding_cell4LS = uint16_t(cell4_data);
                    holding_reg_params.holding_cell4MS = cell4_data >> 16;
                    holding_reg_params.holding_pesoTotLS = uint16_t(pesoTot_data);
                    holding_reg_params.holding_pesoTotMS = pesoTot_data >> 16;
                    holding_reg_params.count = holding_reg_params.count + 1;
                                    if (bilancia->check_adcs(0)) holding_reg_params.diagnostic |= 1UL << 0;
                else holding_reg_params.diagnostic &= ~(1UL << 0);
                    if (bilancia->check_adcs(1)) holding_reg_params.diagnostic |= 1UL << 1;
                    else holding_reg_params.diagnostic &= ~(1UL << 1);
                    if (bilancia->check_adcs(2)) holding_reg_params.diagnostic |= 1UL << 2;
                    else holding_reg_params.diagnostic &= ~(1UL << 2);
                    if (bilancia->check_adcs(3)) holding_reg_params.diagnostic |= 1UL << 3;
                    else holding_reg_params.diagnostic &= ~(1UL << 3);
                    if (!mem->systemOK()) holding_reg_params.diagnostic |= 1UL << 4;
                    else holding_reg_params.diagnostic &= ~(1UL << 4);
                    portEXIT_CRITICAL(&param_lock);

                stato = "A";
                coil_reg_params.coil_CalibCommand = 0;
                coil_reg_params.coil_LastCommandSuccess = 1;
            }
        }
    }
    if (checkCells == 1 && difftime(time(NULL), tStart_check) > discharge_time && coil_reg_params.coil_PresenceStatus == 0) {
        printf("Controllando stato celle\n");
        unsigned int result_cells = bilancia->check_loadCells(err_perc);
        if (result_cells > 0) {
            printf("Errore celle\n");
            portENTER_CRITICAL(&param_lock);
            coil_reg_params.coil_CellStatus = 1;
            if (result_cells == 1) holding_reg_params.diagnostic |= 1UL << 5;
            else holding_reg_params.diagnostic &= ~(1UL << 5);
            if (result_cells == 2) holding_reg_params.diagnostic |= 1UL << 6;
            else holding_reg_params.diagnostic &= ~(1UL << 6);
            if (result_cells == 3) holding_reg_params.diagnostic |= 1UL << 7;
            else holding_reg_params.diagnostic &= ~(1UL << 7);
            if (result_cells == 4) holding_reg_params.diagnostic |= 1UL << 8;
            else holding_reg_params.diagnostic &= ~(1UL << 8);
            portEXIT_CRITICAL(&param_lock);
        } else {
            portENTER_CRITICAL(&param_lock);
            coil_reg_params.coil_CellStatus = 0;
            portEXIT_CRITICAL(&param_lock);
        }
        printf("Controllando stato adcs\n");
        unsigned int result_adcs = bilancia->check_adcs();
        if (result_adcs > 0) {
            printf("Errore adcs\n");
            portENTER_CRITICAL(&param_lock);
            coil_reg_params.coil_AdcsStatus = 1;
            portEXIT_CRITICAL(&param_lock);
        } else {
            portENTER_CRITICAL(&param_lock);
            coil_reg_params.coil_AdcsStatus = 0;
            portEXIT_CRITICAL(&param_lock);
        }
        printf("Tutto ok\n");
        checkCells = 0;
    }
}

ESP_LOGI(SLAVE_TAG, "Modbus controller destroyed.");
vTaskDelay(100);
ESP_ERROR_CHECK(mbc_slave_destroy());

algo->free_mem();
bilancia->free_mem();
free(conds);