#include "DAISY_PINOUT.hpp"

void init_Daisy(){
    gpio_reset_pin(led1);
    gpio_reset_pin(led2);
    // gpio_reset_pin(DirMotor);
    // gpio_reset_pin(Q5);
    // gpio_reset_pin(esito_controllo_peso);
    // gpio_reset_pin(esito_lettura);
    // gpio_reset_pin(Q6);
    gpio_reset_pin(avvio_lettura);
    // gpio_reset_pin(reset);
    // gpio_reset_pin(interruttore);
    // gpio_reset_pin(Sens1);
    // gpio_reset_pin(Sens2);
    gpio_reset_pin(ADC1_DATA);
    gpio_reset_pin(ADC1_CLOCK);
    gpio_reset_pin(ADC2_DATA);
    gpio_reset_pin(ADC2_CLOCK);
    gpio_reset_pin(RS485_EN);
    gpio_set_direction(led1, GPIO_MODE_OUTPUT);
    gpio_set_direction(led2, GPIO_MODE_OUTPUT);
    // gpio_set_direction(Q5, GPIO_MODE_OUTPUT);
    // gpio_set_direction(DirMotor, GPIO_MODE_OUTPUT);
    // gpio_set_direction(esito_lettura, GPIO_MODE_OUTPUT);
    // gpio_set_direction(esito_controllo_peso, GPIO_MODE_OUTPUT);
    // gpio_set_direction(Q6, GPIO_MODE_OUTPUT);
    gpio_set_direction(avvio_lettura, GPIO_MODE_INPUT);
    // gpio_set_direction(reset, GPIO_MODE_INPUT);
    // gpio_set_direction(interruttore, GPIO_MODE_INPUT);
    // gpio_set_direction(Sens1, GPIO_MODE_INPUT);
    // gpio_set_direction(Sens2, GPIO_MODE_INPUT);
    gpio_set_direction(ADC1_DATA, GPIO_MODE_INPUT);
    gpio_set_direction(ADC1_CLOCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADC2_DATA, GPIO_MODE_INPUT);
    gpio_set_direction(ADC2_CLOCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(RS485_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(led1, 0);
    gpio_set_level(led2, 0);
    // gpio_set_level(Q6, 0);
    // gpio_set_level(Q5, 0);
    // gpio_set_level(DirMotor, 0);
    // gpio_set_level(esito_controllo_peso, 0);
    // gpio_set_level(esito_lettura, 0);
}