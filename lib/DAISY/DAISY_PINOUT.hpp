#ifndef PINOUT
#define PINOUT
#include "driver/gpio.h"
//Defining pinout
#define led1 GPIO_NUM_25  //Q0_PWM
#define led2 GPIO_NUM_26  //Q1_PWM
// #define esito_lettura GPIO_NUM_27  //Q2
// #define DirMotor GPIO_NUM_12  //Q3
// #define esito_controllo_peso GPIO_NUM_13  //Q4
// #define Q5 GPIO_NUM_2   //Q5
// #define Q6 GPIO_NUM_0   //Q6
#define avvio_lettura GPIO_NUM_34  //I0
// #define reset GPIO_NUM_32  //I2
// #define interruttore GPIO_NUM_33  //I3
// #define Sens1 GPIO_NUM_36  //I4
// #define Sens2 GPIO_NUM_39  //I5
#define ADC1_DATA GPIO_NUM_35
#define ADC1_CLOCK GPIO_NUM_4
#define ADC2_DATA GPIO_NUM_14
#define ADC2_CLOCK GPIO_NUM_15
#define RS485_TX GPIO_NUM_17    //uart 2 tx
#define RS485_RX GPIO_NUM_16    //uart 2 rx
#define RS485_EN GPIO_NUM_5 
#define SD_MISO GPIO_NUM_19
#define SD_MOSI GPIO_NUM_23
#define SD_CLK GPIO_NUM_18
#define SD_CS GPIO_NUM_5

void init_Daisy();

#endif