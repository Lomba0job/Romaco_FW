#include "HX711.hpp"
#include <time.h>

HX711::HX711(){
	this->calfact=1;
	this->offset=0;
}

void HX711::init(gpio_num_t adc_dt, gpio_num_t adc_ck) {
	ADC_DT=adc_dt;
	ADC_CK=adc_ck;
	gpio_reset_pin(ADC_DT);
	gpio_reset_pin(ADC_CK);
	if(gpio_set_direction(ADC_DT, GPIO_MODE_INPUT)!=ESP_OK) printf("Errore pin HX711\n");
    if(gpio_set_direction(ADC_CK, GPIO_MODE_OUTPUT)!=ESP_OK) printf("Errore pin HX711\n");
	this->power_up();
}

void HX711::set_gain(HX711_GAIN gain)
{
	this->GAIN = gain;
	if(this->GAIN==eGAIN_128) this->gain_value=128;
	else if(this->GAIN==eGAIN_64) this->gain_value=64;
	else if(this->GAIN==eGAIN_32) this->gain_value=32;
}

void HX711::set_calfact(float calfact){
	this->calfact=calfact;
}

void HX711::set_offset(long offset){
	this->offset=offset;
}

long HX711::get_offset(void){
	return this->offset;
}

float HX711::get_calfact(void){
	return this->calfact;
}

void HX711::power_down() 
{
	gpio_set_level(ADC_CK, LOW);
	ets_delay_us(CLOCK_DELAY_US);
	gpio_set_level(ADC_CK, HIGH);
	ets_delay_us(CLOCK_DELAY_US*3);
}

void HX711::power_up() 
{
	gpio_set_level(ADC_CK, LOW);
}

//When output data is not ready for retrieval, 
//digital output pin DOUT is high
bool HX711::is_not_ready()
{
	return gpio_get_level(this->ADC_DT)==1;
}

//When DOUT goes to low, 
//it indicates data is ready for retrieval
bool HX711::is_data_ready()
{
	return gpio_get_level(this->ADC_DT)==0;
}

unsigned long HX711::read(void)
{
	unsigned long value;

	for(int i=0; i<2; i++){	//DEVO FARE DUE MISURE PERCHE LA PRIMA MI SERVE SOLO PER SETTARE IL GUADAGNO!!!
		gpio_set_level(ADC_CK, LOW);
		// wait for the chip to become ready
		time_t tstart;
		tstart=time(NULL);
		while (this->is_not_ready()) 
		{
			gpio_set_level(ADC_CK, LOW);
			vTaskDelay(10 / portTICK_PERIOD_MS);
			if(difftime(time(NULL), tstart)>this->timeout) {
				this->err_count++;
				return READ_ERROR_CODE;
			}
		}
		value = 0;
		//--- Enter critical section ----
		portDISABLE_INTERRUPTS();
		for(int i=0; i < 24 ; i++)
		{   
			gpio_set_level(ADC_CK, HIGH);
			ets_delay_us(CLOCK_DELAY_US);
			value = value << 1;
			gpio_set_level(ADC_CK, LOW);
			ets_delay_us(CLOCK_DELAY_US);

			if(gpio_get_level(ADC_DT))
				value++;
		}
		// set the channel and the gain factor for the next reading using the clock pin
		for (unsigned int i = 0; i < GAIN; i++) 
		{	
			gpio_set_level(ADC_CK, HIGH);
			ets_delay_us(CLOCK_DELAY_US);
			gpio_set_level(ADC_CK, LOW);
			ets_delay_us(CLOCK_DELAY_US);
		}	
		portENABLE_INTERRUPTS();
		//--- Exit critical section ----
		value =value^0x800000;
	}
	unsigned long result = this->algo->filter(value);
	return (result);
}

unsigned long HX711::read_average(unsigned int times) 
{
	unsigned long sum = 0;
	for (unsigned int i = 0; i < times; i++) 
	{
		sum += this->read();
	}
	return sum / times;
}

float HX711::get_units(unsigned int times)
{
	unsigned long avg = this->read_average(times);
	avg=avg-this->offset;
	
	return avg / this->calfact;
}

long HX711::tare(unsigned int times) 
{
	unsigned long avg = 0; 
	avg = this->read_average(times);
	this->set_offset(avg);
	
	return this->offset;
}

float HX711::calib(int weight, unsigned int times)
{
	unsigned long avg=0;
	avg = this->read_average(times);
	this->calfact=(avg - this->offset) / weight;

	return this->calfact;
}
