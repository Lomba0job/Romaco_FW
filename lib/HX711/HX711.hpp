
#ifndef HX711_h

#define HX711_h
#define HIGH 1
#define LOW 0
#define CLOCK_DELAY_US 20
#define READ_ERROR_CODE 0 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "ALGO.hpp"

typedef enum HX711_GAIN
{
	eGAIN_128 = 1,
	eGAIN_64 = 3,
	eGAIN_32 = 2
} HX711_GAIN;

class HX711
{
	private:
		gpio_num_t ADC_DT;
		gpio_num_t ADC_CK;
		float calfact=1;
		long offset=0;
		HX711_GAIN GAIN=eGAIN_128;
		unsigned int gain_value=128;
		unsigned int timeout=1;
		unsigned int err_count=0;
		ALGO* algo;

	public: 
		HX711();
		// waits for the chip to be ready and returns a reading.
		// we can select one of the two ADCs: number1==0 or number2==1
		unsigned long read(void);
		// define clock and data pin, channel, and gain factor
		// channel selection is made by passing the appropriate gain: 128 or 64 for channel A, 32 for channel B
		// gain: 128 or 64 for channel A; channel B works with 32 gain factor only
		void init(gpio_num_t adc_dt, gpio_num_t adc_ck);

		// set the gain factor; takes effect only after a call to read()
		// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
		// depending on the parameter, the channel is also set to either A or B
		void set_gain(HX711_GAIN gain);

		// set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
		void set_calfact(float calfact);

		// set OFFSET, the value that's subtracted from the actual reading (tare weight)
		void set_offset(long offset);

		// set timeout for the "read" function
		void set_timeout(unsigned int timeout) {
			 this->timeout=timeout;
		}

		// set the algo to filter the measures
		void set_algo(ALGO* algo){
			this->algo=algo;
		}

		void free_mem(void) {
			this->algo->free_mem();
		}
		
		// get the current SCALE
		float get_calfact();

		// get the current OFFSET
		long get_offset();

		//get the current GAIN
		HX711_GAIN get_gain() {
			return this->GAIN;
		}

		//get the current GAIN value
		unsigned int get_gain_value() {
			return this->gain_value;
		}

		//get the timeout
		unsigned int get_timeout(){
			return this->timeout;
		}

		//get the error count
		unsigned int get_error_count(){
			return this->err_count;
		}

		//reads the two ADCs and return the average reading of "times" measures
		unsigned long read_average(unsigned int times);

		// returns get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
		// times = how many readings to do
		float get_units(unsigned int times);

		// set the OFFSET value for tare weight; times = how many times to read the tare value
		// returns the new OFFSET
		long tare(unsigned int times);

		//set the CALFACT value based on weight provided. 
		float calib(int weight, unsigned int times);

		// check if HX711 is ready
		// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
		// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
		bool is_not_ready();

		bool is_data_ready();
		
		// puts the chip into power down mode
		void power_down();

		// wakes up the chip after power down mode
		void power_up();
};
#endif