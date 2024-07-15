/*
Autore: Fabbri Simone
Data: 21/11/2022
Titolo: Libreria di controllo di un condizionatore di segnale basato su diversi ADC HX711
Descrizione: Implementazione operazioni di tara, calibrazione e ottenimento peso totale
*/
#ifndef COND_h
#define COND_h
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "HX711.hpp"
#include "ALGO.hpp"
#include "PERSISTENCE.hpp"

class COND
{
	private:
		HX711** adcs;
		unsigned long* last_read;
		ALGO* algo;
		SPIFFS* mem;
		unsigned int* adcs_errors;
        unsigned int n_adcs;
		unsigned int *correction_factors;
		float calfact=1;
		unsigned long offset=0;
		unsigned long* cell_offset;

	public: 
		COND();
		//Initialize the object with several params:
        //-adcs: array of HX711 adcs;
        //-n_adcs: number of HX711 adcs;
        //-adc_sfigato: ADC that has half the gain of the others
		void init(HX711** adcs, unsigned int n_adcs);
        // set the ALGO of the weighting scale to use to filter measures
		void set_algo(ALGO* algo){
			this->algo=algo;
		}
		// set the PERSISTENCE used to save data
		void set_mem(SPIFFS* mem);
		// set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
		void set_calfact(float calfact);
		// set OFFSET, the value that's subtracted from the actual reading (tare weight)
		void set_offset();
		// get the current SCALE
		float get_calfact();
		// get the current OFFSET
		unsigned long get_offset();
		unsigned long get_offset(int n_adc);
		// waits for the chips to be ready and returns a reading.
		unsigned long read(void);
		//reads the ADCs and return the average reading of "times" measures
		unsigned long read_average(unsigned int times);
		// returns get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
		// times = how many readings to do
		float get_units(unsigned int times);
		// set the OFFSET value for tare weight; times = how many times to read the tare value
		// returns the new OFFSET
		unsigned long tare(unsigned int times);
		//set the CALFACT value based on weight provided. 
		float calib(int weight, unsigned int times);
		//Retrieve single last reads
		unsigned long get_last_read(unsigned int n_adc);
		float get_last_units(unsigned int n_adc);
		//Check if load cells are ok (0=ok, >0=not ok)
		unsigned int check_loadCells(unsigned int err_perc);
		//Check if a specified adc have worked correctly in the last measures (0 ok, > 0 not ok)
		int check_adcs(unsigned int n_adc);
		//Check if adcs have worked correctly in the last measures (0 ok, > 0 not ok)
		unsigned int check_adcs(void);
		//Release the heap memory previously allocated
		void free_mem(void);
};
#endif