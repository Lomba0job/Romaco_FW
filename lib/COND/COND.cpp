#include "COND.hpp"


COND::COND(){
	this->calfact=1;
	this->offset=0;
    this->n_adcs=0;
    this->algo=NULL;
    this->mem=NULL;
}

void COND::init(HX711** adcs, unsigned int n_adcs) {
	this->adcs=adcs;
    this->n_adcs=n_adcs;
	//Initialize corrector factors (to balance via software gain factors differences)
	this->correction_factors = (unsigned int*) malloc(sizeof(unsigned int)*this->n_adcs);
	assert(this->correction_factors!=NULL);
	int i;
	unsigned int max_gain=32;
	for(i=0; i<this->n_adcs; i++)  if(this->adcs[i]->get_gain_value() > max_gain) max_gain=this->adcs[i]->get_gain_value();	//Find max gain
	for(i=0; i<this->n_adcs; i++) this->correction_factors[i]=max_gain/this->adcs[i]->get_gain_value();	//Balance all ADCs to this max gain
	//Save last measures
	this->last_read = (unsigned long*) malloc(sizeof(unsigned long)*this->n_adcs);
	assert(this->last_read!=NULL);
	//Save single offset
	this->cell_offset = (unsigned long*) malloc(sizeof(unsigned long)*this->n_adcs);
	assert(this->cell_offset!=NULL);
	for(int i=0; i<n_adcs; i++) this->cell_offset[i]=0;
	//Save single errors
	this->adcs_errors = (unsigned int*) malloc(sizeof(unsigned int)*this->n_adcs);
	assert(this->adcs_errors!=NULL);
	for(int i=0; i<n_adcs; i++) this->adcs_errors[i]=this->adcs[i]->get_error_count();
}

void COND::set_calfact(float calfact){
	this->calfact=calfact;
    if(this->mem!=NULL) this->mem->saveCalfact(this->get_calfact());
}

void COND::set_offset(){
	unsigned long sum = 0;
	for(int i=0; i<this->n_adcs; i++) sum+=this->get_offset(i);
	this->offset=sum;
    if(this->mem!=NULL) this->mem->saveOffset(this->n_adcs, this->cell_offset);
}

void COND::set_mem(SPIFFS* mem) {
	this->mem=mem;
	if(this->mem!=NULL) {
        this->calfact=this->mem->retrieveCalfact();
        this->cell_offset=this->mem->retrieveOffset(this->n_adcs);
    }
}

unsigned long COND::get_offset(void){
	int i;
	long result=0;
	for(i=0; i<this->n_adcs; i++) {
		result+=this->cell_offset[i];
	}
	return result;
}

unsigned long COND::get_offset(int n_adc){
	if(n_adc>=this->n_adcs) return 0;
	return this->cell_offset[n_adc];
}

float COND::get_calfact(void){
	return this->calfact;
}

unsigned long COND::read(void)
{
	unsigned long result=0;
    unsigned long read=0;
	unsigned long reading=0;
    unsigned int i=0;
    for(i=0;i<this->n_adcs;i++){
		reading = this->adcs[i]->read();
		if(reading==READ_ERROR_CODE) this->adcs_errors[i]++;
        read = reading * this->correction_factors[i]; //<-----------------QUI
		this->last_read[i]=read;
        result+=read;   
    }
    return this->algo->filter(result);
}

unsigned long COND::read_average(unsigned int times) 
{
	unsigned long single_measure = 0;
	unsigned long result = 0;
	unsigned long read = 0;
	unsigned int i = 0, j = 0;
	for (j = 0; j<this->n_adcs; j++) this->last_read[j]=0;
	for (i = 0; i < times; i++) {
		single_measure = 0;
		for (j = 0; j<this->n_adcs; j++) {
			read=this->adcs[j]->read() * this->correction_factors[j]; //<-----------------QUI
			//printf("Cell: %d,Value: %ld\n", j, read);
			this->last_read[j]+=read/times;
			single_measure += read;
		}
		result += this->algo->filter(single_measure)/times;
	}
	return result;
}

float COND::get_units(unsigned int times)
{
	unsigned long avg = this->read_average(times);
	long diff = avg - this->get_offset(); 
	
	return diff / this->calfact;
}

unsigned long COND::tare(unsigned int times) 
{
	unsigned long avg = 0; 
	int i;
	for(i=0; i<this->n_adcs; i++) {
		avg = this->adcs[i]->read_average(times) * this->correction_factors[i];
		this->cell_offset[i] = avg;
	}
	this->set_offset();
	return this->get_offset();
}

float COND::calib(int weight, unsigned int times)
{
	if(weight==0) return this->calfact;
	unsigned long avg=0;
	avg = this->read_average(times);
	long diff = avg - this->offset;
	float calfact = (1.0*(diff)) / (1.0*weight);
	if(calfact!=0) this->set_calfact(calfact);

	return this->get_calfact();
}

unsigned long COND::get_last_read(unsigned int n_adc){
	if(n_adc>=this->n_adcs) return 0;
	return this->last_read[n_adc];
}

float COND::get_last_units(unsigned int n_adc){
	if(n_adc>=this->n_adcs) return 0;
	long diff = this->get_last_read(n_adc) - this->get_offset(n_adc);
	float units = (1.0*diff) / (1.0*this->get_calfact());
	return units;
}

unsigned int COND::check_loadCells(unsigned int err_perc) {
	int i;
	unsigned int result=0;
	for(i=0; (i<this->n_adcs)&&(result==0); i++) {
		//printf("Cella: %d\tOffset: %ld\tLast_read: %ld\t", i, this->get_offset(i), this->get_last_read(i));
		long diff = this->get_offset(i) - this->get_last_read(i);
		float err_perc_calc = (diff) / (1.0*this->get_offset(i)) * 100.0; //Errore percentuale
		err_perc_calc = (err_perc_calc > 0) ? err_perc_calc : (-1.0 * err_perc_calc); //Errore percentuale in valore assoluto
		if(err_perc_calc > err_perc) result=i+1;	//La cella i ha un problema
		//printf("Errore percentuale calcolato: %f\n", err_perc_calc);
	}
	return result;
}

int COND::check_adcs(unsigned int n_adc) {
	int result = 0;
	if(n_adc>=this->n_adcs) return -1;
	if(this->adcs_errors[n_adc] != this->adcs[n_adc]->get_error_count()) result = 1;
	else result = 0;
	this->adcs_errors[n_adc] = this->adcs[n_adc]->get_error_count();
	return result;
}

unsigned int COND::check_adcs(void) {
	for(int i=0; i<this->n_adcs; i++) {
		if(this->check_adcs(i)) return 1;
	}
	return 0;
}

void COND::free_mem(void){
	free(this->last_read);
	free(this->correction_factors);
	free(this->cell_offset);
	for(int i=0; i<this->n_adcs; i++) this->adcs[i]->free_mem();
}
