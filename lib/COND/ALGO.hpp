/*

*/
#ifndef ALGO_h
#define ALGO_h
#include "assert.h"
#include <math.h>
//INTERFACCIA ALGORITMO DI FILTRAGGIO
class ALGO 
{
    public:
    ALGO(){}

    virtual unsigned long filter(unsigned long measure)=0;
    virtual void free_mem();
};
//ALGORITMO DUMMY: RESTITUISCE LA MISURA
class Dummy:public ALGO{
    public:
    Dummy(){}
    unsigned long filter(unsigned long measure) {
        return measure;
    }
    void free_mem(){}
};

//ALGORITMO1: MEDIA MOBILE
class MediaMobile:public ALGO{
    private:
    unsigned int n_samples;
    unsigned long *samples;
    unsigned int i;
    void allocateMemory(void){
        assert(this->n_samples>0);
        this->samples=(unsigned long*) malloc(sizeof(unsigned long)*this->n_samples);
        assert(this->samples!=NULL);
        this->i=0;
    }

    public:
    MediaMobile(){
        this->n_samples=1;
        this->allocateMemory();
    }
    MediaMobile(unsigned int n_samples){
        this->n_samples=n_samples;
        this->allocateMemory();
    }
    unsigned long filter(unsigned long measure) {
        int j=0;
        unsigned long sum=0;
        if(this->i < this->n_samples) { //Vector not full
            samples[i]=measure;
            i++;
        }
        else { //Vector already full
            for(j=0; j<n_samples-1; j++){ //Left shift vector with new entrance
                this->samples[j]=this->samples[j+1];
            }
            this->samples[n_samples-1]=measure;
        }
        //Evaluate the mean
        for(j=0;j<this->i;j++) sum+=this->samples[j];
        return sum/this->i;
    }
    void free_mem(){
        free(this->samples);
    }
};

//ALGORITMO2: MEDIANA
class Mediana:public ALGO{
    private:
    unsigned int n_samples;
    unsigned long *samples;
    unsigned long *temp;
    unsigned int i;
    void allocateMemory(void){
        assert(this->n_samples>0);
        this->samples=(unsigned long*) malloc(sizeof(unsigned long)*this->n_samples);
        assert(this->samples!=NULL);
        this->temp=(unsigned long*) malloc(sizeof(unsigned long)*this->n_samples);
        assert(this->temp!=NULL);
        for(int j=0; j<this->n_samples; j++) this->samples[j]=0;
        this->i=0;
    }

    public:
    Mediana(){
        this->n_samples=1;
        this->allocateMemory();
    }
    Mediana(unsigned int n_samples){
        this->n_samples=n_samples;
        this->allocateMemory();
    }
    unsigned long filter(unsigned long measure) {
        unsigned long result = 0;
        int j=0;
        if(this->i < this->n_samples) { //Vector not full
            samples[this->i]=measure;
            this->i++;
        }
        else { //Vector already full
            for(j=0; j<this->n_samples-1; j++){ //Left shift vector with new entrance
                this->samples[j]=this->samples[j+1];
            }
            this->samples[this->n_samples-1]=measure;
        }
        //Evaluate the median of vector samples*: [0], [1], ..., [nsamples-2], [nsamples-1]
        memcpy(this->temp, this->samples, sizeof(unsigned long) * this->n_samples);
        if(i<n_samples) result = measure;
        else result = quicksort(this->temp, 0, this->n_samples, this->n_samples/2); 
        return result;
    }
    void swap(unsigned long *arr,int i,int j)
    {
        unsigned long t;
        t=arr[i];
        arr[i]=arr[j];
        arr[j]=t;
    }
    int partition(unsigned long *arr,int l,int r)
    {
        int i=l-1,pivot=arr[r];
        for(int j=l;j<r;j++)
        {
            if(arr[j]<=pivot)
                {
                    ++i;
                    swap(arr,i,j);
                }
        }
        swap(arr,i+1,r);
        return i+1;
    }
    unsigned long quicksort(unsigned long *arr,int l,int r,int k)
    {
        int pi=partition(arr,l,r);
        if(pi+1==k)
        {
            return arr[pi];
        }
        if(pi+1>k)
        {
            return quicksort(arr,l,pi-1,k);
        }
        else
        {
            return quicksort(arr,pi+1,r,k);
        }
    }
    void printArray(unsigned long* arr) {
        for(int i=0; i<this->n_samples; i++) {
            printf("%ld\t", arr[i]);
        }
        printf("\n");
    }
    void free_mem() {
        free(this->samples);
        free(this->temp);
    }
};

//ALGORITMO3: BESSEL IMA
class BesselIMA:public ALGO{
    private:
    unsigned int order;
    unsigned int cutoff;
    double *xv;
    double *yv;
    unsigned int i;
    const double GAIN = 1.626925725e+04;
    void allocateMemory(void){
        assert(this->order>0);
        this->xv=(double*) malloc(sizeof(double)*(this->order+1));
        assert(this->xv!=NULL);
        this->yv=(double*) malloc(sizeof(double)*(this->order+1));
        assert(this->yv!=NULL);
        this->i=0;
    }

    public:
    BesselIMA(){
        this->order=4;
        this->cutoff=5;
        this->allocateMemory();
    }
    unsigned long filter(unsigned long measure){
        if(this->i < (this->order+1)) { //Vectors not full
            xv[this->i] = measure / this->GAIN;
            yv[this->i] = measure / 1.0;
            this->i++;
            return measure;
        }
        else { //Vector already full
            xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
            xv[4] = measure / this->GAIN;
            yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
            yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
                + ( -0.5517688291 * yv[0]) + (  2.5441192827 * yv[1])
                + ( -4.4173936876 * yv[2]) + (  3.4240597840 * yv[3]);
            return long(yv[4]);
        }
    }
    void free_mem(){
        free(this->xv);
        free(this->yv);
    }
};

//ALGORITMO4: STABILITY RESEARCH
/*
Creazione di un filtro Bessel non solo immune ai disturbi ad alta frequenza,
ma anche immune a misure "statisticamente improbabili" tramite il filtro a Mediana Mobile
*/
class Combination:public ALGO{
    private:
    ALGO* internalFilter;
    ALGO* externalFilter;
    public:
    Combination(){
    }
    Combination(ALGO* internalFilter, ALGO* externalFilter){
        this->internalFilter=internalFilter;
        this->externalFilter=externalFilter;
    }
    unsigned long filter(unsigned long measure) {
        return this->externalFilter->filter(this->internalFilter->filter(measure));
    }
    void free_mem(){
        this->internalFilter->free_mem();
        this->externalFilter->free_mem();
    }
};

class BesselICAM:public ALGO{
    private:
    unsigned int order;
    unsigned int cutoff;
    double *xv;
    double *yv;
    unsigned int i;
    const double k = 1.4142135623730951;
    const double g = tan(M_PI * 20.0 / 4);
    const double R = 1.0 / (2.0 * g);
    const double k1 = 1.0 / (1.0 + k * R + R * R);

    void allocateMemory(void){
        assert(this->order>0);
        this->xv=(double*) malloc(sizeof(double)*(this->order));
        assert(this->xv!=NULL);
        this->yv=(double*) malloc(sizeof(double)*(this->order));
        assert(this->yv!=NULL);
        this->i=0;
    }

    void init(void) {
        assert(this->xv!=NULL);
        assert(this->yv!=NULL);
        for(int i=0; i<this->order; i++) {
            this->xv[i]=0.0;
            this->yv[i]=0.0;
        }
    }

    public:
    BesselICAM(){
        this->order=2;
        this->cutoff=20;
        this->allocateMemory();
        this->init();
    }
    unsigned long filter(unsigned long measure){
        double output = k1 * (measure + 2.0 * this->xv[0] + this->xv[1] - (k * R - R * R) * this->yv[0] - (-2.0 * R * R) * this->yv[1]);

        this->xv[1] = this->xv[0];
        this->xv[0] = measure;
        this->yv[1] = this->yv[0];
        this->yv[0] = output;

        return output;
    }
    void free_mem(){
        free(this->xv);
        free(this->yv);
    }
};


#endif