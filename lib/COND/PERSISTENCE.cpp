#include "PERSISTENCE.hpp"

void SPIFFS::initializeSPIFFS(void){
    ret = esp_vfs_spiffs_register(&(this->conf));
    if (ret != ESP_OK) { //Controllo corretta registrazione e montaggio
        if (ret == ESP_FAIL) printf("Failed to mount or format filesystem\n");
        else if (ret == ESP_ERR_NOT_FOUND) printf("Failed to find SPIFFS partition\n");
        else printf("Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
    }
	else {	//Controllo integrità
		ret = esp_spiffs_check(conf.partition_label);
        if (ret != ESP_OK) printf("SPIFFS_check() failed (%s)\n", esp_err_to_name(ret));
        else printf("SPIFFS initialized successfully\n");
    }
}

SPIFFS::SPIFFS(void){
    this->initializeSPIFFS();
}

bool SPIFFS::saveCalfact(float calfact){
    FILE *f = fopen("/spiffs/calfact.bin", "wb");
    if(f==NULL) return 1;
    fwrite(&calfact, sizeof(calfact), 1, f);
    fclose(f);
    return 0;
}

bool SPIFFS::saveOffset(int n_adcs, unsigned long* offset){
    FILE *f = fopen("/spiffs/offset.bin", "wb");
    if(f==NULL) return 1;
    if(offset==NULL) return 1;
    fwrite(&n_adcs, sizeof(n_adcs), 1, f);
    for(int i=0; i<n_adcs; i++) {
        fwrite(&(offset[i]), sizeof(offset[i]), 1, f);
    }
    fclose(f);
    return 0;
}

float SPIFFS::retrieveCalfact(void){
    printf("Retrieving calfact\n");
    float calfact;
    FILE *f = fopen("/spiffs/calfact.bin", "rb");
    if(f==NULL) {
        printf("Could not find Calib data\n");
        return 1;
    }
    fread(&calfact, sizeof(calfact), 1, f);
    fclose(f);
    printf("Correctly retrieved calfact: %f\n", calfact);
    return calfact;
}

unsigned long* SPIFFS::retrieveOffset(int n_adcs){
    printf("Retrieving offset\n");
    unsigned long* offset = (unsigned long*) malloc(sizeof(unsigned long)*n_adcs);
    int n_adcs_saved=0;
    assert(offset!=NULL);
    FILE *f = fopen("/spiffs/offset.bin", "rb");
    if(f==NULL) {
        printf("Could not find Offset data\n");
        for(int i=0; i<n_adcs; i++) offset[i]=0;
        return offset;
    }
    fread(&(n_adcs_saved), sizeof(n_adcs_saved), 1, f);
    if(n_adcs!=n_adcs_saved) return 0;
    for(int i=0; i<n_adcs; i++) fread(&(offset[i]), sizeof(offset[i]), 1, f);
    fclose(f);
    printf("Correctly retrieved offset\n");
    return offset;
}

bool SPIFFS::saveImbalanceLimit(int32_t imbalanceLimit) {
    printf("Saving imbalance limit\n");
    FILE *f = fopen("/spiffs/imbalance.bin", "wb");
    if(f==NULL) return 1;
    fwrite(&imbalanceLimit, sizeof(imbalanceLimit), 1, f);
    fclose(f);
    return 0;
}

int32_t SPIFFS::retrieveImbalanceLimit(void) {
    printf("Retrieving imbalance limit\n");
    int32_t imbalanceLimit;
    FILE *f = fopen("/spiffs/imbalance.bin", "rb");
    if(f==NULL) {
        printf("Could not find imbalance limit data\n");
        return 0;
    }
    fread(&(imbalanceLimit), sizeof(imbalanceLimit), 1, f);
    fclose(f);
    printf("Correctly retrieved imbalance limit\n");
    return imbalanceLimit;
}

bool SPIFFS::saveMaximumLimit(int32_t maximumLimit) {
    printf("Saving maximum limit\n");
    FILE *f = fopen("/spiffs/maximum.bin", "wb");
    if(f==NULL) return 1;
    fwrite(&maximumLimit, sizeof(maximumLimit), 1, f);
    fclose(f);
    return 0;
}

int32_t SPIFFS::retrieveMaximumLimit(void) {
    printf("Retrieving maximum limit\n");
    int32_t maximumLimit;
    FILE *f = fopen("/spiffs/maximum.bin", "rb");
    if(f==NULL) {
        printf("Could not find maximum limit data\n");
        return 0;
    }
    fread(&(maximumLimit), sizeof(maximumLimit), 1, f);
    fclose(f);
    printf("Correctly retrieved imbalance limit\n");
    return maximumLimit;
}

bool SPIFFS::saveStoreID(uint16_t storeID) {
    printf("Saving storeID\n");
    FILE *f = fopen("/spiffs/storeID.bin", "wb");
    if(f==NULL) return 1;
    fwrite(&storeID, sizeof(storeID), 1, f);
    fclose(f);
    return 0;
}

uint16_t SPIFFS::retrieveStoreID(void) {
    printf("Retrieving store ID\n");
    uint16_t storeID;
    FILE *f = fopen("/spiffs/storeID.bin", "rb");
    if(f==NULL) {
        printf("Could not find store ID data\n");
        return 0;
    }
    fread(&(storeID), sizeof(storeID), 1, f);
    fclose(f);
    printf("Correctly retrieved store ID\n");
    return storeID;
}

bool SPIFFS::saveWorkplaceID(uint16_t workplaceID) {
    printf("Saving workplaceID\n");
    FILE *f = fopen("/spiffs/workplaceID.bin", "wb");
    if(f==NULL) return 1;
    fwrite(&workplaceID, sizeof(workplaceID), 1, f);
    fclose(f);
    return 0;
}

uint16_t SPIFFS::retrieveWorkplaceID(void) {
    printf("Retrieving workplaceID\n");
    uint16_t workplaceID;
    FILE *f = fopen("/spiffs/workplaceID.bin", "rb");
    if(f==NULL) {
        printf("Could not find workplaceID data\n");
        return 0;
    }
    fread(&(workplaceID), sizeof(workplaceID), 1, f);
    fclose(f);
    printf("Correctly retrieved workplaceID\n");
    return workplaceID;
}