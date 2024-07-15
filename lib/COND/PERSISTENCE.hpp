/*
Autore: Fabbri Simone
Data: 27/11/2022
Titolo: Classe per gestire la persistenza di dati su memoria EEPROM
Descrizione: Vengono implementate funzioni di salvataggio permanente 
di dati su memoria SPIFFS di ESP32.
*Ultima modifica:
*/
#ifndef PERSISTENCE_H
#define PERSISTENCE_H
#include "esp_spiffs.h"

class SPIFFS{
    private:
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "nvs", //<----------------------------------------QUI
      .max_files = 5,
      .format_if_mount_failed = true
    };
    esp_err_t ret;
    void initializeSPIFFS(void);

    public:
    SPIFFS();
    //Salvataggio fattore di calibrazione (return 0: ok, return 1: errore)
    bool saveCalfact(float calfact);
    //Salvataggio offset (return 0: ok, return 1: errore)
    bool saveOffset(int n_adcs, unsigned long* offset);
    //Ottenimento calfact (1: errore)
    float retrieveCalfact(void);
    //Ottenimento offset (0: errore)
    unsigned long* retrieveOffset(int n_adcs);
    //Check integrità
    bool systemOK(void) {
        return this->ret==ESP_OK;
    }
};
#endif