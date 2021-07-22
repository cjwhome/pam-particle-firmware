#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "PAMEEPROM/EEPROMAddresses.h"
#include "SdFat.h"

#define CS A2               //Chip select for SPI/uSD card

class Global {

public:
    static Global* GetInstance();
    void writeLogFile(String data);

    int device_id;
    bool ozone_enabled;
    String fileName;
    bool cellular_enabled;
    bool sensible_iot_en;
    bool esp_wifi_connection_status = false;
    bool debugging_enabled;

    SdFat sd;
    SdFile file;
    SdFile log_file;
    File file1;
    String logFileName;
    bool file_started = false;
    bool log_file_started = false;

    union{
        uint16_t status_int;
        char byte[2];
    } * status_word;

    private: 
        static Global *instance;
        Global();
        ~Global();


};

#endif // __SENDING_DATA_H__