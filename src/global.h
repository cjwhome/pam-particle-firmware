#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "PAMEEPROM/EEPROMAddresses.h"
#include "SdFat.h"

#define CS A2               //Chip select for SPI/uSD card
#define APP_VERSION 8
#define BUILD_VERSION 1

class Global {

public:
    static Global* GetInstance();
    void writeLogFile(String data);



    int device_id = -1;
    bool ozone_enabled = false;
    String fileName =  "";
    bool cellular_enabled = true;
    bool sensible_iot_en = false;
    bool esp_wifi_connection_status = false;
    bool debugging_enabled = false;
    bool temperature_units = false;
    bool car_topper = false;
    int app_version = APP_VERSION;
    int build_version = BUILD_VERSION;
    time_t car_topper_time = 0;

    SdFat sd;
    SdFile file;
    SdFile log_file;
    File file1;
    String logFileName = "";
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