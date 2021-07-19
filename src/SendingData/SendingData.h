#ifndef __SENDING_DATA_H__
#define __SENDING_DATA_H__

#include "Particle.h"

#include "../PAMEEPROM/EEPROMAddresses.h"
#include "../Sensors/108L/108L.h"
#include "../Sensors/TPHFusion/TPHFusion.h"
#include "../Sensors/PAMCO/PAMCO.h"
#include "../Sensors/Plantower/Plantower.h"
#include "../Sensors/T6713/T6713.h"
#include "../PAMSensorManager/PAMSensorManager.h"
#include "../PAMSensor/PAMSensor.h"
#include "BLEconstants.h"
#include "gps.h"
#include "../global.h"
#include "SdFat.h"


class SendingData {

public:
    static SendingData* GetInstance();


    void SendDataToSd();
    void SendDataToParticle();
    void SendDataToESP(); 
    void SendDataToSensible();

private: 
    static SendingData *instance;
    SendingData();
    ~SendingData();

    T6713 * t6713 = NULL;
    TPHFusion * tph_fusion = NULL;
    Plantower * plantower = NULL;
    PAMCO * pamco = NULL;
    PAMCO * pamco2 = NULL;
    PAM_108L * pam_108L = NULL;
    int sample_counter = 0;
    GPS gps;
    FuelGauge fuel;
    char geolocation_latitude[12] = "999.9999999";
    char geolocation_longitude[13] = "99.9999999";
    char geolocation_accuracy[6] = "255.0";
    Global * globalVariables = Global::GetInstance();
    PAMSensorManager *sensorManager = PAMSensorManager::GetInstance();



};

#endif // __SENDING_DATA_H__