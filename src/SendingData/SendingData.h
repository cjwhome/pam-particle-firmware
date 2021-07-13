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


class SendingData {

public:
    SendingData();
    ~SendingData();

    SendDataToSd(); // done
    SendDataToParticle(); //done
    SendDataToESP(); //done
    SendDataToSensible(); //done

    static SendingData *instance;
    String device_id = "";

private: 
    static SendingData* GetInstance();
    T6713 t6713 = NULL;
    TPHFusion tph_fusion = NULL;
    Plantower plantower = NULL;
    PAMCO pamco = NULL;
    PAMCO pamco2 = NULL;
    PAM_108L pam_108L = NULL;
    int sample_counter = 0;


};

#endif // __SENDING_DATA_H__