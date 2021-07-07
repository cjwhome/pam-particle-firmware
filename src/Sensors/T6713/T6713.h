#ifndef __PAM_T6713_H__
#define __PAM_T6713_H__

#include "Particle.h"

#include "Telaire_T6713.h"

#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../PAMEEPROM/EEPROMAddresses.h"
#include "../../PAMSensorManager/PAMSensorManager.h"

#define LOW_PRESSURE_LIMIT (100)
#define HIGH_PRESSURE_LIMIT (1500)
#define SEALEVELPRESSURE_HPA (1013.25)

class T6713: public PAMSensor {

public:
    T6713();
    ~T6713();

    bool start();
    bool measure();

    PAMSpecie CO2;
    bool pressure_correct = false;

    Telaire_T6713 _t6713;
    PAMSensorManager *manager = PAMSensorManager::GetInstance();

};

#endif // __PAM_T6713_H__