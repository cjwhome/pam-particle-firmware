#ifndef __PAM_108L_H__
#define __PAM_108L_H__

#include "Particle.h"

#include "Telaire_T6713.h"

#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../PAMEEPROM/EEPROMAddresses.h"
#include "../../PAMSerial/PAMSerialEditEEPROMValue/PAMSerialEditEEPROMValue.h"

class PAM_108L: public PAMSensor {

public:
    PAM_108L();
    ~PAM_108L();

    bool start();
    bool measure();
    void getEspOzoneData();

    PAMSpecie ozone;
    bool ozone_enabled;

};

#endif // __PAM_T6713_H__