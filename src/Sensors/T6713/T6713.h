#ifndef __PAM_T6713_H__
#define __PAM_T6713_H__

#include "Particle.h"

#include "../../PAMSensor/PAMSensor.h"

class T6713: public PAMSensor {

public:
    T6713();
    ~T6713();

    void start();

};

#endif // __PAM_T6713_H__