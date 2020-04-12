/**
 * PAMSensor.h - Created 04/11/2020
 * Author: David Kopala
 * 
 * Provides a common interface for interacting with sensors
 */ 

#ifndef __PAM_SENSOR_H__
#define __PAM_SENSOR_H__

#include "Particle.h"
#include <vector>

#include "../PAMSpecie/PAMSpecie.h"

class PAMSensor {

public:
    PAMSensor();
    ~PAMSensor();

    // Lifecycle Methods
    virtual void start() { Serial.println("[PAMSensor]::start\tNOT IMPLEMENTED"); };
    virtual bool measure() { Serial.println("[PAMSensor]::measure\tNOT IMPLEMENTED"); return false; };
    virtual void sleep() { Serial.println("[PAMSensor]::sleep\tNOT IMPLEMENTED"); };
    virtual void wakeup() { Serial.println("[PAMSensor]::wakeup\tNOT IMPLEMENTED"); };

    char *csvHeader();

    std::vector<PAMSpecie *>* getSpecies() { return &(this->species); }

protected:
    std::vector<PAMSpecie *> species;

private:
    

};

#endif // __PAM_SENSOR_H__