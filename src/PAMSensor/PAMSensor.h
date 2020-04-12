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

    enum SensorState { UNINITIALIZED, IDLE, MEASURING, SLEEPING, ERROR };
    SensorState state = SensorState::UNINITIALIZED;

    char *name = "";

    // Lifecycle Methods
    virtual bool start() { Serial.println("[PAMSensor]::start\tNOT IMPLEMENTED"); return false; };
    virtual bool stop() { Serial.println("[PAMSensor]::start\tNOT IMPLEMENTED"); return false; }
    virtual bool measure() { Serial.println("[PAMSensor]::measure\tNOT IMPLEMENTED"); return false; };
    virtual bool sleep() { Serial.println("[PAMSensor]::sleep\tNOT IMPLEMENTED"); return false; };
    virtual bool wakeup() { Serial.println("[PAMSensor]::wakeup\tNOT IMPLEMENTED"); return false; };

    char *csvHeader();

    std::vector<PAMSpecie *>* getSpecies() { return &(this->species); }

protected:
    std::vector<PAMSpecie *> species;

private:
    

};

#endif // __PAM_SENSOR_H__