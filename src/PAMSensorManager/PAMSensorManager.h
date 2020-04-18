/**
 * PAMSensor.h - Created on 04/11/2020
 * Author: David Kopala
 * 
 * Provides a singleton for managing sensor modules
 */

#ifndef __PAM_SENSOR_MANAGER_H__
#define __PAM_SENSOR_MANAGER_H__

#include "Particle.h"
#include <vector>

#include "../PAMSensor/PAMSensor.h"
#include "../PAMSpecie/PAMSpecie.h"

class PAMSensorManager {

private:
    static PAMSensorManager *instance;
    PAMSensorManager();
    ~PAMSensorManager();

    std::vector<PAMSensor *> sensors;

    uint64_t last_loop_ms = 0;

public:
    static PAMSensorManager* GetInstance();

    void addSensor(PAMSensor *sensor);
    char *csvHeader();
    std::vector<PAMSpecie *>* findSpeciesForName(char *name);

    void sleep();
    void wakeup();

    void loop();
    uint16_t measurement_period_ms = 5000;

};

#endif // __PAM_SENSOR_MANAGER_H__