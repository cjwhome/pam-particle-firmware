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

#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"

class PAMSensorManager {

private:
    static PAMSensorManager *instance;
    PAMSensorManager();
    ~PAMSensorManager();

    std::vector<PAMSensor *> sensors;
    std::vector<PAMSpecie *> specie;

    uint64_t last_loop_ms = 0;
    
    PAMSerialMenu serial_menu;

public:
    static PAMSensorManager* GetInstance();

    void addSensor(PAMSensor *sensor);
    char *csvHeader();
    std::vector<PAMSpecie *>* findSpeciesForName(char *name);
    PAMSpecie * findSpecieByName(char *name);

    void sleep();
    void wakeup();

    std::vector<PAMSensor *> getSensors();
    void runAllAverages();

    void loop();
    uint16_t measurement_period_ms = 5000;

    uint16_t serial_menu_rd;

};

#endif // __PAM_SENSOR_MANAGER_H__