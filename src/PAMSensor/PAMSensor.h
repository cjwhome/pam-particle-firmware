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

#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"

class PAMSensorFeed;

class PAMSensor {

public:
    PAMSensor();
    ~PAMSensor();

    enum SensorState { UNINITIALIZED, IDLE, MEASURING, SLEEPING, ERROR };
    SensorState state = SensorState::UNINITIALIZED;

    char *name = "";
    virtual void registerSpecieSettings();

    // Lifecycle Methods
    virtual bool start() { /* Serial.println("[PAMSensor]::start\tNOT IMPLEMENTED"); */ return false; };
    virtual bool stop() { /* Serial.println("[PAMSensor]::start\tNOT IMPLEMENTED"); */ return false; };
    virtual void loop();
    virtual bool measure() { /* Serial.println("[PAMSensor]::measure\tNOT IMPLEMENTED"); */ return false; };
    virtual bool sleep() { /* Serial.println("[PAMSensor]::sleep\tNOT IMPLEMENTED"); */ return false; };
    virtual bool wakeup() { /* Serial.println("[PAMSensor]::wakeup\tNOT IMPLEMENTED"); */ return false; };

    char *csvHeader();

    std::vector<PAMSpecie *>* getSpecies() { return &(this->species); }

    uint16_t serial_menu_rd;

protected:
    std::vector<PAMSpecie *> species;

    PAMSerialMenu serial_menu;
    PAMSensorFeed *sensor_feed;
};

class PAMSensorFeed: public PAMSerialResponder {

public:
    PAMSensorFeed(PAMSensor *sensor);
    ~PAMSensorFeed();

    void becomesResponder(uint16_t rd, bool child_returned);
    void onData(uint16_t rd, uint8_t *data, uint8_t length);

    void loop();

private:
    PAMSensor *sensor;
    uint16_t rd = -1;
    uint64_t last_feed = 0;

};

#endif // __PAM_SENSOR_H__