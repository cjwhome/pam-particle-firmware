/**
 * HIH8120.h - Created 04/11/2020
 * Author: David Kopala
 */

#ifndef __HIH_8120_H__
#define __HIH_8120_H__

#include "Particle.h"
#include "HIH61XX.h"

#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../PAMEEPROM/EEPROMAddresses.h"
#include "../../global.h"

class HIH8120: public PAMSensor {

public:
    HIH8120(uint8_t i2c_address);
    ~HIH8120();

    bool start();
    bool measure();
    bool stop();

    PAMSpecie temperature;
    PAMSpecie humidity;
    Global * globalVariables = nullptr;

    HIH61XX *_hih;

private:
    int i2c_address;
};

#endif // __HIH_8120_H__