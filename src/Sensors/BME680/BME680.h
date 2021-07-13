/**
 * BME680.h - Created 04/12/2020
 * Author: David Kopala
 * 
 * PAM Sensor for communicating with the onboard BME680
 */

#ifndef __BME_680_H__
#define __BME_680_H__

#include "Particle.h"
#include "Adafruit_BME680.h"

#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../PAMEEPROM/EEPROMAddresses.h"

class BME680: public PAMSensor {

public:
    BME680();
    ~BME680();

    PAMSpecie temperature;
    PAMSpecie humidity;
    PAMSpecie pressure;
    //PAMSpecie voc;

    Adafruit_BME680 _bme680;

    bool start();
    bool measure();
};

#endif //  __BME_680_H__