/**
 * PRACTICE.h - Created on 04/18/2020
 * Author: David Kopala
 */

#ifndef __PRACTICE_H__
#define __PRACTICE_H__

#include "Particle.h"

#include "LMP91000.h"
#include "Adafruit_ADS1X15.h"

#include "../../Constants.h"
#include "../../PAMSensorManager/PAMSensorManager.h"
#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../PAMEEPROM/EEPROMAddresses.h"

#define ALPHA_ADC_READ_AMOUNT (10)

class PRACTICE: public PAMSensor {

public:
    PRACTICE(uint8_t ads_address, uint8_t enable_pin);
    ~PRACTICE();

    bool start();
    bool measure();

    PAMSpecie co;

private:
    uint8_t enable_pin; // Pulling the pin low enables the chip
    
    LMP91000 lmp91000;

    uint8_t ads_address;
    Adafruit_ADS1115 *ads1115;
    
};

#endif // __PAMCO_H__