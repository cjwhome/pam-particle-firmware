#ifndef __PAMNO2_H__
#define __PAMNO2_H__

#include "Particle.h"

#include "LMP91000.h"
#include "Adafruit_ADS1X15.h"

#include "../../PAMSensorManager/PAMSensorManager.h"
#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../PAMEEPROM/EEPROMAddresses.h"

#define ALPHA_ADC_READ_AMOUNT (10)
#define ADS_BIT_MV  (0.1875)

class PAMNO2: public PAMSensor {

public:
    PAMNO2(uint8_t ads_address, uint8_t enable_pin);
    ~PAMNO2();

    bool start();
    bool measure();

    PAMSpecie no2;

private:
    uint8_t enable_pin; // Pulling the pin low enables the chip
    
    LMP91000 lmp91000;

    uint8_t ads_address;
    Adafruit_ADS1115 *ads1115;
    
};

#endif // __PAMNO2_H__