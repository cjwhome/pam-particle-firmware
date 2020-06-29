/**
 * Plantower.h - Created on 04/14/2020
 * Author: David Kopala
 */

#ifndef __PLANTOWER_H__
#define __PLANTOWER_H__

#include "Particle.h"

#include "../../PAMSensorManager/PAMSensorManager.h"
#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../PAMEEPROM/EEPROMAddresses.h"

#define PLANTOWER_SERIAL_PACKET_LENGTH (31)
#define PM_25_CONSTANT_A (1.19)
#define PM_25_CONSTANT_B (0.119)

class Plantower: public PAMSensor {

public:
    Plantower(USARTSerial &serial);
    ~Plantower();

    void loop();

    PAMSpecie pm1;
    PAMSpecie pm2_5;
    PAMSpecie pm10;

private:
    USARTSerial *serial;
    char buff[PLANTOWER_SERIAL_PACKET_LENGTH];

    bool verifyPacket(char *packet);
};

#endif // __PLANTOWER_H__