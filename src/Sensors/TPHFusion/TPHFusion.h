/**
 * TPHFusion.h - Created 04/12/2020
 * Author: David Kopala
 * 
 * Fuses data from a BME680 and a HIH8120.
 * If the HIH8120 has been enabled, we used its Temperature and Humidity data, otherwise we use the BME860.
 * We always read the pressure from the BME860.
 */

#ifndef __TPH_FUSION_H__
#define __TPH_FUSION_H__

#include "Particle.h"

#include "../../PAMSensor/PAMSensor.h"
#include "../../PAMSpecie/PAMSpecie.h"
#include "../../Sensors/BME680/BME680.h"
#include "../../Sensors/HIH8120/HIH8120.h"

class TPHFusion: public PAMSensor {

public:
    TPHFusion(uint8_t hih_address, bool hih_enabled = false);
    ~TPHFusion();

    bool start();
    bool measure();

    bool enable_hih();
    bool disable_hih();

    PAMSpecie *temperature = nullptr;
    PAMSpecie *humidity = nullptr;
    PAMSpecie *pressure = nullptr;
    PAMSpecie *voc = nullptr;

    BME680 bme680;
    HIH8120 *hih8120 = nullptr;

private:
    bool hih_enabled = false;

};

#endif // __TPH_FUSION_H__