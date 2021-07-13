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

#include "../../PAMSerial/PAMSerialResponder.h"

class TPHFusion: public PAMSensor, public PAMSerialResponder {

public:
    TPHFusion(uint8_t hih_address, bool hih_enabled = false);
    ~TPHFusion();

    bool start();
    bool measure();
    void registerSpecieSettings();

    void becomesResponder(uint16_t rd, bool child_returned);
    void onData(uint16_t rd, uint8_t *data, uint8_t length);
    void calculateAQI(void);

    bool enable_hih();
    bool disable_hih();

    PAMSpecie *temperature = nullptr;
    PAMSpecie *humidity = nullptr;
    PAMSpecie *pressure = nullptr;
    //PAMSpecie *voc = nullptr;
    //PAMSpecie air_quality_score;

    BME680 bme680;
    HIH8120 *hih8120 = nullptr;

private:
    bool hih_enabled = false;
    // float gas_upper_limit;
    // float gas_lower_limit;

};

#endif // __TPH_FUSION_H__