/**
 * PAMSensor.h - Created on 04/11/2020
 * Author: David Kopala
 * 
 * Provides a singleton for managing sensor modules
 */

#ifndef __TAKE_MEASUREMENTS_H__
#define __TAKE_MEASUREMENTS_H__

#include "../Sensors/T6713/T6713.h"
#include "../Sensors/TPHFusion/TPHFusion.h"
#include "../Sensors/Plantower/Plantower.h"
#include "../Sensors/PAMCO/PAMCO.h"
#include "../Wiring.h"
#include "../PAMSensor/PAMSensor.h"
#include "../PAMSpecie/PAMSpecie.h"


class TakeMeasurements {

    private:


        float air_quality_score = 0;
        float sound_average = 0;
        float ozone = 0;
        float co2 = 0;
        int ozone_offset; //changable by user
        int gas_upper_limit = 50000;  // Good air quality limit // Changable by user
        int gas_lower_limit = 1200;   // Bad air quality limit // Changable by user
        T6713 *t6713;
        TPHFusion *tph_fusion; 
        Plantower *plantower; 
        PAMCO *pamco;

    public:
        TakeMeasurements(T6713 *T6713, TPHFusion *Tph_fusion, Plantower *Plantower, PAMCO *Pamco);
        ~TakeMeasurements();

        void calculateAQI();
        void readSound();
        void getEspOzoneData();
        void readCO2();
        void readOzone();

        float get_air_quality_score();
        float get_sound_average();
        float get_ozone();
        float get_co2();



};

#endif // __MEASUREMENT_HANDLER_H