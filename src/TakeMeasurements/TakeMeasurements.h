/**
 * PAMSensor.h - Created on 04/11/2020
 * Author: David Kopala
 * 
 * Provides a singleton for managing sensor modules
 */

#ifndef __TAKE_MEASUREMENTS_H__
#define __TAKE_MEASUREMENTS_H__

#include"../Sensors/T6713/T6713.h"

class TakeMeasurements {

    private:


        float air_quality_score = 0;
        float sound_average = 0;
        float ozone = 0;
        float co2 = 0;

    public:
        TakeMeasurements();
        ~TakeMeasurements();

        void calculateAQI();
        void readSound();
        void getEspOzoneData();
        void readCO2(T6713 t6713);
        void readOzone();

        float get_air_quality_score();
        float get_sound_average();
        float get_ozone();
        float get_co2();



};

#endif // __MEASUREMENT_HANDLER_H