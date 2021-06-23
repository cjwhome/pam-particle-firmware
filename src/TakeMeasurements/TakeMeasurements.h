/**
 * PAMSensor.h - Created on 04/11/2020
 * Author: David Kopala
 * 
 * Provides a singleton for managing sensor modules
 */

#ifndef __TAKE_MEASUREMENTS_H__
#define __TAKE_MEASUREMENTS_H__

class TakeMeasurements {

private:
    TakeMeasurements();
    ~TakeMeasurements();

    void calculateAQI();
    void readSound();
    void getEspOzoneData();
    void readCO2(T6713 t6713);
    void readOzone();


public:
    float air_quality_score;
    float sound_average;
    float ozone;
    float co2;



};

#endif // __MEASUREMENT_HANDLER_H