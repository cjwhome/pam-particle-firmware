/**
 * PAMSensor.h - Created on 04/11/2020
 * Author: David Kopala
 * 
 * Provides a singleton for managing sensor modules
 */

#ifndef __MEASUREMENT_HANDLER_H__
#define __MEASUREMENT_HANDLER_H__

#include "../Sensors/T6713/T6713.h"
#include "../Sensors/TPHFusion/TPHFusion.h"
#include "../Sensors/Plantower/Plantower.h"
#include "../Sensors/PAMCO/PAMCO.h"

class MeasurementHandler {

private:
    MeasurementHandler(T6713 T6713, TPHFusion Tph_fusion, Plantower Plantower, PAMCO Pamco);
    ~MeasurementHandler();


    bool send_to_sensibl

    void output_data_to_esp();
    void output_data_to_sd();
    void output_data_to_cloud();

public:


};

#endif // __MEASUREMENT_HANDLER_H__