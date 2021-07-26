#ifndef __CALIBRATE_CO2_H__
#define __CALIBRATE_CO2_H__

#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "Telaire_T6713.h"


class CalibrateCO2: public PAMSerialResponder {

public:
    CalibrateCO2() : PAMSerialResponder() { };
    ~CalibrateCO2() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        Telaire_T6713 t6713;  //CO2 sensor
        Serial.print("Calibrating CO2");
        t6713.calibrate(1); 
        return ;
    }

    void onData(uint16_t rd, uint8_t *data, uint8_t length)
    {
        PAMSerial.printf(rd, "Ondata for reset triggered");
        PAMSerial.popResponder();
    }

    void loop()
    {
        return ;
    }

private:



};

#endif // __DATE_TIME_SET_H__