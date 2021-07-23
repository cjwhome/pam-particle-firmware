#ifndef __CONTINUOUS_GPS_H__
#define __CONTINUOUS_GPS_H__




#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "CellularHelper.h"


class ContinuousGps: public PAMSerialResponder {

public:
    ContinuousGps() : PAMSerialResponder() { };
    ~ContinuousGps() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        char gps_byte = 0;
        while(!Serial.available()){
            if(Serial5.available() > 0){
                gps_byte = Serial5.read();
                Serial.print(gps_byte);
            }
        }
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

#endif // __CONTINUOUS_GPS_H__