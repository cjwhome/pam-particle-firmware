#ifndef __TIME_ZONE_SET_H__
#define __TIME_ZONE_SET_H__




#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "CellularHelper.h"


class TimeZoneSet: public PAMSerialResponder {

public:
    TimeZoneSet() : PAMSerialResponder() { };
    ~TimeZoneSet() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        Serial.println("Enter new Device time zone (-12.0 to 14.0)");
        Serial.setTimeout(50000);
        String tempString = Serial.readStringUntil('\r');
        int tempValue = tempString.toInt();
        Serial.println("");
        if(tempValue >= -12 && tempValue <= 14){
            Time.zone(tempValue);
            Serial.print("\n\rNew Device time zone:");
            Serial.println(tempValue);
            EEPROM.put(TIME_ZONE_MEM_ADDRESS, tempValue);
        }else{
            Serial.println("\n\rInvalid value!");
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

#endif // __DATE_TIME_SET_H__