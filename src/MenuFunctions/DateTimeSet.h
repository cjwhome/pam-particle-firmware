#ifndef __DATE_TIME_SET_H__
#define __DATE_TIME_SET_H__




#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "CellularHelper.h"


class DateTimeSet: public PAMSerialResponder {

public:
    DateTimeSet() : PAMSerialResponder() { };
    ~DateTimeSet() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        Serial.println("Enter new Device time and date (10 digit epoch timestamp):");
        Serial.setTimeout(50000);
        String tempString = Serial.readStringUntil('\r');
        int tempValue = tempString.toInt();
        Serial.println("");
        if(tempValue > 966012661 && tempValue < 4121686261){       //min is the year 2000, max is the year 2100
            Time.setTime(tempValue);
            Serial.print("\n\rNew Device Time:");
            Serial.println(Time.timeStr());
        }
        else{
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