#ifndef __ABC_LOGIC_H__
#define __ABC_LOGIC_H__

#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "Telaire_T6713.h"


class ABCLogic: public PAMSerialResponder {

public:
    ABCLogic() : PAMSerialResponder() { };
    ~ABCLogic() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        Telaire_T6713 t6713;  //CO2 sensor
        Serial.println("Enter 0 to turn off ABC Logic, enter 1 to turn on ABC Logic.");
        Serial.setTimeout(50000);
        String tempString = Serial.readStringUntil('\r');
        int tempValue = tempString.toInt();
        Serial.println("");
        if(tempValue == 0){
            Serial.println("Disabling ABC Logic...");
            t6713.disableABCLogic();
        }
        else if (tempValue == 1) {
            Serial.println("Enabling ABC Logic...");
            t6713.enableABCLogic();
        }
        else 
        {
            Serial.println("You did not enter 0 or 1. Try again.");
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

#endif // __ABC_LOGIC_H__