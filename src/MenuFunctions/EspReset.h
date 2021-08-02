#ifndef __ESP_RESET_H__
#define __ESP_RESET_H__




#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "Wiring.h"


class EspReset: public PAMSerialResponder {

public:
    EspReset() : PAMSerialResponder() { };
    ~EspReset() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        PAMSerial.printf(rd, "Resetting ESP... \r\n");
        resetESP();
        PAMSerial.printf(rd, "Finished resetting ESP \r\n");
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

    void resetESP(void){
        digitalWrite(ESP_WROOM_EN, LOW);
        digitalWrite(PLANTOWER_EN, LOW);
        digitalWrite(BLOWER_EN, LOW);
        digitalWrite(CO2_EN, LOW);
        delay(1000);
        digitalWrite(ESP_WROOM_EN, HIGH);
        digitalWrite(PLANTOWER_EN, HIGH);
        digitalWrite(BLOWER_EN, HIGH);
        digitalWrite(CO2_EN, HIGH);
        delay(1000);
    }

private:



};

#endif // __CELLULAR_INFO_H__