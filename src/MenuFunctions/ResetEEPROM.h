#ifndef __EEPROM_RESET_H__
#define __EEPROM_RESET_H__

//#include "../Particle.h
#include <vector>


#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"


class EEPROMReset: public PAMSerialResponder {

public:
    EEPROMReset() : PAMSerialResponder() { };
    ~EEPROMReset() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        PAMSerial.printf(rd, "Resetting the EEPROM values. The PAM will reset now.\r\n");
        EEPROM.clear();
        System.reset();
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

#endif // __EEPROM_RESET_H__