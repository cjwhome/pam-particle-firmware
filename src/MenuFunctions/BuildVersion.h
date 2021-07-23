#ifndef __BUILD_VERSION_H__
#define __BUILD_VERSION_H__

#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "../global.h"


class BuildVersion: public PAMSerialResponder {

public:
    BuildVersion() : PAMSerialResponder() { };
    ~BuildVersion() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        Serial.print("APP Version: ");
        Serial.println(Global::GetInstance()->app_version);
        Serial.print("Build: ");
        Serial.println(Global::GetInstance()->build_version);
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