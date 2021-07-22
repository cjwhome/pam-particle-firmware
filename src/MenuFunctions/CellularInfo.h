#ifndef __CELLULAR_INFO_H__
#define __CELLULAR_INFO_H__




#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "CellularHelper.h"


class CellularInfo: public PAMSerialResponder {

public:
    CellularInfo() : PAMSerialResponder() { };
    ~CellularInfo() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        PAMSerial.printf(rd, "Getting cellular info. This may take a while.... \n\r");

        Log.info("IMEI=%s", CellularHelper.getIMEI().c_str());

        Log.info("IMSI=%s", CellularHelper.getIMSI().c_str());

        Log.info("ICCID=%s", CellularHelper.getICCID().c_str());
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

#endif // __CELLULAR_INFO_H__