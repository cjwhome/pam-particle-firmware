#ifndef __PRINT_HEADER_H__
#define __PRINT_HEADER_H__




#include "../PAMSerial/PAMSerial.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"


class PrintHeader: public PAMSerialResponder {

public:
    PrintHeader() : PAMSerialResponder() { };
    ~PrintHeader() {};

    void becomesResponder(uint16_t rd, bool child_returned)
    {
        PAMSensorManager * manager = PAMSensorManager::GetInstance();
        char *csv_header = manager->csvHeader();
        PAMSerial.printf(rd, csv_header);
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