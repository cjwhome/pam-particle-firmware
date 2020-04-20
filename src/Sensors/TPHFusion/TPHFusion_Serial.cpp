#include "TPHFusion.h"

void TPHFusion::becomesResponder(uint16_t rd, bool child_returned)
{
    PAMSerial.println(rd, "TPH Fusion Settings");
    PAMSerial.printf(rd, "Prioritizing: ");
    if (this->hih_enabled == true) {
        PAMSerial.println(rd, "HIH8120");
    } else {
        PAMSerial.println(rd, "BME680");
    }
    PAMSerial.println(rd, "Pick an option below to select the device to prioritize: ('x' to exit)");
    PAMSerial.println(rd, "0 - HIH8120");
    PAMSerial.println(rd, "1 - BME680");
}

void TPHFusion::onData(uint16_t rd, uint8_t *data, uint8_t length)
{
    uint8_t choice;
    if (*data == 'x') {
        PAMSerial.popResponder();
    } else if ((sscanf((char *) data, "%d", &choice)) == 1) {
        if (choice == 0) {
            PAMSerial.println(rd, "Prioritizing HIH8120");
            this->enable_hih();
            PAMSerial.popResponder();
        } else if (choice == 1) {
            PAMSerial.println(rd, "Prioritizing BME680");
            this->disable_hih();
            PAMSerial.popResponder();
        } else {
            PAMSerial.printf(rd, "Unknown option: '%d'. Enter a new option, or press 'x' to exit.\n\r", choice);
        }
    } else {
        PAMSerial.println(rd, "Could not read the input. Enter a new option, or press 'x' to exit.\n\r");
    }
}