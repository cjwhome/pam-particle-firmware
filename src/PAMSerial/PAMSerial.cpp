#include "PAMSerial.h"

PAMSerialManager& __fetch_pam_usb_serial() {
    static PAMSerialManager pam_serial(Serial);
    return pam_serial;
}