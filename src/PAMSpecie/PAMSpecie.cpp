#include "PAMSpecie.h"

PAMSpecie::PAMSpecie(uint16_t slope_address, uint16_t zero_address, float default_slope, float default_zero)
{
    this->slope_responder = new PAMSerialEditEEPROMValue<float>(this->slope, slope_address, default_slope);
    this->slope_rd = this->serial_menu.addResponder(this->slope_responder, "Calibration Slope");

    this->zero_responder = new PAMSerialEditEEPROMValue<float>(this->zero, zero_address, default_zero);
    this->zero_rd = this->serial_menu.addResponder(this->zero_responder, "Calibration Zero");

    this->serial_menu_rd = PAMSerial.registerResponder(&this->serial_menu);
    //Serial.println("pam species here");
}

PAMSpecie::~PAMSpecie() {}