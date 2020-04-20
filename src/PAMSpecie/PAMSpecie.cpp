#include "PAMSpecie.h"

PAMSpecie::PAMSpecie()
{
    this->slope_responder = new PAMSerialEditValue<float>(this->slope);
    this->slope_rd = this->serial_menu.addResponder(this->slope_responder, "Calibration Slope");

    this->zero_responder = new PAMSerialEditValue<float>(this->zero);
    this->zero_rd = this->serial_menu.addResponder(this->zero_responder, "Calibration Zero");

    this->serial_menu_rd = PAMSerial.registerResponder(&this->serial_menu);
}

PAMSpecie::~PAMSpecie() {}