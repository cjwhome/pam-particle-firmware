#include "PAMSpecie.h"

PAMSpecie::PAMSpecie(uint16_t slope_address, uint16_t zero_address, float default_slope, float default_zero)
{
    this->serial_menu_rd = PAMSerial.registerResponder(&this->serial_menu);
    if (slope_address == 0x00 && zero_address == 0x00)
    {
        return ;
    }
    this->slope_responder = new PAMSerialEditEEPROMValue<float>(this->slope, slope_address, default_slope);
    this->slope_rd = this->serial_menu.addResponder(this->slope_responder, "Calibration Slope");

    this->zero_responder = new PAMSerialEditEEPROMValue<float>(this->zero, zero_address, default_zero);
    this->zero_rd = this->serial_menu.addResponder(this->zero_responder, "Calibration Zero");


}

PAMSpecie::~PAMSpecie() {}

void PAMSpecie::averaged_value() { 
    Serial.print("This is specie: ");
    Serial.println(this->name);
    Serial.print("This is the accumulated_value and number_of_measures: ");
    Serial.print(this->accumulated_value);
    Serial.print(", ");
    Serial.println(this->number_of_measures);
    this->average = this->accumulated_value/this->number_of_measures;
    this->accumulated_value = 0;
    this->number_of_measures = 0;
};