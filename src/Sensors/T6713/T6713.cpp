#include "T6713.h"

T6713::T6713()
{
    this->name = "T6713";

    this->CO2.name = "CO2";
    this->CO2.units = "PPM";
    this->CO2.packet_constant = CO2_PACKET_CONSTANT;
    this->species.push_back(&this->CO2);
}

T6713::~T6713() {}

bool T6713::start() {
    bool adafruit_init_success = false;

    Serial.println("[T6713]::init");

    if (!(adafruit_init_success = this->_t6713.begin())) {
        Serial.println("Could not find a T6713! Check the wiring!");
    }

    return adafruit_init_success;
}

bool T6713::measure() {
    float measurement = this->_t6713.readPPM();
    if (measurement == 0) {
        return false;
    }

    this->CO2.raw_value = measurement;
    this->CO2.adj_value = (this->CO2.slope * measurement) + this->CO2.zero;

    return true;
}