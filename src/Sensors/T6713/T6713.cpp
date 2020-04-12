#include "T6713.h"

T6713::T6713() {}
T6713::~T6713() {}

void T6713::start() {
    Serial.println("[T6713]::init");

    PAMSpecie *co2 = new PAMSpecie();
    co2->name = "CO2";
    co2->units = "PPM";
    co2->packet_constant = CO2_PACKET_CONSTANT;
    this->species.push_back(co2);
}