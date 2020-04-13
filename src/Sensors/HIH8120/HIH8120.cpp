#include "HIH8120.h"

HIH8120::HIH8120(uint8_t i2c_address)
{
    this->_hih = new HIH61XX(i2c_address);

    this->temperature.name = "Temperature";
    this->temperature.units = "C";
    this->temperature.packet_constant = TEMPERATURE_PACKET_CONSTANT;
    this->species.push_back(&this->temperature);

    this->humidity.name = "Humidity";
    this->humidity.units = "%";
    this->humidity.packet_constant = HUMIDITY_PACKET_CONSTANT;
    this->species.push_back(&this->humidity);
}

HIH8120::~HIH8120() {}

bool HIH8120::start()
{
    return this->_hih->start();
}

bool HIH8120::measure()
{
    Serial.println("[HIH8120]::measure");
    if (this->_hih->update() != this->_hih->NoError) {
        // return false;
    }
    
    this->temperature.raw_value = this->_hih->temperature();
    this->temperature.adj_value = (this->temperature.slope * this->temperature.raw_value) + this->temperature.zero;

    this->humidity.raw_value = this->_hih->humidity();
    this->humidity.adj_value = min(100, (this->humidity.slope * this->humidity.raw_value) + this->humidity.zero);
    return true;
}

bool HIH8120::stop()
{
    return this->_hih->stop();
}