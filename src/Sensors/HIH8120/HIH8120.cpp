#include "HIH8120.h"

HIH8120::HIH8120(uint8_t i2c_address)
    : temperature(TEMP_SLOPE_MEM_ADDRESS, TEMP_ZERO_MEM_ADDRESS), humidity(RH_SLOPE_MEM_ADDRESS, RH_ZERO_MEM_ADDRESS)
{
    this->name = "HIH8120";

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
    if (this->_hih->update() != this->_hih->NoError) {
        // return false;
    }
    
    this->temperature.raw_value = this->_hih->temperature();
    float adj_value = (this->temperature.slope * this->temperature.raw_value) + this->temperature.zero;
    this->temperature.adj_value = adj_value;

    this->temperature.accumulated_value += adj_value;
    this->temperature.number_of_measures++;

    this->humidity.raw_value = this->_hih->humidity();
    adj_value = min(100, (this->humidity.slope * this->humidity.raw_value) + this->humidity.zero);
    this->humidity.adj_value = adj_value;

    this->humidity.accumulated_value += adj_value;
    this->humidity.number_of_measures++;

    return true;
}

bool HIH8120::stop()
{
    return this->_hih->stop();
}