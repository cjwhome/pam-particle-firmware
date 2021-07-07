#include "T6713.h"

T6713::T6713() : CO2(CO2_SLOPE_MEM_ADDRESS, CO2_ZERO_MEM_ADDRESS)
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
    PAMSpecie * specie = this->manager->findSpecieByName("Pressure");

        //correct for altitude
    float pressure_correction = specie->adj_value/100;
    if(pressure_correction > LOW_PRESSURE_LIMIT && pressure_correction < HIGH_PRESSURE_LIMIT){
        pressure_correction /= SEALEVELPRESSURE_HPA;
        this->pressure_correct = true;
    }

    this->CO2.raw_value = measurement;

    float adj_value = (this->CO2.slope * measurement) + this->CO2.zero;
    if (this->pressure_correct)
    {
        this->pressure_correct = false;
        adj_value *= pressure_correction;
    }
    this->CO2.adj_value = adj_value;

    this->CO2.accumulated_value += adj_value;
    this->CO2.number_of_measures++;

    return true;
}