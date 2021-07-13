#include "TPHFusion.h"

TPHFusion::TPHFusion(uint8_t hih_address, bool _hih_enabled) : air_quality_score(0x00, 0x00)
{
    this->name = "Temp/Press/RH Fusion";

    this->serial_menu.addResponder(this, "Fusion Settings");
    this->serial_menu.addResponder(this->bme680.serial_menu_rd, "BME680 Settings");
    if (hih_address != 0x00) {
        this->hih8120 = new HIH8120(hih_address);
        this->hih_enabled = _hih_enabled;
        this->serial_menu.addResponder(this->hih8120->serial_menu_rd, "HIH8120 Settings");
    }

    if (this->hih_enabled) {
        this->temperature = &this->hih8120->temperature;
        this->humidity = &this->hih8120->humidity;
    } else {
        this->temperature = &this->bme680.temperature;
        this->humidity = &this->bme680.humidity;
    }
    this->pressure = &this->bme680.pressure;
    //this->voc = &this->bme680.voc;

    // this->air_quality_score.name = "ozone";
    // this->air_quality_score.units = "PPB";
    //this->air_quality_score.packet_constant = VOC_PACKET_CONSTANT;

    // this->serial_menu.addResponder(this->air_quality_score.serial_menu_rd, "Air Quality Score");

    // PAMSerialEditEEPROMValue<float> *Gas_Upper_responder = new PAMSerialEditEEPROMValue<float>(this->gas_upper_limit, GAS_UPPER_LIMIT_MEM_ADDRESS, 10000);
    // this->air_quality_score.serial_menu.addResponder(Gas_Upper_responder, "Gas Upper Limit");

    // PAMSerialEditEEPROMValue<float> *Gas_Lower_responder = new PAMSerialEditEEPROMValue<float>(this->gas_lower_limit, GAS_LOWER_LIMIT_MEM_ADDRESS, 1000);
    // this->air_quality_score.serial_menu.addResponder(Gas_Lower_responder, "Gas Lower Limit");

    this->species.push_back(this->temperature);
    this->species.push_back(this->humidity);
    this->species.push_back(this->pressure);
    //this->species.push_back(this->voc);
    this->species.push_back(&this->air_quality_score);
}

TPHFusion::~TPHFusion()
{
    free(this->hih8120);
    this->hih8120 = nullptr;
}

bool TPHFusion::start()
{
    bool success = this->bme680.start();
    if (this->hih_enabled) {
        success &= this->hih8120->start();
    }

    return success;
}

void TPHFusion::registerSpecieSettings() {
    this->hih8120->registerSpecieSettings();
    this->bme680.registerSpecieSettings();
}

bool TPHFusion::measure()
{
    bool success = this->bme680.measure();
    if (this->hih_enabled) {
        success &= this->hih8120->measure();
    }
    calculateAQI();
    return success;
}

bool TPHFusion::enable_hih()
{
    if (this->hih8120 == nullptr) {
        return false;
    }
    if (this->hih_enabled) {
        return true;
    }

    this->hih_enabled = true;

    this->temperature = &this->hih8120->temperature;
    this->humidity = &this->hih8120->humidity;

    this->species.clear();
    this->species.push_back(this->temperature);
    this->species.push_back(this->humidity);
    this->species.push_back(this->pressure);
    //this->species.push_back(this->voc);

    return this->hih8120->start();
}

bool TPHFusion::disable_hih()
{
    if (this->hih8120 == nullptr) {
        return false;
    }
    if (this->hih_enabled == false) {
        return true;
    }

    this->hih_enabled = false;

    this->temperature = &this->bme680.temperature;
    this->humidity = &this->bme680.humidity;

    this->species.clear();
    this->species.push_back(this->temperature);
    this->species.push_back(this->humidity);
    this->species.push_back(this->pressure);
    //this->species.push_back(this->voc);

    return this->hih8120->stop();
}



// void TPHFusion::calculateAQI(){
//     float hum_score;
//     float gas_reference = 250000;
//     float hum_reference = 40; 
//     //Calculate humidity contribution to IAQ index
//     gas_reference = this->voc->adj_value;

//         float current_humidity = this->humidity->adj_value;

//     if (current_humidity >= 38 && current_humidity <= 42)
//         hum_score = 0.25*100; // Humidity +/-5% around optimum
//     else
//     { //sub-optimal
//         if (current_humidity < 38)
//             hum_score = 0.25/hum_reference*current_humidity*100;
//         else
//         {
//             hum_score = ((-0.25/(100-hum_reference)*current_humidity)+0.416666)*100;
//         }
//     }

//     //Calculate gas contribution to IAQ index

//     if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit;
//     if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
//     float gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;

//     //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
//     this->air_quality_score.adj_value = hum_score + gas_score;

//     this->air_quality_score.accumulated_value += hum_score+ gas_score;
//     this->air_quality_score.number_of_measures++;
// }