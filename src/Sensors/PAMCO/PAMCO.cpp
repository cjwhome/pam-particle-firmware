#include "PAMCO.h"

PAMCO::PAMCO(uint8_t _ads_address, uint8_t _enable_pin)
    : co(CO_SLOPE_MEM_ADDRESS, CO_ZERO_MEM_ADDRESS)
{
    this->name = "CO Sensor";

    this->enable_pin = _enable_pin;
    pinMode(this->enable_pin, OUTPUT);
    digitalWrite(this->enable_pin, HIGH);

    // Wire.begin() needs to be called before initializing the sensor
    // since it uses I2C, but this should be done in the .ino file
    this->ads_address = _ads_address;
    this->ads1115 = new Adafruit_ADS1115(_ads_address);

    this->co.name = "CO";
    this->co.units = "PPM";
    this->species.push_back(&this->co);
}

PAMCO::~PAMCO() {} 

bool PAMCO::start()
{
    digitalWrite(this->enable_pin, LOW);
    if(this->lmp91000.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0) {
        Serial.println("Couldn't communicate with LMP91000 for CO");
        return false;
    }
    Serial.println("Initialized LMP91000 for CO");
    digitalWrite(this->enable_pin, HIGH);

    this->ads1115->begin();
    // Try to read from the ADS to make sure it's working
    if (Wire.requestFrom(this->ads_address, 1) == 0) {
        Serial.println("Could not communicate with Adafruit_ADS1115 for CO");
        return false;
    }
    this->ads1115->setGain(GAIN_TWOTHIRDS);

    return true;
}

bool PAMCO::measure()
{
    // Serial.print("lmp91000_1_en: ");
    // Serial.println(this->enable_pin);
    digitalWrite(this->enable_pin, LOW);

    if(Wire.requestFrom(0x49,1) == 0) {
        Serial.println("Couldn't communicate with LMP91000");
        digitalWrite(this->enable_pin, HIGH);
        return false;
    }
    
    float half_vref = this->ads1115->readADC_SingleEnded(3); //half of Vref
    float volt_half_Vref = half_vref * ADS_BIT_MV;
    if(abs((volt_half_Vref)/1000 - 1.25) > 0.5) {
        Serial.printf("Halfvolt: %1.2f\n\r", volt_half_Vref/1000);
    }

    if(this->lmp91000.read(LMP91000_STATUS_REG) == 0){
        // Serial.println("Status = 0 from LMP91000 status reg");
        digitalWrite(this->enable_pin, HIGH);
        return false;
    }

    if(Wire.requestFrom(0x49,1) == 0 || lmp91000.read(LMP91000_STATUS_REG) == 0) {
        digitalWrite(this->enable_pin, HIGH);
        return false;
    }

    float A0_gas = 0, A1_aux = 0, A2_temperature = 0; half_vref = 0;
    for (uint8_t i = 0; i < ALPHA_ADC_READ_AMOUNT; i++) {
        A0_gas += this->ads1115->readADC_SingleEnded(0); //gas
        A1_aux += this->ads1115->readADC_SingleEnded(1); //aux out
        A2_temperature += this->ads1115->readADC_SingleEnded(2); //temperature
        half_vref += this->ads1115->readADC_SingleEnded(3); //half of Vref
    }
    A0_gas = (A0_gas / ALPHA_ADC_READ_AMOUNT) * ADS_BIT_MV;
    A1_aux = (A1_aux / ALPHA_ADC_READ_AMOUNT) * ADS_BIT_MV;
    A2_temperature = (A2_temperature / ALPHA_ADC_READ_AMOUNT) * ADS_BIT_MV;
    half_vref = (half_vref / ALPHA_ADC_READ_AMOUNT) * ADS_BIT_MV;

    float sensorCurrent = (half_vref - A0_gas) / -120; // Working Electrode current in microamps (millivolts / Kohms)
    float auxCurrent = (half_vref - A1_aux) / -150;
        //{1, -1, -0.76}, //CO-A4 (<=10C, 20C, >=30C)

    float temperature = 0;
    std::vector<PAMSpecie *> *temperatures = PAMSensorManager::GetInstance()->findSpeciesForName("temperature");
    if (temperatures->size() > 0) {
        temperature = temperatures->at(0)->adj_value;
    }
    free(temperatures);

    if(temperature <= 15) {
        sensorCurrent = ((sensorCurrent) - (auxCurrent));
    } else if(temperature <= 25) {
        sensorCurrent = ((sensorCurrent) - (-1)*(auxCurrent));
    } else {
        sensorCurrent = ((sensorCurrent) - (-0.76)*(auxCurrent));
    }



    this->co.raw_value = (sensorCurrent / 0.358); //sensitivity .358 nA/ppb - from Alphasense calibration certificate, So .358 uA/ppm
    float adj_value = (this->co.slope * this->co.raw_value) + this->co.zero;
    this->co.adj_value = adj_value;


    this->co.accumulated_value += adj_value;
    this->co.number_of_measures++;

    digitalWrite(this->enable_pin, HIGH);
    return true;
}