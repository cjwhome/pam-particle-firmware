#include "PAMNO2.h"

PAMNO2::PAMNO2(uint8_t _ads_address, uint8_t _enable_pin)
    : no2(NO2_SLOPE_MEM_ADDRESS, NO2_ZERO_MEM_ADDRESS)
{
    this->name = "NO2 Sensor";

    this->enable_pin = _enable_pin;
    pinMode(this->enable_pin, OUTPUT);
    digitalWrite(this->enable_pin, HIGH);

    // Wire.begin() needs to be called before initializing the sensor
    // since it uses I2C, but this should be done in the .ino file
    this->ads_address = _ads_address;
    this->ads1115 = new Adafruit_ADS1115(_ads_address);

    this->no2.name = "NO2";
    this->no2.units = "PPM";
    this->species.push_back(&this->no2);
}

PAMNO2::~PAMNO2() {} 

bool PAMNO2::start()
{
    digitalWrite(this->enable_pin, LOW);
    if(this->lmp91000.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0) {
        Serial.println("Couldn't communicate with LMP91000 for NO2");
        return false;
    }
    Serial.println("Initialized LMP91000 for NO2");
    digitalWrite(this->enable_pin, HIGH);

    this->ads1115->begin();
    // Try to read from the ADS to make sure it's working
    if (Wire.requestFrom(this->ads_address, 1) == 0) {
        Serial.println("Could not communicate with Adafruit_ADS1115 for NO2");
        return false;
    }
    this->ads1115->setGain(GAIN_TWOTHIRDS);

    return true;
}

bool PAMNO2::measure()
{
    // Serial.print("lmp91000_1_en: ");
    // Serial.println(this->enable_pin);
    digitalWrite(this->enable_pin, LOW);

    if(Wire.requestFrom(0x49,1) == 0) {
        Serial.println("Couldn't communicate with NO2 Sensor");
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

    this->no2.raw_value = (sensorCurrent / -0.358); //sensitivity .358 nA/ppb - from Alphasense calibration certificate, So .358 uA/ppm
    float adj_value = (this->no2.slope * this->no2.raw_value) + this->no2.zero;
    this->no2.adj_value = adj_value;


    this->no2.accumulated_value += adj_value;
    this->no2.number_of_measures++;

    digitalWrite(this->enable_pin, HIGH);
    return true;
}