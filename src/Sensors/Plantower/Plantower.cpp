#include "Plantower.h"

Plantower::Plantower(USARTSerial &_serial)
{
    this->serial = &_serial;

    this->serial->begin(9600);
    this->serial->setTimeout(5000);

    this->pm1.name = "PM1";
    this->pm1.units = "ug/m3";
    this->species.push_back(&this->pm1);

    this->pm2_5.name = "PM2.5";
    this->pm2_5.units = "ug/m3";
    this->species.push_back(&this->pm2_5);

    this->pm10.name = "PM10";
    this->pm10.units = "ug/m3";
    this->species.push_back(&this->pm10);
}

Plantower::~Plantower() {}

bool Plantower::verifyPacket(char *packet)
{
    size_t receive_sum = 0;

    for (uint8_t i = 0; i < PLANTOWER_SERIAL_PACKET_LENGTH - 2; i++) {
        receive_sum += packet[i];
    }
    receive_sum += 0x42;

    uint16_t checksum = (packet[PLANTOWER_SERIAL_PACKET_LENGTH-2] << 8) + packet[PLANTOWER_SERIAL_PACKET_LENGTH - 1];
    return receive_sum == checksum;
}

void Plantower::loop()
{
    memset(this->buff, 0, PLANTOWER_SERIAL_PACKET_LENGTH);

    if (this->serial->find("B")) {
        this->serial->readBytes(this->buff, PLANTOWER_SERIAL_PACKET_LENGTH);
        if(this->buff[0] == 0x4d){
            if(this->verifyPacket(this->buff)){ //All units are ug/m^3
                this->pm1.raw_value = (this->buff[3] << 8) + this->buff[4];
                this->pm1.adj_value = (this->pm1.slope * this->pm1.raw_value) + this->pm1.zero;

                float pm2_5_correction_factor = 1;
                std::vector<PAMSpecie *> *humidity_species = PAMSensorManager::GetInstance()->findSpeciesForName("humidity");
                if (humidity_species->size() >= 1) {
                    float humidity = humidity_species->at(0)->adj_value / 100;
                    pm2_5_correction_factor = PM_25_CONSTANT_A + (PM_25_CONSTANT_B * humidity) / (1 - humidity);
                }
                free(humidity_species);

                this->pm2_5.raw_value = (this->buff[5] << 8) + this->buff[6];
                this->pm2_5.adj_value = (this->pm2_5.slope * (this->pm2_5.raw_value / pm2_5_correction_factor)) + this->pm2_5.zero;

                this->pm10.raw_value = (this->buff[7] << 8) + this->buff[8];
                this->pm10.adj_value = (this->pm10.slope * this->pm10.raw_value) + this->pm10.zero;
            }
        }
    } else {
        while(this->serial->available()) {
            this->serial->read();
        }
    }
}