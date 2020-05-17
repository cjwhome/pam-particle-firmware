#include "PAMSensor.h"

PAMSensor::PAMSensor()
{
    this->serial_menu_rd = PAMSerial.registerResponder(&this->serial_menu);
    this->sensor_feed = new PAMSensorFeed(this);
    this->serial_menu.addResponder(this->sensor_feed, "Output species constantly and rapidly");
}

PAMSensor::~PAMSensor() {}

char *PAMSensor::csvHeader() {
    bool first = true;
    char *header = (char *) malloc(128);
    memset(header, 0, 128);

    for (size_t i = 0; i < this->species.size(); i++) {
        PAMSpecie *specie = this->species[i];
        if (first) {
            first = false;
            sprintf(header, "%s", specie->name);
        } else {
            sprintf(header, "%s,%s", header, specie->name);
        }
    }

    return header;
}

void PAMSensor::registerSpecieSettings()
{
    for (uint8_t i = 0; i < this->species.size(); i++) {
        PAMSpecie *specie = this->species[i];

        size_t setting_name_length = strlen(specie->name) + 32;
        char *name = (char *) malloc(setting_name_length);
        memset(name, 0, setting_name_length);
        snprintf(name, setting_name_length, "%s Settings", specie->name);

        this->serial_menu.addResponder(specie->serial_menu_rd, name);
    }
}

void PAMSensor::loop() {
    this->sensor_feed->loop();
}