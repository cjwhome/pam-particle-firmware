#include "PAMSensor.h"

PAMSensor::PAMSensor() {}
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