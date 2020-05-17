#include "PAMSensor.h"

PAMSensorFeed::PAMSensorFeed(PAMSensor *_sensor): PAMSerialResponder(BYTE) {
    this->sensor = _sensor;
}

PAMSensorFeed::~PAMSensorFeed() {}

void PAMSensorFeed::becomesResponder(uint16_t rd, bool child_returned) {
    PAMSerial.printf(rd, "Constantly printing data for the %s sensor. Press any key to exit.\r\n", this->sensor->name);
    
    char *csv_header = this->sensor->csvHeader();
    PAMSerial.printf(rd, "%s\r\n", csv_header);
    free(csv_header);

    this->rd = rd;
}

void PAMSensorFeed::loop()
{
    if (this->rd == -1 || this->sensor->getSpecies()->size() == 0) return;
    if ((millis() - this->last_feed) < 200) return;

    this->last_feed = millis();
    this->sensor->measure();

    std::vector<PAMSpecie *> *species = this->sensor->getSpecies();
    size_t specie_count = species->size();

    for (size_t i = 0; i < ((int8_t) specie_count) - 1; i++) {
        PAMSpecie *specie = species->at(i);
        PAMSerial.printf(this->rd, "%0.2f,", specie->adj_value);
    }
    PAMSerial.printf(this->rd, "%0.2f\r\n", species->at(specie_count - 1)->adj_value);
}

void PAMSensorFeed::onData(uint16_t rd, uint8_t *data, uint8_t length) {
    this->rd = -1;
    PAMSerial.popResponder();
}
