#include "PAMSensorManager.h"

PAMSensorManager *PAMSensorManager::instance = nullptr;

PAMSensorManager::PAMSensorManager() { Serial.println("[PAMSensorManager]::init"); }
PAMSensorManager::~PAMSensorManager() {}

PAMSensorManager* PAMSensorManager::GetInstance()
{
    if (instance == nullptr) {
        instance = new PAMSensorManager();
    }
    return instance;
}

void PAMSensorManager::addSensor(PAMSensor *sensor)
{
    this->sensors.push_back(sensor);
    if (sensor->start()) {
        sensor->state = PAMSensor::SensorState::IDLE;
    } else {
        sensor->state = PAMSensor::SensorState::ERROR;
    }
}

char *PAMSensorManager::csvHeader()
{
    bool first = true;
    char *header = (char *) malloc(128);
    memset(header, 0, 128);

    for (size_t i = 0; i < this->sensors.size(); i++) {
        PAMSensor *sensor = this->sensors[i];
        char *sensor_header = sensor->csvHeader();
        if (first) {
            first = false;
            sprintf(header, "%s", sensor_header);
        } else {
            sprintf(header, "%s,%s", header, sensor_header);
        }
        free(sensor_header);
    }
    
    return header;
}

std::vector<PAMSpecie *>* PAMSensorManager::findSpeciesForName(char *name)
{
    std::vector<PAMSpecie *> *species = new std::vector<PAMSpecie *>();

    for (size_t i = 0; i < this->sensors.size(); i++) {
        PAMSensor *sensor = this->sensors[i];
        for (size_t j = 0; j < sensor->getSpecies()->size(); j++) {
            PAMSpecie *specie = sensor->getSpecies()->at(j);
            if (strcmp(name, specie->name) == 0) {
                species->push_back(specie);
            }
        }
    }

    return species;
}

void PAMSensorManager::loop() {
    for (size_t i = 0; i < this->sensors.size(); i++) {
        PAMSensor *sensor = this->sensors[i];
        sensor->loop();
    }

    if (millis() > (this->last_loop_ms + this->measurement_period_ms)) {
        this->last_loop_ms = millis();

        char *serial_line = (char *) malloc(256);
        memset(serial_line, 0, 256);
        sprintf(serial_line, "%d", 1011);

        for (size_t i = 0; i < this->sensors.size(); i++) {
            PAMSensor *sensor = this->sensors[i];
            sensor->state = PAMSensor::SensorState::MEASURING;
            if (sensor->measure()) {
                sensor->state = PAMSensor::SensorState::IDLE;
            } else {
                sensor->state = PAMSensor::SensorState::ERROR;
            }

            for (size_t j = 0; j < sensor->getSpecies()->size(); j++) {
                PAMSpecie *specie = sensor->getSpecies()->at(j);
                sprintf(serial_line, "%s,%0.2f", serial_line, specie->adj_value);
            }
        }
        Serial.println(serial_line);
    }
}