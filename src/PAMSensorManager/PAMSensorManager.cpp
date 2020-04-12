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
    sensor->start();
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