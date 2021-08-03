#include "PAMSensorManager.h"

PAMSensorManager *PAMSensorManager::instance = nullptr;
bool PAMSensorManager::isInitialized = false;

PAMSensorManager::PAMSensorManager() { 
    this-> serial_menu_rd = PAMSerial.registerResponder(&this->serial_menu);
}
PAMSensorManager::~PAMSensorManager()
{
    isInitialized = false; 
}

PAMSensorManager* PAMSensorManager::GetInstance()
{
    if (isInitialized == false) {
        isInitialized = true;
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

    sensor->registerSpecieSettings();

    size_t device_setting_name_length = strlen(sensor->name) + 10;
    char *device_setting_name = (char *) malloc(device_setting_name_length);
    memset(device_setting_name, 0, device_setting_name_length);
    snprintf(device_setting_name, device_setting_name_length, "%s Settings", sensor->name);
    this->serial_menu.addResponder(sensor->serial_menu_rd, device_setting_name);
}

char *PAMSensorManager::csvHeader()
{
    bool first = true;
    char *header = (char *) malloc(128);
    memset(header, 0, 128);
    sprintf(header, "%s", "Device_Id");
    for (size_t i = 0; i < this->sensors.size(); i++) {
        PAMSensor *sensor = this->sensors[i];

        char *sensor_header = sensor->csvHeader();
        // if (first) {
        //     first = false;
        //     sprintf(header, "%s", sensor_header);
        // } else {
        //     sprintf(header, "%s,%s", header, sensor_header);
        // }
        sprintf(header, "%s,%s", header, sensor_header);
        free(sensor_header);
    }
    sprintf(header, "%s,%s", header, "Battery,Latitude,Longitude,Date,Time");
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

PAMSpecie * PAMSensorManager::findSpecieByName(char *name)
{
    PAMSpecie * specie;

    for(size_t i = 0; i < this->sensors.size(); i++)
    {
        PAMSensor * sensor = this->sensors[i];
        for(size_t j = 0; j < sensor->getSpecies()->size(); j++)
        {
            specie = sensor->getSpecies()->at(j);
            if (strcmp(name, specie->name) == 0)
            {
                return specie;
            }
        }
    }
    return NULL;
}

void PAMSensorManager::loop() {
    for (size_t i = 0; i < this->sensors.size(); i++) {
        PAMSensor *sensor = this->sensors[i];
        sensor->loop();
    }
}

std::vector<PAMSensor *> PAMSensorManager::getSensors()
{
    return this->sensors;
}

void PAMSensorManager::runAllAverages()
{
    std::vector<PAMSpecie *> *species = new std::vector<PAMSpecie *>();

    for (size_t i = 0; i < this->sensors.size(); i++) {
        PAMSensor *sensor = this->sensors[i];
        for (size_t j = 0; j < sensor->getSpecies()->size(); j++) {
            PAMSpecie *specie = sensor->getSpecies()->at(j);
            specie->averaged_value();
        }
    }
}