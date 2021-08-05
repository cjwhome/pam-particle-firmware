#include "SendingData.h"

SendingData::SendingData()
{
}

SendingData::~SendingData() 
{
    isInitialized = false; 
}

SendingData * SendingData::instance = nullptr;
bool SendingData::isInitialized = false;
SendingData* SendingData::GetInstance()
{
    if (isInitialized == false) {
        isInitialized = true;
        instance = new SendingData();
    }
    return instance;
}

void SendingData::addSensors() 
{
    std::vector<PAMSensor *> sensors = sensorManager->getSensors();
    PAMSensor *sensor; 
    for (int i = 0; i < sensors.size(); i++)
    {
        sensor = sensors[i];
        if (sensor->name == "CO2 Sensor")
        {
            this->t6713 = (T6713*)(sensor);
        }
        else if (sensor->name == "Temp/Press/RH Fusion")
        {
            this->tph_fusion = (TPHFusion*)(sensor);
        }
        else if (sensor->name == "Plantower")
        {
            this->plantower = (Plantower*)(sensor);
        }
        else if (sensor->name == "CO Sensor")
        {
            this->pamco = (PAMCO*)(sensor);
        }
        else if (sensor->name == "NO2 Sensor")
        {
            this->pamno2 = (PAMNO2*)(sensor);
        }
        else if (sensor->name == "108L")
        {
            this->pam_108L = (PAM_108L*)(sensor);
        }
        else {
            Serial.println("Didn't find a sensor matching: ");
            Serial.println(sensor->name);
        }
    }
}



void SendingData::SendDataToESP()
{
    if (sample_counter == 99)
    {
        sample_counter = 0;
    }
    sample_counter++;



    //used for converting float to bytes for measurement value
    union {
        float myFloat;
        unsigned char bytes[4];
    } floatBytes;

    //used for converting word to bytes for lat and longitude
    union {
        int16_t myWord;
        unsigned char bytes[2];
    }wordBytes;

    //create an array of binary data to store and send all data at once to the ESP
    //Each "section" in the array is separated by a #
    //we are using binary for the ble packets so we can compress the data into 19 bytes for the small payload

    byte ble_output_array[NUMBER_OF_SPECIES*BLE_PAYLOAD_SIZE];     //19 bytes per data line and 12 species to output


    for(int i=0; i<NUMBER_OF_SPECIES; i++){

        //************Fill the ble output array**********************//
        //Serial.printf("making array[%d]\n", i);
        //byte 0 - version
        ble_output_array[0 + i*(BLE_PAYLOAD_SIZE)] = 1;

        //bytes 1,2 - Device ID
        //DEVICE_id = 555;
        wordBytes.myWord = this->globalVariables->device_id;
        ble_output_array[1 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[2 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        //byte 3 - Measurement number
        ble_output_array[3 + i*(BLE_PAYLOAD_SIZE)] = sample_counter;

        //byte 4 - Identifier (B:battery, a:Latitude, o:longitude,
        //t:Temperature, P:Pressure, h:humidity, s:Sound, O:Ozone,
        //C:CO2, M:CO, r:PM1, R:PM2.5, q:PM10, g:VOCs)
        /*
        0-CO_float
        1-CO2_float
        2-bme.gas_resistance / 1000.0
        3-PM01Value
        4-PM2_5Value
        5-PM10Value
        6-bme.temperature
        7-bme.pressure / 100.0
        8-bme.humidity
        9-O3_float
        10-fuel.getSoC()
        */
        if(i == 0){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_MONOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = this->pamco->co.adj_value;
        }else if(i == 1){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_DIOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = this->t6713->CO2.adj_value;
        }else if(i == 2){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
            floatBytes.myFloat = fuel.getSoC();
        }else if(i == 3){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
            floatBytes.myFloat = this->plantower->pm1.adj_value;
        }else if(i == 4){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
            floatBytes.myFloat = this->plantower->pm2_5.adj_value;
        }else if(i == 5){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
            floatBytes.myFloat = this->plantower->pm10.adj_value;
        }else if(i == 6){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = TEMPERATURE_PACKET_CONSTANT;
            floatBytes.myFloat = this->tph_fusion->temperature->adj_value;
        }else if(i == 7){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PRESSURE_PACKET_CONSTANT;
            floatBytes.myFloat = this->tph_fusion->pressure->adj_value / 100.0;
        }else if(i == 8){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = HUMIDITY_PACKET_CONSTANT;
            floatBytes.myFloat = this->tph_fusion->humidity->adj_value;
        }
        else if (i ==9)
        {
            if (this->globalVariables->no2_enabled == true)
            {
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = NO2_PACKET_CONSTANT;
                floatBytes.myFloat = this->pamno2->no2.adj_value;
            }
        }
        else if(i == 10){
            if (this->globalVariables->ozone_enabled)
            {
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = OZONE_PACKET_CONSTANT;
                floatBytes.myFloat = this->pam_108L->ozone.adj_value;
            }

        }

        //bytes 5,6,7,8 - Measurement Value
        ble_output_array[5 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[0];
        ble_output_array[6 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[1];
        ble_output_array[7 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[2];
        ble_output_array[8 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[3];


        //bytes 9-12 - latitude
        wordBytes.myWord = gps.get_latitudeWhole();
        ble_output_array[9 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[10 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        wordBytes.myWord = gps.get_latitudeFrac();
        ble_output_array[11 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[12 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        //bytes 14-17 - longitude
        wordBytes.myWord = gps.get_longitudeWhole();
        ble_output_array[13 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[14 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        wordBytes.myWord = gps.get_longitudeFrac();
        ble_output_array[15 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[16 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];


        //byte 18 - east west and north south indicator
        //  LSB 0 = East, LSB 1 = West
        //  MSB 0 = South, MSB 1 = North
        int northSouth = gps.get_nsIndicator();
        int eastWest = gps.get_ewIndicator();

        ble_output_array[17 + i*(BLE_PAYLOAD_SIZE)] = northSouth | eastWest;
        ble_output_array[18 + i*(BLE_PAYLOAD_SIZE)] = gps.get_horizontalDillution();
        ble_output_array[19 + i*(BLE_PAYLOAD_SIZE)] = this->globalVariables->status_word->byte[1];
        ble_output_array[20 + i*(BLE_PAYLOAD_SIZE)] = this->globalVariables->status_word->byte[0];

        ble_output_array[21 + i*(BLE_PAYLOAD_SIZE)] = '#';     //delimeter for separating species

    }

    //send start delimeter to ESP
    Serial1.print("$");
    //send the packaged data with # delimeters in between packets
    Serial1.write(ble_output_array, NUMBER_OF_SPECIES*BLE_PAYLOAD_SIZE);

    //send ending delimeter
    Serial1.print("&");


}


void SendingData::SendDataToParticle()
{
    String cloud_output_string = "";    //create a clean string
    cloud_output_string += '^';         //start delimeter
    cloud_output_string += String(1) + ";";           //header
    cloud_output_string += String(DEVICE_ID_PACKET_CONSTANT) + String(this->globalVariables->device_id);   //device id
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(this->pamco->co.average, 3);
    cloud_output_string += String(CARBON_DIOXIDE_PACKET_CONSTANT) + String(this->t6713->CO2.average, 0);

    cloud_output_string += String(PM1_PACKET_CONSTANT) + String(this->plantower->pm1.average);
    cloud_output_string += String(PM2PT5_PACKET_CONSTANT) + String(this->plantower->pm2_5.average, 0);
    cloud_output_string += String(PM10_PACKET_CONSTANT) + String(this->plantower->pm10.average);
    cloud_output_string += String(TEMPERATURE_PACKET_CONSTANT) + String(this->tph_fusion->temperature->average, 1);

    cloud_output_string += String(PRESSURE_PACKET_CONSTANT) + String(this->tph_fusion->pressure->average / 100.0, 1);
    cloud_output_string += String(HUMIDITY_PACKET_CONSTANT) + String(this->tph_fusion->humidity->average, 1);
    if (this->globalVariables->no2_enabled)
    {
        cloud_output_string += String(NO2_PACKET_CONSTANT) + String(this->pamno2->no2.average, 1);
    }
    if(this->globalVariables->ozone_enabled){
        cloud_output_string += String(OZONE_PACKET_CONSTANT) + String(this->pam_108L->ozone.average, 1);
    }
    cloud_output_string += String(BATTERY_PACKET_CONSTANT) + String(fuel.getSoC(), 1);
    //cloud_output_string += String(SOUND_PACKET_CONSTANT) + String(sound_average, 0);
    cloud_output_string += String(LATITUDE_PACKET_CONSTANT);
    if(gps.get_latitude() != 0){
        if(gps.get_nsIndicator() == 0){
            cloud_output_string += "-";
        }
        cloud_output_string += String(gps.get_latitude());
    }else{
        cloud_output_string += this->geolocation_latitude;
    }

    cloud_output_string += String(LONGITUDE_PACKET_CONSTANT);

    if(gps.get_longitude() != 0){
        if(gps.get_ewIndicator() == 0x01){
            cloud_output_string += "-";
        }
        cloud_output_string += String(gps.get_longitude());
    }
    cloud_output_string += String(ACCURACY_PACKET_CONSTANT);
    if (gps.get_longitude() != 0) {
        cloud_output_string += String(gps.get_horizontalDillution() / 10.0);
    } else {
        cloud_output_string += this->geolocation_accuracy;
    }
    cloud_output_string += String(PARTICLE_TIME_PACKET_CONSTANT) + String(Time.now());
    cloud_output_string += '&';

    if (Particle.connected()){
        this->globalVariables->status_word->status_int |= 0x0002;
        Particle.publish("pamup", cloud_output_string, PRIVATE);
        Particle.process(); //attempt at ensuring the publish is complete before sleeping
	}
    else if(this->globalVariables->esp_wifi_connection_status){
        if(this->globalVariables->debugging_enabled){
            Serial.println("Sending data to esp to upload via wifi...");
            this->globalVariables->writeLogFile("Sending data to esp to upload via wifi");
          }
        Serial1.println(cloud_output_string);
    }
}

void SendingData::SendDataToSd()
{
    String csv_output_string = "";
    csv_output_string += String(this->globalVariables->device_id) + ",";
    csv_output_string += String(this->pamco->co.adj_value, 3) + ",";
    csv_output_string += String(this->t6713->CO2.adj_value, 0) + ",";
    csv_output_string += String(this->plantower->pm1.adj_value) + ",";
    csv_output_string += String(this->plantower->pm2_5.adj_value, 0) + ",";
    csv_output_string += String(this->plantower->pm10.adj_value) + ",";
    csv_output_string += String(this->tph_fusion->temperature->adj_value, 1) + ",";
    csv_output_string += String(this->tph_fusion->pressure->adj_value / 100.0, 1) + ",";

    csv_output_string += String(this->tph_fusion->humidity->adj_value, 1) + ",";
    if (this->globalVariables->no2_enabled)
    {
        csv_output_string += String(this->pamno2->no2.adj_value, 3) + ",";
    }
    if(this->globalVariables->ozone_enabled)
    {
        Serial.println("getting ozone");
        csv_output_string += String(this->pam_108L->ozone.adj_value, 1) + ",";
    }

    csv_output_string += String(fuel.getSoC(), 1) + ",";

    double latitude = gps.get_latitude();
    if(latitude == 0)
    {
        csv_output_string += this->geolocation_latitude+ ",";
    }
    else 
    {
        if(gps.get_nsIndicator() == 0){
            csv_output_string += "-";
        }
        Serial.println("We are in gps for some reason. here is what it is: ");
        Serial.println(latitude);
        csv_output_string += String(latitude) + ",";
    }
    double longitude = gps.get_longitude();
    if(longitude == 0)
    {
        csv_output_string += this->geolocation_longitude+ ",";
    }
    else
    {
        if(gps.get_ewIndicator() == 0x01){
            csv_output_string += "-";
        }
        csv_output_string += String(longitude) + ",";
    }
    csv_output_string += String(Time.format(Time.now(), "%d/%m/%y,%H:%M:%S"));
    
    if (globalVariables->inMenu == false)
    {
        Serial.println(csv_output_string);
    }

    SdFile file = this->globalVariables->file;

    //write data to file
    if (this->globalVariables->sd.begin(CS)){
        SdFile::dateTimeCallback(this->globalVariables->dateTime);
        file.open(this->globalVariables->fileName, O_CREAT | O_APPEND | O_WRITE);
        if(this->globalVariables->file_started == false){
            file.println("File Start timestamp: ");
            file.println(Time.timeStr());
            file.println(String(sensorManager->csvHeader()));
            this->globalVariables->file_started = true;
        }
        file.println(csv_output_string);

        file.close();
    }
}

void SendingData::SendDataToSensible()
{
    String latitude_string = "";
    String longitude_string = "";
    char sensible_buf[256];

    JSONBufferWriter writer(sensible_buf, sizeof(sensible_buf) - 1);
    writer.beginObject();
    String device_string = "PAM-" + String(this->globalVariables->device_id);
    //String device_time = String(Time.format(time, "%Y/%m/%dT%H:%M:%SZ"));
    //String co2_string = String(CO2_float, 0);
    //String co_string = String(CO_float, 3);
    writer.name("instrumentKey").value(device_string);
    writer.name("datetime").value(String(Time.format(Time.now(), "%Y-%m-%dT%H:%M:%SZ")));
    writer.name("CO2").value(String(this->t6713->CO2.average, 0));
    writer.name("CO").value(String(this->pamco->co.average, 3));
    writer.name("PM1_0").value(String(this->plantower->pm1.average));
    writer.name("PM2_5").value(String(this->plantower->pm10.average, 0)); 
    writer.name("Temp").value(String(this->tph_fusion->temperature->average, 1));
    writer.name("Press").value(String(this->tph_fusion->pressure->average / 100.0, 1));
    writer.name("Hmdty").value(String(this->tph_fusion->humidity->average, 1));
    writer.name("Battery").value(String(fuel.getSoC(), 2));
    if (this->globalVariables->no2_enabled)
    {
        writer.name("NO2").value(String(this->pamno2->no2.average, 1));
    }
    if (this->globalVariables->ozone_enabled)
    {
        writer.name("ozone").value(String(this->pam_108L->ozone.average, 1));
    }
    //add gps coordinates to json:
    if(gps.get_latitude() != 0){
        if(gps.get_nsIndicator() == 0){
            latitude_string += "-";
        }
        latitude_string += String(gps.get_latitude());
    }
    else{
        latitude_string = "";
    }
    writer.name("Lat").value(latitude_string);

    if(gps.get_longitude() != 0){
        if(gps.get_ewIndicator() == 0x01){
            longitude_string += "-";
            
        }
        longitude_string += String(gps.get_longitude());
    }  
      
    writer.name("Long").value(longitude_string);
    
    
    writer.endObject();
    writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;


    if (Particle.connected())
    {
        Particle.publish("sensiblePamUp", sensible_buf, PRIVATE);
        Particle.process();
    }
}

