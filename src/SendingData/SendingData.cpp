#include "SendingData.h"

SendingData::SendingData()
{
    EEPROM.get(DEVICE_ID_MEM_ADDRESS, device_id);
    PAMSensorManager sensorManager();
    std::vector<PAMSensor *> sensors = sensorManager->getSensors();
    PAMSensor *sensor; 
    for (int i = 0; i < sensors.size(); i++)
    {
        sensor = this->sensors[i];
        if (sensor->name == "T6713")
        {
            this->t6713 = sensor;
        }
        else if (sensor->name == "Temp/Press/RH Fusion")
        {
            this->tph_fusion = sensor;
        }
        else if (sensor->name == "Plantower")
        {
            this->plantower = sensor;
        }
        else if (sensor->name == "CO Sensor")
        {
            this->pamco = sensor;
        }
        else if (sensor->name == "CO Sensor 2")
        {
            this->pamco2 = sensor;
        }
        else if (sensor->name == "108L")
        {
            this->pam_108L = sensor;
        }
    }
}

SendingData::~SendingData() {}

SendingData* SendingData::GetInstance()
{
    if (instance == nullptr) {
        instance = new SendingData();
    }
    return instance;
}



void SendingData::SendDataToESP()
{
    if (sample_counter == 99)
    {
        sample_counter = 0;
    }
    sample_counter++;

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
        wordBytes.myWord = this->device_id;
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
            floatBytes.myFloat = this->pamco.co.average;
        }else if(i == 1){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_DIOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = this->t6713.CO2.average;
        }else if(i == 2){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
            floatBytes.myFloat = fuel.getSoC();
        }else if(i == 3){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
            floatBytes.myFloat = this->plantower.pm1.average;
        }else if(i == 4){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
            floatBytes.myFloat = this->plantower.pm2_5.average;
        }else if(i == 5){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
            floatBytes.myFloat = this->plantower.pm10.average;
        }else if(i == 6){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = TEMPERATURE_PACKET_CONSTANT;
            floatBytes.myFloat = this->tph_fusion.temperature.average;
        }else if(i == 7){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PRESSURE_PACKET_CONSTANT;
            floatBytes.myFloat = this->tph_fusion.pressure.average / 100.0;
        }else if(i == 8){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = HUMIDITY_PACKET_CONSTANT;
            floatBytes.myFloat = this->tph_fusion.humidity.average;
        }else if(i == 9){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = VOC_PACKET_CONSTANT;
            floatBytes.myFloat = this->tph_fusion.air_quality_score.average;
        }else if(i == 10){
            if (ozone_enabled)
            {
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = OZONE_PACKET_CONSTANT;
                floatBytes.myFloat = this->pam_108L.ozone.adj_value;
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
        ble_output_array[19 + i*(BLE_PAYLOAD_SIZE)] = status_word.byte[1];
        ble_output_array[20 + i*(BLE_PAYLOAD_SIZE)] = status_word.byte[0];

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
    cloud_output_string += String(DEVICE_ID_PACKET_CONSTANT) + String(this->device_id);   //device id
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(this->pamco.co.average, 3);

    if (this->pamco2 != NULL)
    {
        cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(this->pamco2.co.average, 3);
    }

    cloud_output_string += String(CARBON_DIOXIDE_PACKET_CONSTANT) + String(this->t6713.CO2.average, 0);
    if(voc_enabled){
        cloud_output_string += String(VOC_PACKET_CONSTANT) + String(this->tph_fusion.air_quality_score.average, 1);
    }
    cloud_output_string += String(PM1_PACKET_CONSTANT) + String(this->plantower.pm1.average);
    cloud_output_string += String(PM2PT5_PACKET_CONSTANT) + String(this->plantower.pm2_5.average, 0);
    cloud_output_string += String(PM10_PACKET_CONSTANT) + String(this->plantower.pm10.average);
    cloud_output_string += String(TEMPERATURE_PACKET_CONSTANT) + String(this->tph_fusion.temperature.average, 1);
    cloud_output_string += String(PRESSURE_PACKET_CONSTANT) + String(this->tph_fusion.pressure.average / 100.0, 1);
    cloud_output_string += String(HUMIDITY_PACKET_CONSTANT) + String(this->tph_fusion.humidity.average, 1);
    if(ozone_enabled){
        cloud_output_string += String(OZONE_PACKET_CONSTANT) + String(this->tph_fusion.ozone.adj_value, 1);
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
        cloud_output_string += String(geolocation_latitude);
    }

    cloud_output_string += String(LONGITUDE_PACKET_CONSTANT);

    if(gps.get_longitude() != 0){
        if(gps.get_ewIndicator() == 0x01){
            cloud_output_string += "-";
        }
        cloud_output_string += String(gps.get_longitude());
    }else{
        cloud_output_string += String(geolocation_longitude);
    }

    cloud_output_string += String(ACCURACY_PACKET_CONSTANT);
    if (gps.get_longitude() != 0) {
        cloud_output_string += String(gps.get_horizontalDillution() / 10.0);
    } else {
        cloud_output_string += String(geolocation_accuracy);
    }
    cloud_output_string += String(PARTICLE_TIME_PACKET_CONSTANT) + String(Time.now());
    cloud_output_string += '&';

    if (Particle.connected() && serial_cellular_enabled){
        status_word.status_int |= 0x0002;
        Particle.publish("pamup", cloud_output_string, PRIVATE);
        Particle.process(); //attempt at ensuring the publish is complete before sleeping
	}
    else if(esp_wifi_connection_status){
        if(debugging_enabled){
            Serial.println("Sending data to esp to upload via wifi...");
            writeLogFile("Sending data to esp to upload via wifi");
          }
        Serial1.println(cloud_output_string);
    }
}

void SendingData::SendDataToSd()
{
    String csv_output_string = "";
    csv_output_string += String(this->device_id) + ",";
    csv_output_string += String(this->pamco.co.average, 3) + ",";
    if (this->pamco2 != NULL)
    {
        csv_output_string += String(this->pamco2.co.average, 3) + ",";
    }

    csv_output_string += String(this->t6713.CO2.average, 0) + ",";

    if(voc_enabled){
        csv_output_string += String(this->tph_fusion.air_quality_score.average, 1) + ",";
    }

    csv_output_string += String(this->plantower.pm1.average) + ",";
    csv_output_string += String(this->plantower.pm2_5.average, 0) + ",";
    csv_output_string += String(this->plantower.pm10.average) + ",";
    csv_output_string += String(this->tph_fusion.temperature.average, 1) + ",";
    csv_output_string += String(this->tph_fusion.pressure.average / 100.0, 1) + ",";

    csv_output_string += String(this->tph_fusion.humidity.average, 1) + ",";
    if(ozone_enabled){
        csv_output_string += String(this->pam_108L.ozone->adj_value, 1) + ",";
    }

    csv_output_string += String(fuel.getSoC(), 1) + ",";

    //csv_output_string += String(sound_average, 0) + ",";

    if(gps.get_latitude() != 0){
        if(gps.get_nsIndicator() == 0){
            csv_output_string += "-";
        }
        csv_output_string += String(gps.get_latitude()) + ",";
    }else{
        csv_output_string += String(geolocation_latitude)+ ",";
    }

    if(gps.get_longitude() != 0){
        if(gps.get_ewIndicator() == 0x01){
            csv_output_string += "-";
        }
        csv_output_string += String(gps.get_longitude()) + ",";
    }else{
        csv_output_string += String(geolocation_longitude) + ",";
    }
    if (gps.get_longitude() != 0) {
        //csv_output_string += String(gps.get_horizontalDillution() / 10.0) + ",";
    } else {
        //csv_output_string += String(geolocation_accuracy) + ",";
    }

    //csv_output_string += String(status_word.status_int) + ",";
    csv_output_string += String(Time.format(Time.now(), "%d/%m/%y,%H:%M:%S"));
    
    Serial.println(csv_output_string);

    //write data to file
    if (sd.begin(CS)){
        if(debugging_enabled)
            Serial.println("Writing row to file.");
        file.open(fileName, O_CREAT | O_APPEND | O_WRITE);
        if(file_started == 0){
            file.println("File Start timestamp: ");
            file.println(Time.timeStr());
            file.println(String(HEADER_STRING));
            file_started = 1;
        }
        file.println(csv_output_string);

        file.close();
    }
}

void SendingData::SendDataToSensible()
{
    char sensible_buf[256];

    JSONBufferWriter writer(sensible_buf, sizeof(sensible_buf) - 1);
    writer.beginObject();
    String device_string = "PAM-" + String(this->device_id);
    //String device_time = String(Time.format(time, "%Y/%m/%dT%H:%M:%SZ"));
    //String co2_string = String(CO2_float, 0);
    //String co_string = String(CO_float, 3);
    writer.name("instrumentKey").value(device_string);
    writer.name("datetime").value(String(Time.format(Time.now(), "%Y-%m-%dT%H:%M:%SZ")));
    writer.name("CO2").value(String(this->t6713.CO2.average, 0));
    writer.name("CO").value(String(this->pamco.co.average, 3));
    writer.name("PM1_0").value(String(this->plantower.pm1.average));
    writer.name("PM2_5").value(String(this->plantower.pm10.average, 0)); 
    writer.name("Temp").value(String(this->tph_fusion.temperature.average, 1));
    writer.name("Press").value(String(this->tph_fusion.pressure.average / 100.0, 1));
    writer.name("Hmdty").value(String(this->tph_fusion.humidity.average, 1));
    //add gps coordinates to json:
    if(gps.get_latitude() != 0){
        if(gps.get_nsIndicator() == 0){
            latitude_string += "-";
        }
    
        latitude_string += String(gps.get_latitude());
    }else{
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

    Particle.publish("sensiblePamUp", sensible_data, PRIVATE);
    Particle.process();
}