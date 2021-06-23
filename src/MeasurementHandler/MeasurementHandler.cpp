#include "MeasurementHandler.h"


MeasurementHandler::MeasurementHandler(T6713 T6713, TPHFusion Tph_fusion, Plantower Plantower, PAMCO Pamco)
{
    this->t6713 = T6713;
    this->tph_fusion = Tph_fusion;
    this->plantower = Plantower;
    this->pamco = Pamco;
}

void sendDatatoSdCard()
{
    //************Fill the cloud output array and file output array for row in csv file on usd card*****************************/
    //This is different than the ble packet in that we are putting all of the data that we have in one packet
    //"$1:D555g47.7M-22.050533C550.866638r1R1q2T45.8P844.9h17.2s1842.700000&"
    //get a current time string
    time_t time = Time.now();
    Time.setFormat(TIME_FORMAT_ISO8601_FULL);

    String csv_output_string = "";

    csv_output_string += String(DEVICE_id) + ",";


    csv_output_string += String(this->pamco.co.adj_value, 3) + ",";


    csv_output_string += String(this->t6713.CO2.adj_value, 0) +  ",";

    if(voc_enabled){
        csv_output_string += String(air_quality_score, 1) + ",";
    }

    // csv_output_string += String(PM01Value) + ",";
    csv_output_string += String(this->plantower.pm1.adj_value) + ",";

    // csv_output_string += String(corrected_PM_25, 0) + ",";
    csv_output_string += String(this->plantower.pm2_5.adj_value, 0) + ",";

    // csv_output_string += String(PM10Value) + ",";
    csv_output_string += String(this->plantower.pm10.adj_value) + ",";


    csv_output_string += String(readTemperature(), 1) + ",";

    // csv_output_string += String(bme.pressure / 100.0, 1) + ",";
    csv_output_string += String(this->tph_fusion.pressure->adj_value, 1) + ",";

    csv_output_string += String(readHumidity(), 1) + ",";
    if(ozone_enabled){
        csv_output_string += String(O3_float, 1) + ",";
    }

    csv_output_string += String(this->fuel.getSoC(), 1) + ",";


    csv_output_string += String(sound_average, 0) + ",";


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
        csv_output_string += String(gps.get_horizontalDillution() / 10.0) + ",";
    } else {
        csv_output_string += String(geolocation_accuracy) + ",";
    }

    csv_output_string += String(status_word.status_int) + ",";
    csv_output_string += String(Time.format(time, "%d/%m/%y,%H:%M:%S"));

    PAMSerial.println(rd, csv_output_string);

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


void outputDataToESP(void){
    //used for converting double to bytes for latitude and longitude
    char buffer[2];
    union{
	       double myDouble;
	       unsigned char bytes[sizeof(double)];
    } doubleBytes;
    //doubleBytes.myDouble = double;
    //

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
        wordBytes.myWord = DEVICE_id;
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
        11-sound_average



        */
        if(i == 0){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_MONOXIDE_PACKET_CONSTANT;
            // floatBytes.myFloat = CO_float;
            floatBytes.myFloat = this->pamco.co.adj_value;
        }else if(i == 1){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_DIOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = CO2_float;
        }else if(i == 2){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
            floatBytes.myFloat = this->fuel.getSoC();
        }else if(i == 3){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
            // floatBytes.myFloat = PM01Value;
            floatBytes.myFloat = this->plantower.pm1.adj_value;
        }else if(i == 4){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
            // floatBytes.myFloat = corrected_PM_25;
            floatBytes.myFloat = this->plantower.pm2_5.adj_value;
        }else if(i == 5){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
            // floatBytes.myFloat = PM10Value;
            floatBytes.myFloat = this->plantower.pm10.adj_value;
        }else if(i == 6){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = TEMPERATURE_PACKET_CONSTANT;
            floatBytes.myFloat = readTemperature();
        }else if(i == 7){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PRESSURE_PACKET_CONSTANT;
            // floatBytes.myFloat = bme.pressure / 100.0;
            floatBytes.myFloat = this->tph_fusion.pressure->adj_value;
        }else if(i == 8){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = HUMIDITY_PACKET_CONSTANT;
            floatBytes.myFloat = readHumidity();
        }else if(i == 9){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = SOUND_PACKET_CONSTANT;
            floatBytes.myFloat = sound_average;
        }else if(i == 10){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = VOC_PACKET_CONSTANT;
            floatBytes.myFloat = air_quality_score;
        }/*else if(i == 11){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = OZONE_PACKET_CONSTANT;
            floatBytes.myFloat = O3_float;
        }*/

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

    /*Serial.println("Successfully output BLE string to ESP");
    for(int i=0;i<NUMBER_OF_SPECIES*BLE_PAYLOAD_SIZE;i++){
        Serial.printf("array[%d]:%X ", i, ble_output_array[i]);
        if(ble_output_array[i]=='#')
            Serial.printf("\n\r");
    }
    Serial.println("End of array");*/

}

//todo: average everything except ozone
void outputToCloud(){
    Serial.println("Making the cloud output string: ");
    String cloud_output_string = "";    //create a clean string
    cloud_output_string += '^';         //start delimeter
    cloud_output_string += String(1) + ";";           //header
    cloud_output_string += String(DEVICE_ID_PACKET_CONSTANT) + String(DEVICE_id);   //device id
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(this->pamco.co.adj_value, 3);
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(this->t6713.CO2.adj_value, 0);
    if(voc_enabled){
        cloud_output_string += String(VOC_PACKET_CONSTANT) + String(air_quality_score, 1);
    }
    cloud_output_string += String(PM1_PACKET_CONSTANT) + String(this->plantower.pm1.adj_value);
    cloud_output_string += String(PM2PT5_PACKET_CONSTANT) + String(this->plantower.pm2_5.adj_value, 0);
    cloud_output_string += String(PM10_PACKET_CONSTANT) + String(this->plantower.pm10.adj_value);
    cloud_output_string += String(TEMPERATURE_PACKET_CONSTANT) + String(readTemperature(), 1);
    cloud_output_string += String(PRESSURE_PACKET_CONSTANT) + String(this->tph_fusion.pressure->adj_value, 1);
    cloud_output_string += String(HUMIDITY_PACKET_CONSTANT) + String(readHumidity(), 1);
    if(ozone_enabled){
        cloud_output_string += String(OZONE_PACKET_CONSTANT) + String(O3_float, 1);
    }
    cloud_output_string += String(BATTERY_PACKET_CONSTANT) + String(this->fuel.getSoC(), 1);
    cloud_output_string += String(SOUND_PACKET_CONSTANT) + String(sound_average, 0);
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
        
    Serial.println("This is the cloud output string: ");
    Serial.println(cloud_output_string);
    //This lets us send the info through Wifi through the ESP. 
    //TO DO: Implement this somewhere else. Not in output_cloud, out to ESP.
    //Serial1.println(cloud_output_string);


    if(Particle.connected() && serial_cellular_enabled){
        status_word.status_int |= 0x0002;
        Particle.publish("pamup", cloud_output_string, PRIVATE);
        Particle.process(); //attempt at ensuring the publish is complete before sleeping
        if(debugging_enabled){
          Serial.println("Published data!");
          writeLogFile("Published data!");
        }
    }else{
        if(serial_cellular_enabled == 0){
            if(debugging_enabled){
                Serial.println("Cellular is disabled.");
                writeLogFile("Cellular is disabled.");
                }
        }else{
            status_word.status_int &= 0xFFFD;   //clear the connected bit
            if(debugging_enabled){
                Serial.println("Couldn't connect to particle.");
                writeLogFile("Couldn't connect to particle.");
              }
        }
    }
    CO_sum = 0;
    CO2_sum = 0;
    O3_sum = 0;
}


void send_to_sensible()
{
    char sensible_buf[256];

    JSONBufferWriter writer(sensible_buf, sizeof(sensible_buf) - 1);
    writer.beginObject();
    String device_string = "PAM-" + String(DEVICE_id);
    //String device_time = String(Time.format(time, "%Y/%m/%dT%H:%M:%SZ"));
    //String co2_string = String(CO2_float, 0);
    //String co_string = String(CO_float, 3);
    writer.name("instrumentKey").value(device_string);
    writer.name("datetime").value(String(Time.format(time, "%Y-%m-%dT%H:%M:%SZ")));
    writer.name("CO2").value(String(CO2_float, 0));
    writer.name("CO").value(String(CO_float, 3));
    writer.name("PM1_0").value(String(PM01Value));
    writer.name("PM2_5").value(String(corrected_PM_25, 0)); 
    writer.name("Temp").value(String(readTemperature(), 1));
    writer.name("Press").value(String(bme.pressure / 100.0, 1));
    writer.name("Hmdty").value(String(readHumidity(), 1));
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