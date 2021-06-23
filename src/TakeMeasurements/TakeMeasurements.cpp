#iclude "TakeMeasurements.h"

TakeMeasurements::TakeMeasurements()
{

}


void calculateAQI(void){
    //Calculate humidity contribution to IAQ index
        // gas_reference = bme.gas_resistance/100;
    gas_reference = tph_fusion.voc->adj_value;
      float current_humidity = readHumidity();
      if(debugging_enabled){
          Serial.printf("gas resistance: %1.0f, humidity: %1.2f\n\r", gas_reference, current_humidity);

      }
      if (current_humidity >= 38 && current_humidity <= 42)
        hum_score = 0.25*100; // Humidity +/-5% around optimum
      else
      { //sub-optimal
        if (current_humidity < 38)
          hum_score = 0.25/hum_reference*current_humidity*100;
        else
        {
          hum_score = ((-0.25/(100-hum_reference)*current_humidity)+0.416666)*100;
        }
      }

      //Calculate gas contribution to IAQ index

      if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit;
      if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
      gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;
      if(debugging_enabled){
        Serial.print("Gas score: ");
        Serial.println(gas_score);
        Serial.print("Humidity score: ");
        Serial.println(hum_score);
    }

      //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
      this->air_quality_score = hum_score + gas_score;


}

//read sound from
void readSound(void){
    int val;
    float sum = 0;
    float average = 0;
    for(int i=0; i< 10;i++){
        val = analogRead(SOUND_INPUT);
        sum += val;
        //Serial.print("Sound level: ");
        //Serial.println(val);
    }
    sum = sum/10;
    sum /= 4095;
    sum *= 100;
    this->sound_average;
}

void readOzone(void){
    int tempValue = 0;
    if(ozone_analog_enabled){
        tempValue = analogRead(A0);  // read the analogPin for ozone voltage
        if(debugging_enabled){
            Serial.print("Ozone Raw analog in:");
            Serial.println(tempValue);

        }
        O3_float = tempValue;
        O3_float *= VOLTS_PER_UNIT;   // .0008        //convert digital reading to voltage
        O3_float /= VOLTS_PER_PPB;    // .0125        //convert voltage to ppb of ozone
        O3_float += ozone_offset;
    }else{
        O3_float = getEspOzoneData();
    }
}


void getEspOzoneData(){
    float ozone_value = 0.0;
    String getOzoneData = "Z&";
    String recievedData = " ";
    bool timeOut = false;
    double counterIndex = 0;
    //if esp doesn't answer, keep going
    Serial1.setTimeout(3000);
    if(debugging_enabled){
        Serial.println("Getting ozone data from esp");
        writeLogFile("Getting ozone data from esp");
      }
    Serial1.print(getOzoneData);
    while(!Serial1.available() && timeOut == false){
      //delay(1);
      counterIndex++;
      if(counterIndex > MAX_COUNTER_INDEX){
        if(debugging_enabled){
          Serial.printf("Unable to get ozone data from ESP, counter index: %1.1f\n\r", counterIndex);
        }
        timeOut = true;
      }
    }


    delay(10);

    recievedData = Serial1.readString();
    //recievedData = "0.1,1.2,3.3,4.5,1.234,10/12/18,9:22:18";
    if(debugging_enabled)
    {
        Serial.print("RECIEVED DATA FROM ESP: ");
        Serial.println(recievedData);
        writeLogFile("Recieved data from ESP");
    }
    //parse data if not null
    int comma_count = 0;
    int from_index = 0;
    int index_of_comma = 0;
    bool still_searching_for_commas = true;
    String stringArray[NUMBER_OF_FEILDS];

    while(still_searching_for_commas && comma_count < NUMBER_OF_FEILDS){
        //Serial.printf("From index: %d\n\r", from_index);

        index_of_comma = recievedData.indexOf(',', from_index);
        if(debugging_enabled){
          Serial.print("comma index: ");
          Serial.println(index_of_comma);
          //writeLogFile("got a comma");

        }

        //if the index of the comma is not zero, then there is data.
        if(index_of_comma > 0){
            stringArray[comma_count] = recievedData.substring(from_index, index_of_comma);
            if(debugging_enabled){
                Serial.printf("String[%d]:", comma_count);
                Serial.println(stringArray[comma_count]);
                //writeLogFile(stringArray[comma_count]);
            }
            comma_count++;
            from_index = index_of_comma;
            from_index += 1;
        }else{
            int index_of_cr = recievedData.indexOf('\r', from_index);
            if(index_of_cr > 0){
                stringArray[comma_count] = recievedData.substring(from_index, index_of_cr);
                if(debugging_enabled){
                    Serial.printf("String[%d]:", comma_count);
                    Serial.println(stringArray[comma_count]);
                }
            }
            still_searching_for_commas = false;
        }
    }
    if(comma_count == NUMBER_OF_FIELDS_LOGGING){
        ozone_value = stringArray[1].toFloat();
        if(debugging_enabled){
            Serial.println("using string array index 1 due to logging");
            //writeLogFile("using string array index 1 due to logging");
          }
    }else if(comma_count == (NUMBER_OF_FIELDS_LOGGING - 1)){
        ozone_value = stringArray[0].toFloat();
        if(debugging_enabled){
            Serial.println("using string array index 0, not logging");
            //writeLogFile("using string array index 0, not logging");
          }
    }
    this->ozone;
    //parseOzoneString(recievedData);
}


void readCO2(T6713 t6713){
    t6713.measure();
    if (t6713.CO2.adj_value != 0 && t6713.CO2.adj_value != VALUE_UNKNOWN) {
        CO2_float = t6713.CO2.adj_value;
    }
    this->co2;
}