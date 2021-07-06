#include "108L.h"

PAM_108L::PAM_108L() :
    ozone(0x00, 0x00)
{
    this->name = "108L";

    EEPROM.get(OZONE_EN_MEM_ADDRESS, this->ozone_enabled);

    this->ozone.name = "ozone";
    this->ozone.units = "PPB";
    this->ozone.packet_constant = OZONE_PACKET_CONSTANT;
    this->species.push_back(&this->ozone);
}

PAM_108L::~PAM_108L() {}

bool PAM_108L::start() {
    if (this->ozone_enabled == 0)
    {
        Serial.println("Ozone not enabled");
        return false;
    }
    Serial.println("Ozone Enabled");
    return true;
}

bool PAM_108L::measure() {
    if (this->ozone_enabled)
    {
        getEspOzoneData();
        return true;
    }
    return false;
}


void PAM_108L::getEspOzoneData(){
    float ozone_value = 0.0;
    String getOzoneData = "Z&";
    String recievedData = " ";
    bool timeOut = false;
    double counterIndex = 0;
    int NUMBER_OF_FEILDS = 7;
    int NUMBER_OF_FIELDS_LOGGING = 7;
    int MAX_COUNTER_INDEX = 15000;
    //if esp doesn't answer, keep going
    Serial1.setTimeout(3000);
    Serial1.print(getOzoneData);
    while(!Serial1.available() && timeOut == false){
      //delay(1);
      counterIndex++;
      if(counterIndex > MAX_COUNTER_INDEX){
        timeOut = true;
      }
    }

    delay(10);

    recievedData = Serial1.readString();
    //recievedData = "0.1,1.2,3.3,4.5,1.234,10/12/18,9:22:18";

    //parse data if not null
    int comma_count = 0;
    int from_index = 0;
    int index_of_comma = 0;
    bool still_searching_for_commas = true;
    String stringArray[NUMBER_OF_FEILDS];

    while(still_searching_for_commas && comma_count < NUMBER_OF_FEILDS){
        //Serial.printf("From index: %d\n\r", from_index);

        index_of_comma = recievedData.indexOf(',', from_index);

        //if the index of the comma is not zero, then there is data.
        if(index_of_comma > 0){
            stringArray[comma_count] = recievedData.substring(from_index, index_of_comma);
            comma_count++;
            from_index = index_of_comma;
            from_index += 1;
        }else{
            int index_of_cr = recievedData.indexOf('\r', from_index);
            if(index_of_cr > 0){
                stringArray[comma_count] = recievedData.substring(from_index, index_of_cr);
            }
            still_searching_for_commas = false;
        }
    }
    if(comma_count == NUMBER_OF_FIELDS_LOGGING){
        ozone_value = stringArray[1].toFloat();
    }else if(comma_count == (NUMBER_OF_FIELDS_LOGGING - 1)){
        ozone_value = stringArray[0].toFloat();
    }
    this->ozone.adj_value = ozone_value;
}