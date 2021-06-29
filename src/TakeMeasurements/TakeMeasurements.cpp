#include "TakeMeasurements.h"

TakeMeasurements::TakeMeasurements(T6713 *T6713, TPHFusion *Tph_fusion, Plantower *Plantower, PAMCO *Pamco)
{
    this->t6713 = T6713;
    this->tph_fusion = Tph_fusion;
    this->plantower = Plantower;
    this->pamco = Pamco;
}

TakeMeasurements::~TakeMeasurements() {}

float TakeMeasurements::get_air_quality_score()
{
  return this->air_quality_score;
}

float TakeMeasurements::get_sound_average()
{
  return this->sound_average;
}

float TakeMeasurements::get_ozone()
{
  return this->ozone;
}

float TakeMeasurements::get_co2()
{
  return this->co2;
}



void TakeMeasurements::calculateAQI(void){
  float hum_score;
  float gas_reference = 250000;
  float hum_reference = 40; 
  //Calculate humidity contribution to IAQ index
  gas_reference = tph_fusion->voc->adj_value;
  float current_humidity = this->tph_fusion->humidity->adj_value;
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
  float gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;

  //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
  this->air_quality_score = hum_score + gas_score;


}

//read sound from
void TakeMeasurements::readSound(void){
    int val;
    float sum = 0;
    for(int i=0; i< 10;i++){
        val = analogRead(SOUND_INPUT);
        sum += val;
        //Serial.print("Sound level: ");
        //Serial.println(val);
    }
    sum = sum/10;
    sum /= 4095;
    sum *= 100;
    this->sound_average = sum;
}

void TakeMeasurements::readOzone(void){
    int tempValue = analogRead(A0);  // read the analogPin for ozone voltage
    float volts_per_unit = .0008;
    float volts_per_pbb = .0125;

    this->ozone = tempValue;
    this->ozone *= volts_per_unit;   // .0008        //convert digital reading to voltage
    this->ozone /= volts_per_pbb;    // .0125        //convert voltage to ppb of ozone
    this->ozone += ozone_offset;

}


void TakeMeasurements::getEspOzoneData(){
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
    this->ozone = ozone_value;
    //parseOzoneString(recievedData);
}


void TakeMeasurements::readCO2(){
    this->t6713->measure();
    if (this->t6713->CO2.adj_value != 0 && this->t6713->CO2.adj_value != VALUE_UNKNOWN) {
        this->co2 = this->t6713->CO2.adj_value;
    }
    else 
    {
      this->co2 = 0;
    }
}