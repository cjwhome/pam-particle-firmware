#include "gps.h"



//set decimal value of latitude from NMEA string
void GPS::set_lat_decimal(String latString, char nsString){
    String whole_str = latString.substring(2,4);
    String frac_str = latString.substring(4,10);
    latWhole = whole_str.toInt();
    latFrac = frac_str.toInt();

    int whole_part = whole_str.toInt();
    // Serial.print("Whole part:");
    // Serial.println(whole_part);

    double frac_part = frac_str.toFloat();
    // Serial.print("Frac part:");
    // Serial.println(frac_part, 5);


    latitude = whole_part;
    latitude += (frac_part)/60;
    if(nsString == 'S'){
        ns_indicator = 0;
    }else{
        ns_indicator = 0x80;
    }
}

void GPS::set_long_decimal(String longString, char ewString){
    String whole_str = longString.substring(2,5);
    String frac_str = longString.substring(5,10);

    longWhole = whole_str.toInt();
    longFrac = frac_str.toInt();
    int whole_part = whole_str.toInt();
    // Serial.print("Whole string: ");
    // Serial.println(whole_str);
    // Serial.print("Whole part:");
    // Serial.println(whole_part);

    double frac_part = frac_str.toFloat();
    // Serial.print("Frac part:");
    // Serial.println(frac_part, 5);


    longitude = whole_part;
    longitude += (frac_part)/60;
    if(ewString == 'E'){
      ew_indicator = 0;
    }else{
      ew_indicator = 0x01;
    }
}

void GPS::set_satellites(String satString){
    satellites_used = satString.toInt();
}

void GPS::set_horizontalDilution(String hdString){
    float temp_float = hdString.toFloat();
    temp_float *= 10;

    horizontal_dilution = temp_float;
}

double GPS::get_latitude(void){
    return latitude;
}

double GPS::get_longitude(void){
    return longitude;
}

int GPS::get_satellites(void){
    return satellites_used;
}

int GPS::get_horizontalDilution(void)
{
    return horizontal_dilution;
}

int16_t GPS::get_latitudeWhole(void){
    return latWhole;
}
int16_t GPS::get_latitudeFrac(void){
    return latFrac;
}
int16_t GPS::get_longitudeWhole(void){
    return longWhole;
}
int16_t GPS::get_longitudeFrac(void){
    return longFrac;
}

int8_t GPS::get_nsIndicator(void){
    return ns_indicator;
}

int8_t GPS::get_ewIndicator(void){
    return ew_indicator;
}
