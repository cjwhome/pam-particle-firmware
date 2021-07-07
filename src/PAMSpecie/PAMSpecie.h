/**
 * PAMSpecie.h - Created 04/11/2020
 * Author: David Kopala
 * 
 * Provides a primitive type for describing a specie
 */

#ifndef __PAM_SPECIE_H__
#define __PAM_SPECIE_H__

#include "cmath"

#define VALUE_UNKNOWN -INFINITY

#define DEVICE_ID_PACKET_CONSTANT 'Z'       //instrument ID number as INTEGER
#define PM1_PACKET_CONSTANT 'r'             //PM1 as UG/M3
#define PM2PT5_PACKET_CONSTANT 'R'          //PM2.5 as UG/M3
#define PM10_PACKET_CONSTANT 'q'            //PM10 as UG/M3
#define CO_PACKET_CONSTANT 'M'              // CO as PPM
#define CO2_PACKET_CONSTANT 'C'             // CO2 as PPM
#define TEMPERATURE_PACKET_CONSTANT 't'     //temperature as DEGREES CELSIUS
#define PRESSURE_PACKET_CONSTANT 'P'        //pressure as MILLIBARS
#define HUMIDITY_PACKET_CONSTANT 'h'        //humidity as PERCENTAGE
#define BATTERY_PACKET_CONSTANT 'x'         //Battery in percentage
#define DONT_ADVERTISE_CONSTANT '\0'
#define OZONE_PACKET_CONSTANT 'O'           //Ozone
#define VOC_PACKET_CONSTANT 'g'             //air_quality_score

#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "../PAMSerial/PAMSerialEditEEPROMValue/PAMSerialEditEEPROMValue.h"

class PAMSpecie {

public:
    PAMSpecie(uint16_t slope_address, uint16_t zero_address, float default_slope = 1, float default_zero = 0);
    ~PAMSpecie();

    virtual float averaged_value();

    char *name;
    char *units;

    float zero = 0;
    float slope = 1;

    float raw_value = VALUE_UNKNOWN;
    float adj_value = VALUE_UNKNOWN;
    uint16_t number_of_measures = 0;
    float accumulated_value = 0;

    char packet_constant;
    PAMSerialMenu serial_menu;

    uint16_t serial_menu_rd;
    
private:


    PAMSerialEditEEPROMValue<float> *slope_responder;
    uint16_t slope_rd;

    PAMSerialEditEEPROMValue<float> *zero_responder;
    uint16_t zero_rd;

};

#endif // __PAM_SPECIE_H__