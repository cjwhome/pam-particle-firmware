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

// #include "../../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
// #include "../../PAMSerial/PAMSerialEditValue/PAMSerialEditValue.h"
#include "../PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "../PAMSerial/PAMSerialEditValue/PAMSerialEditValue.h"

class PAMSpecie {

public:
    PAMSpecie();
    ~PAMSpecie();

    char *name;
    char *units;

    float zero = 0;
    float slope = 1;

    float raw_value = VALUE_UNKNOWN;
    float adj_value = VALUE_UNKNOWN;

    char packet_constant;

    uint16_t serial_menu_rd;

private:
    PAMSerialMenu serial_menu;

    PAMSerialEditValue<float> *slope_responder; // { this->slope };
    uint16_t slope_rd;

    PAMSerialEditValue<float> *zero_responder; // { this->zero };
    uint16_t zero_rd;

};

#endif // __PAM_SPECIE_H__