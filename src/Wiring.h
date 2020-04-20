#ifndef __WIRING_H__
#define __WIRING_H__

#include "Particle.h"

#define LMP91000_1_EN       (B0)
#define ADS1115_1_ADDR      (0x49)

#define LMP91000_2_EN       (B2)
#define ADS1115_2_ADDR      (0x4A)

#define FIVE_VOLT_EN        (D5)
#define PLANTOWER_EN        (B4)
#define POWER_LED_EN        (D6)
#define ESP_WROOM_EN        (D7)
#define BLOWER_EN           (D2)
#define CO2_EN              (C5)        //enables the CO2 sensor power

#define KILL_POWER          (WKP)
#define SOUND_INPUT         (B5)  //ozone monitor's voltage output is connected to this input

#endif // __WIRING_H__