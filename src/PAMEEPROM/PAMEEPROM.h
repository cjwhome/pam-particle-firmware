#ifndef __PAM_EEPROM_H__
#define __PAM_EEPROM_H__

#include "Particle.h"

#define PAMEEPROM_readFixedPoint(address, default_value) (PAMEEPROM<int>::ReadStoredVar(address, default_value) / 100.0)
#define PAMEEPROM_writeFixedPoint(address, value) PAMEEPROM<int>::WriteStoredVar(address, (int) (value * 100))

template<typename T>
class PAMEEPROM {

public:
    PAMEEPROM();
    ~PAMEEPROM();

    static int remoteWriteStoredVars(String addressAndValue);
    static int remoteReadStoredVars(String mem_address);
    // static void readStoredVars(void);
    // static void writeDefaultSettings(void);

    static T ReadStoredVar(int address, T default_value, T &target) {
        union {
            T value;
            uint8_t bytes[sizeof(T)];
        } to_bytes;

        EEPROM.get(address, to_bytes.value);
        
        bool is_uninitialized = true;
        for (uint8_t i = 0; i < sizeof(T); i++) {
            // uint8_t check_byte = value >> i;
            uint8_t check_byte = to_bytes.bytes[i];
            is_uninitialized &= (check_byte & 0xFF) == 0xFF;
        }

        if (is_uninitialized) {
            // return default_value;
            target = default_value;
            Serial.println("Using default EEPROM value");
        } else {
            // return value;
            // return to_bytes.value;
            target = to_bytes.value;
            Serial.println("Using existing EEPROM value");
        }
    }
    
    static void WriteStoredVar(int address, T *ptr) {
        EEPROM.put(address, *ptr);
    }

};

#endif // __PAM_EEPROM_H__