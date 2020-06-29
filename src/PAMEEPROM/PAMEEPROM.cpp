#include "PAMEEPROM.h"

#define MAX_MEM_ADDRESS (2047)

template<typename T>
PAMEEPROM<T>::PAMEEPROM() {};

template<typename T>
PAMEEPROM<T>::~PAMEEPROM() {};

template<typename T>
int PAMEEPROM<T>::remoteWriteStoredVars(String addressAndValue)
{
    uint16_t tempValue = 0;

    int index_of_comma = addressAndValue.indexOf(',');
    Serial.print("Full address and value substring: ");
    Serial.println(addressAndValue);
    String addressString = addressAndValue.substring(0, index_of_comma);
    String valueString = addressAndValue.substring(index_of_comma + 1);

    Serial.printf("address substring: %s\n\r", addressString);
    Serial.printf("Value substring: %s\n\r", valueString);

    int numerical_mem_address = addressString.toInt();
    int numerical_value = valueString.toInt();

    if(numerical_mem_address >= 0 && numerical_mem_address <= MAX_MEM_ADDRESS){
        EEPROM.put(numerical_mem_address, numerical_value);
        return 1;
    }else{
        return -1;
    }
}

template<typename T>
int PAMEEPROM<T>::remoteReadStoredVars(String mem_address)
{
    uint16_t tempValue = 0;
    int numerical_mem_address = mem_address.toInt();
    if(numerical_mem_address >= 0 && numerical_mem_address <= MAX_MEM_ADDRESS){
        EEPROM.get(numerical_mem_address, tempValue);
        return tempValue;
    }else{
        return -1;
    }
}
