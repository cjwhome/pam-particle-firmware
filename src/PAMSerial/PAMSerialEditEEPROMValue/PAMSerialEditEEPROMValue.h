#ifndef __PAM_SERIAL_EDITEEPROMVALUE_H__
#define __PAM_SERIAL_EDITEEPROMVALUE_H__

#include "../PAMSerialEditValue/PAMSerialEditValue.h"
#include "../PAMSerialResponder.h"
#include "../PAMSerial.h"
#include "../../PAMEEPROM/PAMEEPROM.h"

template<typename T>
class PAMSerialEditEEPROMValue : public PAMSerialResponder {

public:
    PAMSerialEditEEPROMValue(T &_ptr, uint16_t _eeprom_address, T default_value, void (*_callback)(uint16_t rd, T *new_value) = nullptr) {
        // Serial.printf("EERPOM Serial Setting at: %d\n\r", _eeprom_address);
        this->eeprom_address = _eeprom_address;
        this->ptr = &_ptr;
        this->callback = _callback;

        this->serial_handler = new PAMSerialEditValue<T>(_ptr, nullptr);
        this->serial_handler_rd = PAMSerial.registerResponder(this->serial_handler);

        PAMEEPROM<T>::ReadStoredVar(this->eeprom_address, default_value, *this->ptr);
    }

    void becomesResponder(uint16_t rd, bool child_returned) {
        if (!child_returned) {
            // The user wants to edit the value, so send them to the serial_handler
            PAMSerial.pushResponder(this->serial_handler_rd);
        } else {
            // The user finished editting the value, so let's consume it and pop ourselves
            PAMEEPROM<T>::WriteStoredVar(this->eeprom_address, this->ptr);
            if (this->callback != nullptr) {
                (*this->callback)(rd, this->ptr);
            }
            PAMSerial.popResponder();
        }
    }

private:
    uint16_t eeprom_address;
    T *ptr = nullptr;
    void (*callback)(uint16_t rd, T *new_value) = nullptr;
    
    uint16_t serial_handler_rd = -1;
    PAMSerialEditValue<T> *serial_handler;
};

#endif // __PAM_SERIAL_EDITEEPROMVALUE_H__