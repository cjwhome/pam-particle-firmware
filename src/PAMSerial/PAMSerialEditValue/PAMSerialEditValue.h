/**
 * PAMSerialEditValue.h - Created 04/19/2020
 * Author: David Kopala
 */

#ifndef __PAMSERIAL_EDITVALUE_H__
#define __PAMSERIAL_EDITVALUE_H__

#include "../PAMSerial.h"
#include "../PAMSerialResponder.h"

template<typename T>
class PAMSerialEditValue: public PAMSerialResponder {

public:
    PAMSerialEditValue(T &_ptr, void (*_callback)(uint16_t rd, T *new_value) = nullptr) { 
        this->ptr = &_ptr;
        this->callback = _callback;
    };
    ~PAMSerialEditValue();

    void becomesResponder(uint16_t rd, bool child_returned) {
        PAMSerial.printf(rd, "PAMSerialEditValue - Enter a new value for something.\n\r");
    }

    void onData(uint16_t rd, uint8_t *data, uint8_t length) {
        PAMSerial.println(rd, "[PAMSerialEditValue]::onData\nTrying to respond to unknown data type!");
    }

    void finish() {}

private:
    T *ptr;
    void (*callback)(uint16_t rd, T *new_value);

};

#endif