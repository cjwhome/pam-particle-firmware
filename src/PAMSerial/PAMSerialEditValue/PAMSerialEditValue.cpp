#include "PAMSerialEditValue.h"

// template<typename T>
// PAMSerialEditValue<T>::PAMSerialEditValue(T &_ptr): PAMSerialResponder(LINE)
// {
    
// }

// template<typename T>
// PAMSerialEditValue<T>::~PAMSerialEditValue() {}

// template<typename T>
// void PAMSerialEditValue<T>::becomesResponder(uint16_t rd, bool child_returned)
// {
//     PAMSerial.printf(rd, "PAMSerialEditValue - Enter a new value for something.\n\r");
// }

template<>
void PAMSerialEditValue<float>::becomesResponder(uint16_t rd, bool child_returned)
{
    PAMSerial.printf(rd, "Current Value: %f\n\r", *this->ptr);
    PAMSerial.println(rd, "Enter a new value ('x' to exit): ");
}

template<>
void PAMSerialEditValue<float>::onData(uint16_t rd, uint8_t *data, uint8_t length)
{
    if (*data == 'x') {
        PAMSerial.popResponder();
    } else {
        float temp = String((char *) data).toFloat();
        if (temp != 0 || (temp == 0 && data[0] == '0')) {
            PAMSerial.printf(rd, "Setting new value to: %f\n\r", temp);
            *(this->ptr) = temp;
            if (this->callback != nullptr) {
                this->callback(rd, this->ptr);
            }
            PAMSerial.popResponder();
        } else {
            PAMSerial.println(rd, "Could not read setting. Try again, or press 'x' to exit.");
        }
    }
}

template<>
void PAMSerialEditValue<bool>::becomesResponder(uint16_t rd, bool child_returned)
{
    PAMSerial.printf(rd, "Current value: ");
    if (*this->ptr == true) {
        PAMSerial.println(rd, "true");
    } else {
        PAMSerial.println(rd, "false");
    }
    PAMSerial.println(rd, "Pick a new value [true, false] ('x' to exit): ");
}

template<>
void PAMSerialEditValue<bool>::onData(uint16_t rd, uint8_t *data, uint8_t length)
{
    if (*data == 'x') {
        PAMSerial.popResponder();
    } else if (strstr((char *) data, "true") != nullptr) {
        PAMSerial.println(rd, "Setting the new value to 'true'");
        *(this->ptr) = true;
        PAMSerial.popResponder();
    } else if (strstr((char *) data, "false") != nullptr) {
        PAMSerial.println(rd, "Setting the new value to 'false'");
        *(this->ptr) = false;
        PAMSerial.popResponder();
    } else {
        PAMSerial.println(rd, "Could not read setting. Try again, or press 'x' to exit.");
    }
}

// template<typename T>
// void PAMSerialEditValue<T>::onData(uint16_t rd, uint8_t *data, uint8_t length)
// {
//     PAMSerial.println(rd, "[PAMSerialEditValue]::onData\nTrying to respond to unknown data type!");
// }