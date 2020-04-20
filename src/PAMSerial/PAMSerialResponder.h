/**
 * PAMSerialResponder.h - Created 04/19/2020
 * Author: David Kopala
 */

#ifndef __PAMSERIALRESPONDER_H__
#define __PAMSERIALRESPONDER_H__


class PAMSerialResponder {

public:
    enum ReadType { LINE = 1, BYTE = 2 };
    PAMSerialResponder(ReadType _readType = LINE) { this->readType = _readType; };
    ~PAMSerialResponder() {};

    virtual void becomesResponder(uint16_t rd, bool child_returned) {};
    virtual void onData(uint16_t rd, uint8_t *data, uint8_t length) {};

    ReadType getReadType() { return this->readType; };

private:
    ReadType readType;

};


#endif // __PAMSERIALRESPONDER_H__