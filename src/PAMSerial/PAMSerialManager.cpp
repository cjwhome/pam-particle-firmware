#include "PAMSerialManager.h"

PAMSerialManager::PAMSerialManager(Stream &_serial)
{
    this->serial = &_serial;
    this->clear_data();
}

PAMSerialManager::~PAMSerialManager() {}

uint16_t PAMSerialManager::registerResponder(PAMSerialResponder *responder)
{
    // return this->registerResponder(responder->toStruct());
    this->serial->printf("[PAMSerialManager]::registerResponder\trd: %d\n\r", this->responders.size());
    this->responders.push_back(responder);
    uint16_t rd = this->responders.size() - 1;
    return this->responders.size() - 1;
}

// uint16_t PAMSerialManager::registerResponder(pam_serial_responder_t *responder)
// {
//     this->responders.push_back(responder);
//     return this->responders.size() - 1;
// }

void PAMSerialManager::pushResponder(uint16_t responder)
{
    Serial.printf("[PAMSerialManager]::pushResponder(%d)\n\r", responder);
    if (responder < this->responders.size()) {
        this->active_responders.push(responder);
        this->clear_data();
        uint16_t rd = this->active_responders.top();
        this->responders[this->active_responders.top()]->becomesResponder(rd, false);
    }
}

void PAMSerialManager::popResponder()
{
    Serial.println("[PAMSerialManager]::popResponder");
    if (this->active_responders.size() > 1) {
        this->active_responders.pop();
        this->clear_data();
        uint16_t rd = this->active_responders.top();
        this->responders[rd]->becomesResponder(rd, true);
    }
}

void PAMSerialManager::clear_data()
{
    memset(this->data, 0, SERIAL_BUFF_LENGTH);
    this->data_index = 0;
}

void PAMSerialManager::loop()
{
    if (this->active_responders.size() == 0) return;

    PAMSerialResponder *responder = this->responders[this->active_responders.top()];
    uint16_t rd = this->active_responders.size() - 1;
    while (this->serial->available() > 0) {
        uint8_t new_byte = this->serial->read();
        if (responder->getReadType() == PAMSerialResponder::BYTE) {
            PAMSerial.printf(rd, "onData::byte(%d,,)\n\r", rd);
            responder->onData(rd, &new_byte, 1);
        } else if (responder->getReadType() == PAMSerialResponder::LINE) {
            if (new_byte == '\r') {
                // User has finished typing a line, so send it to the responder
                PAMSerial.printf(rd, "onData::line(%d,,)\n\r", rd);
                this->data[this->data_index++] =  '\0';
                responder->onData(rd, this->data, this->data_index);
                this->clear_data();
            } else {
                this->data[this->data_index++] = new_byte;
            }
        }
    }
}