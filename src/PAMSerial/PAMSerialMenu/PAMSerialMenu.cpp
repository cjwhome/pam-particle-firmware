#include "PAMSerialMenu.h"

PAMSerialMenu::PAMSerialMenu(): PAMSerialResponder(LINE) {}
PAMSerialMenu::~PAMSerialMenu() {}

uint16_t PAMSerialMenu::addResponder(PAMSerialResponder *responder, char *name)
{
    uint8_t rd = PAMSerial.registerResponder(responder);
    this->addResponder(rd, name);
    return rd;
}

void PAMSerialMenu::addResponder(uint16_t responder, char *name) {
    serial_menu_entry_t *entry = new serial_menu_entry_t();
    entry->rd = responder;
    entry->name = name;
    this->entries.push_back(entry);
}

void PAMSerialMenu::becomesResponder(uint16_t rd, bool child_returned)
{
    PAMSerial.printf(rd, "PAMSerialMenu\n\r");
    for (size_t i = 0; i < this->entries.size(); i++)
    {
        PAMSerial.printf(rd, "%d\t%s\n\r", i, this->entries[i]->name);
    }
    PAMSerial.printf(rd, "PAM> ");
}

void PAMSerialMenu::onData(uint16_t rd, uint8_t *data, uint8_t length) 
{
    if (length > 0 && *data == 'x') {
        PAMSerial.popResponder();
    } else {
        size_t index = 0;
        if (sscanf((char *) data, "%d\n", &index) == 1) {
            if (index < this->entries.size()) {
                PAMSerial.pushResponder(this->entries[index]->rd);
            } else {
                PAMSerial.printf(rd, "The selection is out of range. Please enter a different option.\n\r");
            }
        } else {
            PAMSerial.printf(rd, "[PAMSerialMenu]::%d Could not read the input. Please try again.\n\r", rd);
        }
    }
}
