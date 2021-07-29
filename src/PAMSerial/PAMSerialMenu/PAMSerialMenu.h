/**
 * PAMSerialMenu.h - Created 04/19/2020
 * Author: David Kopala
 */

#ifndef __PAMSERIALMENU_H__
#define __PAMSERIALMENU_H__

#include "Particle.h"
#include <vector>

#include "../PAMSerialResponder.h"
#include "../PAMSerial.h"
#include "../../global.h"

class PAMSerialMenu: public PAMSerialResponder {

public:
    PAMSerialMenu();
    ~PAMSerialMenu();

    void becomesResponder(uint16_t rd, bool child_returned);
    void onData(uint16_t rd, uint8_t *data, uint8_t length);

    uint16_t addResponder(PAMSerialResponder *responder, char *name);
    void addResponder(uint16_t responder, char *name);

private:
    typedef struct {
        char *name;
        uint8_t rd;
    } serial_menu_entry_t;
    std::vector<serial_menu_entry_t *> entries;
};

#endif // __PAMSERIALMENU_H__