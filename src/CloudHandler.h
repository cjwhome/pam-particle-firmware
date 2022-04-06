#pragma once
#include <Base64RK.h>
#include "manifest.h"

#include "SdFat.h"
#define CS A2               //Chip select for SPI/uSD card

class CloudHandler
{
private:
    CloudClass* _particle;
    void process();
    static bool toHex(char* dest, size_t dest_len, const char* values, size_t val_len);
    uint8_t buffer[7914];
    char out[7914 * 2 + 1];

    bool savedIt = false;
    SdFat sd;
    SdFile file;

public:
    CloudHandler(CloudClass* particle_ptr);
    bool publish(String jsonManifest);

};
