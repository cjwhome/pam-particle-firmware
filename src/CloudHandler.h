#pragma once
#include <Base64RK.h>
#include "MY_pb.h"
#include "MY_pb_common.h"
#include "MY_pb_encode.h"
#include "pam.pb.h"
#include "PI_pb_encode.h"

#include "SdFat.h"
#define CS A2               //Chip select for SPI/uSD card

class CloudHandler
{
private:
    CloudClass* _particle;
    void process();
    static bool toHex(char* dest, size_t dest_len, const uint8_t* values, size_t val_len);
    uint8_t buffer[7914];
    char out[7914 * 2 + 1];

    bool savedIt = false;
    SdFat sd;
    SdFile file;

public:
    CloudHandler(CloudClass* particle_ptr);
    bool publish(SystemManifest &upload);

};
