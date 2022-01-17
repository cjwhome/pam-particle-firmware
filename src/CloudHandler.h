#pragma once
#include <Base64RK.h>
#include "MY_pb.h"
#include "MY_pb_common.h"
#include "MY_pb_encode.h"
#include "pam_upload.pb.h"

class CloudHandler
{
private:
    CloudClass* _particle;
    void process();
    static bool toHex(char* dest, size_t dest_len, const uint8_t* values, size_t val_len);
public:
    CloudHandler(CloudClass* particle_ptr);
    bool publish(Upload &upload);

    static GPS_V buildGPS(double latitude, double longitude, double acc);
    static const uint8_t MAX_BUF = GPS_V_size + Upload_size;
};
