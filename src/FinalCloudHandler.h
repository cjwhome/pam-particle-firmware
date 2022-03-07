#pragma once
#include <Base64RK.h>
#include "MY_pb.h"
#include "MY_pb_common.h"
#include "MY_pb_encode.h"
#include "pam_upload.pb.h"

class FinalCloudHandler
{
private:
    CloudClass* _particle;
    void process();
    static bool toHex(char* dest, size_t dest_len, const uint8_t* values, size_t val_len);
public:
    FinalCloudHandler(CloudClass* particle_ptr);
    // bool publish(Upload &upload);

};
