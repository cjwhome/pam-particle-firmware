#include "CloudHandler.h"

CloudHandler::CloudHandler(CloudClass* particle_ptr)
{
    _particle = particle_ptr;
}

bool CloudHandler::toHex(char* dest, size_t dest_len, const uint8_t* values, size_t val_len) {
    if(dest_len < (val_len*2+1)) /* check that dest is large enough */
        return false;

    *dest = '\0'; /* in case val_len==0 */
    while(val_len--) {
        /* sprintf directly to where dest points */
        sprintf(dest, "%02X", *values);
        dest += 2;
        ++values;
    }
    return true;
}

bool CloudHandler::publish(SystemManifest &upload)
{
    uint8_t buffer[17649];
    char out[17649 * 2 + 1];
    String hexString = "";

    MY_pb_ostream_t stream = MY_pb_ostream_from_buffer(buffer, sizeof(buffer));
    MY_pb_encode(&stream, Upload_fields, &upload);

    // encode binary protobuf to base64 string
    String encoded = Base64::encodeToString(buffer, sizeof(buffer));

    toHex(out, sizeof(out), buffer, stream.bytes_written);
    for (int i = 0; i < sizeof(buffer); i++) {
        hexString += out[i];
    }

    Serial.println(hexString);
    Serial.println(hexString.length());

    bool published = _particle->publish("uploadCellular", hexString, PRIVATE);
    _particle->process();

    return published;
    return true;
}
