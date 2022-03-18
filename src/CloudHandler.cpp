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
    // Serial.println("At beginning of publish: ");
    // uint8_t buffer[7914];
    // Serial.println("Made the buffer");
    // char out[7914 * 2 + 1];
    Serial.println("At beginning of publish: ");
    uint8_t buffer[4];
    Serial.println("Made the buffer");
    char out[4 * 2 + 1];
    Serial.println("Finsihed making the out");
    String hexString = "";
    Serial.println("About to do the ostream");
    MY_pb_ostream_t stream = MY_pb_ostream_from_buffer(buffer, sizeof(buffer));
    Serial.println("About to do the encoding");
    MY_pb_encode(&stream, Upload_fields, &upload);
    Serial.println("About to encode to string");

    // encode binary protobuf to base64 string
    String encoded = Base64::encodeToString(buffer, sizeof(buffer));
    Serial.println("About toHex");
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