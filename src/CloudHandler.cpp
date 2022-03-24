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

bool CloudHandler::publish(SystemManifest &manifest)
{
    Serial.println("Going to print some stuff out to maybe see some problems");
    Serial.println(manifest.settings.name);
    Serial.println(manifest.settings.serial);
    Serial.println(manifest.settings.type);
    Serial.println(manifest.settings.primaryUploadFrequeny);
    Serial.println(manifest.settings.diagnosticUploadFrequeny);
    Serial.println(manifest.settings.calParams_count);
    String hexString = "";
    MY_pb_ostream_t stream = MY_pb_ostream_from_buffer(buffer, sizeof(buffer));
    Serial.println("About to do the encoding");
    MY_pb_encode(&stream, Upload_fields, &manifest);
    Serial.println("About to encode to string");

    // encode binary protobuf to base64 string
    String encoded = Base64::encodeToString(buffer, sizeof(buffer));
    Serial.println("About toHex");
    toHex(out, sizeof(out), buffer, stream.bytes_written);
    for (int i = 0; i < sizeof(buffer); i++) 
    {
        hexString += out[i];
    }

    Serial.println(hexString);
    Serial.println(hexString.length());

    bool published = _particle->publish("uploadCellular", hexString, PRIVATE);
    _particle->process();

    if (savedIt == false)
    {
        if (sd.begin(CS)){
            file.open("HexOfProto", O_CREAT | O_APPEND | O_WRITE);
            file.println(hexString);
            file.close();
        }
        savedIt = true;
    }

    return published;
    return true;
}
