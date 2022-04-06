#include "CloudHandler.h"

CloudHandler::CloudHandler(CloudClass* particle_ptr)
{
    _particle = particle_ptr;
}

bool CloudHandler::toHex(char* dest, size_t dest_len, const char* values, size_t val_len) {
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

bool CloudHandler::publish(String jsonManifest)
{
    // Serial.println("Inside publish");
    // char buffer[3];
    // char out[2000 * 2 + 1];
    // String hexString = "";
    // delay(100);
    // String hexString = "";
    // Serial.println("About to do the encoding");
    //     delay(100);

    // encode binary protobuf to base64 string

    // toHex(out, sizeof(out), jsonManifest, sizeof(jsonManifest));
    
    // for (int i = 0; i < sizeof(jsonManifest); i++) {
    //     sprintf(buffer, "%02x", (int)jsonManifest[i]);
    //     Serial.println(buffer);
    //     hexString += buffer;
    // }
    // Serial.println(sizeof(jsonManifest));
    // Serial.println(hexString.length());
    Serial.println(jsonManifest);
    Particle.publish("TestingMsgPack", jsonManifest, PRIVATE);
    // free(jsonManifest);


    

    // // MY_pb_encode(&stream, SystemTopology_fields, &manifest.topology);
    // Serial.print("Bytes written: ");
    // Serial.println(stream.bytes_written);
    // Serial.println("About to encode to string");
    //     delay(100);
    // Serial.print("Bytes written: ");
    // Serial.println(stream.bytes_written);
    // // encode binary protobuf to base64 string
    // String encoded = Base64::encodeToString(buffer, sizeof(buffer));
    // Serial.println("About toHex");
    // toHex(out, sizeof(out), buffer, stream.bytes_written);
    // for (int i = 0; i < sizeof(buffer); i++) 
    // {
    //     hexString += out[i];
    // }

    // Serial.println(hexString);
    // Serial.println(hexString.length());

    // bool published = _particle->publish("uploadCellular", hexString, PRIVATE);
    // _particle->process();

    // if (savedIt == false)
    // {
    //     if (sd.begin(CS)){
    //         file.open("HexOfProto", O_CREAT | O_APPEND | O_WRITE);
    //         file.println(hexString);
    //         file.close();
    //     }
    //     savedIt = true;
    // }

    return true;
}
