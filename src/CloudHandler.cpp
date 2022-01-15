#include "CloudHandler.h"

CloudHandler::CloudHandler(CloudClass* particle_ptr)
{
    _particle = particle_ptr;
}

bool CloudHandler::publish(Upload &upload)
{
    uint8_t buffer[MAX_BUF];
    MY_pb_ostream_t stream = MY_pb_ostream_from_buffer(buffer, sizeof(buffer));
    MY_pb_encode(&stream, Upload_fields, &upload);

    // encode binary protobuf to base64 string
    String encoded = Base64::encodeToString(buffer, sizeof(buffer));

    _particle->publish("uploadCellular", encoded, PRIVATE);
    _particle->process();
}

GPS_V CloudHandler::buildGPS(double latitude, double longitude, double acc)
{
    GPS_V gpsData = GPS_V_init_zero;
    gpsData.lat = latitude;
    gpsData.long_ = longitude;
    gpsData.acc = acc;

    return gpsData;
}