#ifndef _BUILD_PROTO_H_
#define _BUILD_PROTO_H_

#include "manifest.h"
#include "Particle.h"
#include "../../ArduinoJson/src/ArduinoJson.h"

class BuildProto
{
    private:
        bool NO2_enabled;
        bool ozone_enabled;
        char * deviceName;

        DynamicJsonDocument doc;
    
    public:
        BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled);

        void buildSettingsCalibration(char * name, CalParamType calParamTypes[2], int count);
        void buildSystemSettings(int deviceSize);
        void buildTopologyDevice(char * SSID, char * name, Units units, int count);

        void buildSystemTopology(int deviceSize);
        String buildSystemManifest();

};


#endif // _BUILD_PROTO_H_