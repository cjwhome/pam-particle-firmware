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
        String deviceName = "";
    
    public:
        BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled);

        CalibrationParam buildSettingsCalibration(String name, CalParamType calParamTypes[2]);
        void buildSystemSettings(int deviceSize, SystemManifest& manifest);
        void buildTopologyDevice(String SSID, String name, Units units, int count);

        void buildSystemTopology(int deviceSize, SystemManifest& manifest);
        SystemManifest buildSystemManifest();

        SystemManifest manifest;
};


#endif // _BUILD_PROTO_H_