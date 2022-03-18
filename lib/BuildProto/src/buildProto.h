#ifndef _BUILD_PROTO_H_
#define _BUILD_PROTO_H_


#include "pam.pb.h"
#include "Particle.h"


class BuildProto
{
    private:
        bool NO2_enabled;
        bool ozone_enabled;
        String deviceName = "";

        SystemSettings settings;
    
    public:
        BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled);

        CalibrationParam buildSettingsCalibration(String name, CalParamType calParamTypes[2]);
        SystemSettings buildSystemSettings(int deviceSize);
        Device buildTopologyDevice(String SSID, String name, Units units);

        SystemTopology buildSystemTopology(int deviceSize);
        SystemManifest buildSystemManifest();


};


#endif // _BUILD_PROTO_H_