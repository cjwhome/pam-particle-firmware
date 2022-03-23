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

        SystemManifest manifest = SystemManifest_init_zero;
        CalibrationParam calibration = CalibrationParam_init_zero;
    
    public:
        BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled);

        CalibrationParam buildSettingsCalibration(String name, CalParamType calParamTypes[2]);
        void buildSystemSettings(int deviceSize, SystemManifest& manifest);
        Device buildTopologyDevice(String SSID, String name, Units units);

        void buildSystemTopology(int deviceSize, SystemManifest& manifest);
        SystemManifest buildSystemManifest();


};


#endif // _BUILD_PROTO_H_