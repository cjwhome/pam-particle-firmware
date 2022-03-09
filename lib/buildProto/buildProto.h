#pragma once
#include "pam.pb.h"

class BuildProto
{
private:
    
public:
    BuildProto();
    Device buildTopologyDevice(String SSID, String name, Units units);
    SystemTopology buildSystemTopology();
    SystemManifest buildSystemManifest();
    Device buildManyTopologyDevices();


};
