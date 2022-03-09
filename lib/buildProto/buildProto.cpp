#include "buildProto.h"

BuildProto::BuildProto()
{
    
}

CalParams BuildProto::buildSettingsCalibrationParams(String name)
{

}

Calibration BuildProto::buildSettingsCalibration(String name, CalParams * params, int sizeOfParam);
{
    Calibration * calibration;
    calibration.name = name;
    calibration.parameters = params;

    return calibration;
}

SystemSettings BuildProto::buildSystemSettings(int deviceSize)
{
    SystemSettings settings;
    String deviceId = "PAM-"+String(DEVICE_id);
    deviceId.toCharArray(settings->SID, deviceId.length());
    String SerialNumber = coreId;
    coreId.toCharArray(settings->SerialNumber, SerialNumber.length());
    settings.type = SystemType_PAM;
    settings.primaryUploadFrequeny = measurements_to_average*2;
    settings.diagnosticUploadFrequeny = 0;
    Calibration * calibration[deviceSize];
    for (int i = 0; i < deviceSize; i++)
    {
        switch(i)
        {
            case 0:
                calibration[count] = buildTopologyDevice(deviceName, "CO", Units_PPM);
                count++;
                break;
            case 1:
                calibration[count] = buildTopologyDevice(deviceName, "CO2", Units_PPM);
                count++;
                break;
            case 2:
            if (NO2_enabled == true)
            {
                calibration[count] = buildTopologyDevice(deviceName, "NO2", Units_PPM);
                count++;
                break;
            }
            case 3:
                calibration[count] = buildTopologyDevice(deviceName, "PM1", Units_PPM);
                count++;
                break;
            case 4:
                calibration[count] = buildTopologyDevice(deviceName, "PM25", Units_PPM);
                count++;
                break;
            case 5:
                calibration[count] = buildTopologyDevice(deviceName, "PM10", Units_PPM);
                count++;
                break;
            case 6:
                calibration[count] = buildTopologyDevice(deviceName, "Temperature", Units_OTHER);
                count++;
                break;
            case 7:
                calibration[count] = buildTopologyDevice(deviceName, "Pressure", Units_OTHER);
                count++;
                break;
            case 8:
                calibration[count] = buildTopologyDevice(deviceName, "Relative Humidity", Units_OTHER);
                count++;
                break;
            case 9: 
                calibration[count] = buildTopologyDevice(deviceName, "Battery", Units_OTHER);
                break;
            case 10:
                calibration[count] = buildTopologyDevice(deviceName, "Latitude", Units_OTHER);
                count++;
                break;
            case 11:
                calibration[count] = buildTopologyDevice(deviceName, "Longitude", Units_OTHER);
                count++;
                break;
            case 12: 
                calibration[count] = buildTopologyDevice(deviceName, "Ozone", Units_PPB);
                count++;
                break;

            default:
            Serial.println("Did not match an i in loop for building Manifest");
            break;
        }
    }
    // settings.calibration = 
    return settings;
}

Device BuildProto::buildTopologyDevice(String SSID, String name, Units units)
{
    Device device;
    SSID.toCharArray(device->SSID, SSID.length());
    name.toCharArray(device->name, name.length());
    device->units = units;
    return device;
}

SystemTopology BuildProto::buildSystemTopology(int deviceSize)
{
    SystemTopology topology;
    int count = 0;
    Device * nameDevice[deviceSize];
    String deviceName = "PAM-"+String(DEVICE_id);
    for (int i = 0; i <  deviceSize; i++)
    {
        switch(i)
        {
            case 0:
                nameDevice[count] = buildTopologyDevice(deviceName, "CO", Units_PPM);
                count++;
                break;
            case 1:
                nameDevice[count] = buildTopologyDevice(deviceName, "CO2", Units_PPM);
                count++;
                break;
            case 2:
            if (NO2_enabled == true)
            {
                nameDevice[count] = buildTopologyDevice(deviceName, "NO2", Units_PPM);
                count++;
                break;
            }
            case 3:
                nameDevice[count] = buildTopologyDevice(deviceName, "PM1", Units_PPM);
                count++;
                break;
            case 4:
                nameDevice[count] = buildTopologyDevice(deviceName, "PM25", Units_PPM);
                count++;
                break;
            case 5:
                nameDevice[count] = buildTopologyDevice(deviceName, "PM10", Units_PPM);
                count++;
                break;
            case 6:
                nameDevice[count] = buildTopologyDevice(deviceName, "Temperature", Units_OTHER);
                count++;
                break;
            case 7:
                nameDevice[count] = buildTopologyDevice(deviceName, "Pressure", Units_OTHER);
                count++;
                break;
            case 8:
                nameDevice[count] = buildTopologyDevice(deviceName, "Relative Humidity", Units_OTHER);
                count++;
                break;
            case 9: 
                nameDevice[count] = buildTopologyDevice(deviceName, "Battery", Units_OTHER);
                break;
            case 10:
                nameDevice[count] = buildTopologyDevice(deviceName, "Latitude", Units_OTHER);
                count++;
                break;
            case 11:
                nameDevice[count] = buildTopologyDevice(deviceName, "Longitude", Units_OTHER);
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    nameDevice[count] = buildTopologyDevice(deviceName, "Ozone", Units_PPB);
                    count++;
                }
                break;
            default:
                Serial.println("Did not match an i in loop for building Manifest");
                break;
        }
    }
    topology.devices_count = deviceSize;
    topology.devices = nameDevice;
    topology.diagnostics_count = 0;
    topology.diagnostics = NULL;
    topology.subSystems_count = 0;
    topology.subSystems = NULL;
    return topology;
}

// build SystemManifest object for protobuf
SystemManifest BuildProto::buildSystemManifest()
{
    SystemManifest manifest;

    SystemSettings settings;
    // Composistion composition;

    String placeHolder = "PAM-"+String(DEVICE_id);
    placeHolder.toCharArray(manifest.SID, placeHolder.length());
    Serial.println("This is the manifest SID: ");
    Serial.println(manifest.SID);

    int deviceSize = 0;
    if (ozone_enabled == true)
    {
    deviceSize++;
    }
    if (NO2_enabled == true)
    {
    deviceSize++;
    }
    deviceSize += 11;

    manifest.has_topology = true;
    manifest.topology = buildSystemTopology(deviceSize);

    manifest.has_settings = true;
    manifest.settings = buildSystemSettings(deviceSize);


    //   cloud.publish(topology);
    Serial.println("after cloud publish");

    return manifest;
}

