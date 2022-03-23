#include "buildProto.h"

BuildProto::BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled)
{
    Serial.println("Starting initialization");
    deviceName = "PAM-"+String(DEVICE_id);
    ozone_enabled = ozoneEnabled;
    NO2_enabled = NO2Enabled;

    deviceName.toCharArray(manifest.SID, deviceName.length());

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
    buildSystemTopology(deviceSize, manifest);

    manifest.has_settings = true;
    buildSystemSettings(deviceSize, manifest);
    // manifest.settings = buildSystemSettings(deviceSize);

    // Serial.println("All done");
    manifest.terminating = false;
    Serial.println("Finsihed terminating");
    manifest.composition = Composition_MULTI_PART;
    Serial.println("Finsihed composition");
}

CalibrationParam BuildProto::buildSettingsCalibration(String name, CalParamType calParamTypes[2])
{
    name.toCharArray(calibration.name, name.length()+1);
    calibration.parameter_count = 2;
    calibration.parameter[0] = calParamTypes[0];
    calibration.parameter[1] = calParamTypes[1];

    return calibration;
}

void BuildProto::buildSystemSettings(int deviceSize, SystemManifest& manifest)
{
    deviceName.toCharArray(manifest.settings.name, deviceName.length());
    deviceName.toCharArray(manifest.settings.serial, deviceName.length());
    manifest.settings.type = SystemType_PAM;
    manifest.settings.primaryUploadFrequeny = 60*2;
    manifest.settings.diagnosticUploadFrequeny = 0;
    manifest.settings.calParams_count = deviceSize;
    CalParamType paramType[2] = {CalParamType_Slope, CalParamType_Zero};
    int count = 0;
    for (int i = 0; i < deviceSize; i++)
    {
        Serial.print("Inisde for loop. The i: ");
        Serial.println(i);
        switch(i)
        {
            case 0:
            {
                manifest.settings.calParams[count] = buildSettingsCalibration("CO", paramType);
                count++;
                break;
            }

            case 1:
                manifest.settings.calParams[count] = buildSettingsCalibration("CO2", paramType);
                count++;
                break;
            case 2:
                if (NO2_enabled == true)
                {
                    manifest.settings.calParams[count] = buildSettingsCalibration("NO2", paramType);
                    count++;
                    break;
                }
            case 3:
                manifest.settings.calParams[count] = buildSettingsCalibration("PM1", paramType);
                count++;
                break;
            case 4:
                manifest.settings.calParams[count] = buildSettingsCalibration("PM2.5", paramType);
                count++;
                break;
            case 5:
                manifest.settings.calParams[count] = buildSettingsCalibration("PM10", paramType);
                count++;
                break;
            case 6:
                manifest.settings.calParams[count] = buildSettingsCalibration("Temperature", paramType);
                count++;
                break;
            case 7:
                manifest.settings.calParams[count] = buildSettingsCalibration("Pressure", paramType);
                count++;
                break;
            case 8:
                manifest.settings.calParams[count] = buildSettingsCalibration("Relative Humidity", paramType);
                count++;
                break;
            case 9: 
                paramType[0] = CalParamType_Voltage;
                paramType[1] = CalParamType_Other;
                manifest.settings.calParams[count] = buildSettingsCalibration("Battery", paramType);
                paramType[0] = CalParamType_Slope;
                paramType[1] = CalParamType_Zero;
                break;
            case 10:
                paramType[0] = CalParamType_Other;
                paramType[1] = CalParamType_Other;
                manifest.settings.calParams[count] = buildSettingsCalibration("Latitude", paramType);
                paramType[0] = CalParamType_Slope;
                paramType[1] = CalParamType_Zero;
                count++;
                break;
            case 11:
                paramType[0] = CalParamType_Other;
                paramType[1] = CalParamType_Other;
                manifest.settings.calParams[count] = buildSettingsCalibration("Longitude", paramType);
                paramType[0] = CalParamType_Slope;
                paramType[1] = CalParamType_Zero;
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    manifest.settings.calParams[count] = buildSettingsCalibration("Ozone", paramType);
                    count++;
                }
                break;
            default:
                break;
        }
    }
    Serial.println("Going to print some stuff out to maybe see some problems");
    Serial.println(manifest.settings.name);
    Serial.println(manifest.settings.serial);
    Serial.println(manifest.settings.type);
    Serial.println(manifest.settings.primaryUploadFrequeny);
    Serial.println(manifest.settings.diagnosticUploadFrequeny);
    Serial.println(manifest.settings.calParams_count);
    for (int i = 0; i < manifest.settings.calParams_count; i++)
    {
        Serial.println(manifest.settings.calParams[i].name);
    }
    Serial.println("Going to return the setttings");
    // return settings;
}

Device BuildProto::buildTopologyDevice(String SSID, String name, Units units)
{
    Device device;
    SSID.toCharArray(device.SSID, SSID.length());
    name.toCharArray(device.name, name.length());
    device.units = units;
    return device;
}

void BuildProto::buildSystemTopology(int deviceSize, SystemManifest& manifest)
{
    int count = 0;
    for (int i = 0; i <  deviceSize; i++)
    {
        switch(i)
        {
            case 0:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "CO", Units_PPM);
                count++;
                break;
            case 1:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "CO2", Units_PPM);
                count++;
                break;
            case 2:
            if (NO2_enabled == true)
            {
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "NO2", Units_PPM);
                count++;
                break;
            }
            case 3:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "PM1", Units_PPM);
                count++;
                break;
            case 4:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "PM2.5", Units_PPM);
                count++;
                break;
            case 5:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "PM10", Units_PPM);
                count++;
                break;
            case 6:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Temperature", Units_OTHER);
                count++;
                break;
            case 7:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Pressure", Units_OTHER);
                count++;
                break;
            case 8:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Relative Humidity", Units_OTHER);
                count++;
                break;
            case 9: 
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Battery", Units_OTHER);
                break;
            case 10:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Latitude", Units_OTHER);
                count++;
                break;
            case 11:
                manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Longitude", Units_OTHER);
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Ozone", Units_PPB);
                    count++;
                }
                break;
            default:
                break;
        }
    }
    manifest.topology.devices_count = deviceSize;
    manifest.topology.diagnostics_count = 0;
    manifest.topology.subSystems_count = 0;
}

// build SystemManifest object for protobuf
SystemManifest BuildProto::buildSystemManifest()
{
    return manifest;
}

