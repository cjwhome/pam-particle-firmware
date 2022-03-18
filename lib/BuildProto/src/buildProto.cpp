#include "buildProto.h"

BuildProto::BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled)
{
    deviceName = "PAM-"+String(DEVICE_id);
    ozone_enabled = ozoneEnabled;
    NO2_enabled = NO2Enabled;
}

CalibrationParam BuildProto::buildSettingsCalibration(String name, CalParamType calParamTypes[2])
{
    Serial.println("immediately in buildSettingsCalibration");
    CalibrationParam calibration = CalibrationParam_init_zero;
    Serial.println("after calParam init");
    name.toCharArray(calibration.name, name.length());
    Serial.println("Set the name");
    calibration.parameter_count = 2;
    Serial.println("Doing the first param copy");
    calibration.parameter[0] = calParamTypes[0];
    Serial.println("Doing the second param copy");
    calibration.parameter[1] = calParamTypes[1];
    Serial.println("At the end of Param copy");

    return calibration;
}

SystemSettings BuildProto::buildSystemSettings(int deviceSize)
{
    Serial.printf("Settings pointer: %p\r\n", settings);
    Serial.printf("This: %p\r\n", this);
    deviceName.toCharArray(settings.name, deviceName.length());
    deviceName.toCharArray(settings.serial, deviceName.length());
    settings.type = SystemType_PAM;
    settings.primaryUploadFrequeny = 60*2;
    settings.diagnosticUploadFrequeny = 0;
    settings.calParams_count = deviceSize;
    CalParamType paramType[2] = {CalParamType_Slope, CalParamType_Zero};
    int count = 0;
    Serial.println("About to do the for loop");
    for (int i = 0; i < deviceSize; i++)
    {
        Serial.print("Inside for loop. i: ");
        Serial.println(i);
        Serial.print("the count: ");
        Serial.println(count);
        switch(i)
        {
            case 0:
            {
                Serial.println("About to make calibration");
                CalibrationParam calibration = CalibrationParam_init_zero;
                Serial.println("After making calibration");
                String name = "CO2";
                Serial.println("Before toChar arraying the name");
                name.toCharArray(calibration.name, name.length());
                Serial.println("After settings cal name");
                calibration.parameter_count = 2;
                Serial.println("Before first param set");
                calibration.parameter[0] = paramType[0];
                Serial.println("Before second param set");
                calibration.parameter[1] = paramType[1];
                Serial.println("Going to add this to settings");
                settings.calParams[count] = calibration;
                Serial.println("Made it to end of adding calibration");
                count++;
                break;
            }

            case 1:
                settings.calParams[count] = buildSettingsCalibration("CO2", paramType);
                count++;
                break;
            case 2:
                if (NO2_enabled == true)
                {
                    settings.calParams[count] = buildSettingsCalibration("NO2", paramType);
                    count++;
                    break;
                }
            case 3:
                settings.calParams[count] = buildSettingsCalibration("PM1", paramType);
                count++;
                break;
            case 4:
                settings.calParams[count] = buildSettingsCalibration("PM2.5", paramType);
                count++;
                break;
            case 5:
                settings.calParams[count] = buildSettingsCalibration("PM10", paramType);
                count++;
                break;
            case 6:
                settings.calParams[count] = buildSettingsCalibration("Temperature", paramType);
                count++;
                break;
            case 7:
                settings.calParams[count] = buildSettingsCalibration("Pressure", paramType);
                count++;
                break;
            case 8:
                settings.calParams[count] = buildSettingsCalibration("Relative Humidity", paramType);
                count++;
                break;
            case 9: 
                paramType[0] = CalParamType_Voltage;
                paramType[1] = CalParamType_Other;
                settings.calParams[count] = buildSettingsCalibration("Battery", paramType);
                paramType[0] = CalParamType_Slope;
                paramType[1] = CalParamType_Zero;
                break;
            case 10:
                paramType[0] = CalParamType_Other;
                paramType[1] = CalParamType_Other;
                settings.calParams[count] = buildSettingsCalibration("Latitude", paramType);
                paramType[0] = CalParamType_Slope;
                paramType[1] = CalParamType_Zero;
                count++;
                break;
            case 11:
                paramType[0] = CalParamType_Other;
                paramType[1] = CalParamType_Other;
                settings.calParams[count] = buildSettingsCalibration("Longitude", paramType);
                paramType[0] = CalParamType_Slope;
                paramType[1] = CalParamType_Zero;
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    settings.calParams[count] = buildSettingsCalibration("Ozone", paramType);
                    count++;
                }
                break;
            default:
                break;
        }
    }
    // settings.calibration = 
    return settings;
}

Device BuildProto::buildTopologyDevice(String SSID, String name, Units units)
{
    Device device;
    SSID.toCharArray(device.SSID, SSID.length());
    name.toCharArray(device.name, name.length());
    device.units = units;
    return device;
}

SystemTopology BuildProto::buildSystemTopology(int deviceSize)
{
    SystemTopology topology = SystemTopology_init_zero;
    int count = 0;
    for (int i = 0; i <  deviceSize; i++)
    {
        switch(i)
        {
            case 0:
                topology.devices[count] = buildTopologyDevice(deviceName, "CO", Units_PPM);
                count++;
                break;
            case 1:
                topology.devices[count] = buildTopologyDevice(deviceName, "CO2", Units_PPM);
                count++;
                break;
            case 2:
            if (NO2_enabled == true)
            {
                topology.devices[count] = buildTopologyDevice(deviceName, "NO2", Units_PPM);
                count++;
                break;
            }
            case 3:
                topology.devices[count] = buildTopologyDevice(deviceName, "PM1", Units_PPM);
                count++;
                break;
            case 4:
                topology.devices[count] = buildTopologyDevice(deviceName, "PM2.5", Units_PPM);
                count++;
                break;
            case 5:
                topology.devices[count] = buildTopologyDevice(deviceName, "PM10", Units_PPM);
                count++;
                break;
            case 6:
                topology.devices[count] = buildTopologyDevice(deviceName, "Temperature", Units_OTHER);
                count++;
                break;
            case 7:
                topology.devices[count] = buildTopologyDevice(deviceName, "Pressure", Units_OTHER);
                count++;
                break;
            case 8:
                topology.devices[count] = buildTopologyDevice(deviceName, "Relative Humidity", Units_OTHER);
                count++;
                break;
            case 9: 
                topology.devices[count] = buildTopologyDevice(deviceName, "Battery", Units_OTHER);
                break;
            case 10:
                topology.devices[count] = buildTopologyDevice(deviceName, "Latitude", Units_OTHER);
                count++;
                break;
            case 11:
                topology.devices[count] = buildTopologyDevice(deviceName, "Longitude", Units_OTHER);
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    topology.devices[count] = buildTopologyDevice(deviceName, "Ozone", Units_PPB);
                    count++;
                }
                break;
            default:
                break;
        }
    }
    topology.devices_count = deviceSize;
    topology.diagnostics_count = 0;
    topology.subSystems_count = 0;
    return topology;
}

// build SystemManifest object for protobuf
SystemManifest BuildProto::buildSystemManifest()
{
    SystemManifest manifest;

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

    Serial.println("Doing topology");
    manifest.has_topology = true;
    manifest.topology = buildSystemTopology(deviceSize);

    Serial.println("Doing System Settings");
    manifest.has_settings = true;
    manifest.settings = buildSystemSettings(deviceSize);

    Serial.println("All done");
    manifest.terminating = false;
    manifest.composition = Composition_MULTI_PART;

    return manifest;
}

