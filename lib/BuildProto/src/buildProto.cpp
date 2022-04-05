#include "buildProto.h"

BuildProto::BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled)
{
    DynamicJsonDocument doc(7000);

// This prints:
// {"sensor":"gps","time":1351824120,"data":[48.756080,2.302038]}
    Serial.println("Starting initialization");
    deviceName = "PAM-"+String(DEVICE_id);
    ozone_enabled = ozoneEnabled;
    NO2_enabled = NO2Enabled;
    deviceName.toCharArray(manifest.sid, deviceName.length()+1);
    doc["manifest"]["sid"] = manifest.sid;
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

    buildSystemTopology(deviceSize, manifest);

    buildSystemSettings(deviceSize, manifest);
    // manifest.settings = buildSystemSettings(deviceSize);

    // Serial.println("All done");
    manifest.terminating = false;
    Serial.println("Finsihed terminating");
    manifest.composition = Multi;
    Serial.println("Finsihed composition");
    String placeHolder = "";
}

CalibrationParam BuildProto::buildSettingsCalibration(String name, CalParamType calParamTypes[2])
{
    CalibrationParam calibration;
    name.toCharArray(calibration.name, name.length()+1);
    calibration.parameters[0] = calParamTypes[0];
    calibration.parameters[1] = calParamTypes[1];

    return calibration;
}

void BuildProto::buildSystemSettings(int deviceSize, SystemManifest& manifest)
{
    deviceName.toCharArray(manifest.settings.name, deviceName.length()+1);
    deviceName.toCharArray(manifest.settings.ssid, deviceName.length()+1);
    manifest.settings.type = PAM;
    manifest.settings.primaryUploadFrequency = 60*2;
    manifest.settings.diagnosticUploadFrequency = 0;
    CalParamType paramType[2] = {CalParamType::Slope, CalParamType::Zero};
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
                paramType[0] = CalParamType::Voltage;
                paramType[1] = CalParamType::Other;
                manifest.settings.calParams[count] = buildSettingsCalibration("Battery", paramType);
                paramType[0] = CalParamType::Slope;
                paramType[1] = CalParamType::Zero;
                break;
            case 10:
                paramType[0] = CalParamType::Other;
                paramType[1] = CalParamType::Other;
                manifest.settings.calParams[count] = buildSettingsCalibration("Latitude", paramType);
                paramType[0] = CalParamType::Slope;
                paramType[1] = CalParamType::Zero;
                count++;
                break;
            case 11:
                paramType[0] = CalParamType::Other;
                paramType[1] = CalParamType::Other;
                manifest.settings.calParams[count] = buildSettingsCalibration("Longitude", paramType);
                paramType[0] = CalParamType::Slope;
                paramType[1] = CalParamType::Zero;
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
    Serial.println(manifest.settings.ssid);
    Serial.println(manifest.settings.type);
    Serial.println(manifest.settings.primaryUploadFrequency);
    Serial.println(manifest.settings.diagnosticUploadFrequency);
    Serial.println("Going to return the setttings");
    // return settings;
}

void BuildProto::buildTopologyDevice(String SSID, String name, Units units, int count)
{
    // doc["manifest"]["topology"]["devices"][count]["ssid"] = SSID; 
    // doc["manifest"]["topology"]["devices"][count]["name"] = name;
    // doc["manifest"]["topology"]["devices"][count]["units"] = Units::PPM;

    // Device device;
    // SSID.toCharArray(device.ssid, SSID.length()+1);
    // name.toCharArray(device.name, name.length()+1);
    // device.units = units;
    // return device;
}

void BuildProto::buildSystemTopology(int deviceSize, SystemManifest& manifest)
{
    int count = 0;
    for (int i = 0; i <  deviceSize; i++)
    {
        switch(i)
        {
            case 0:
                buildTopologyDevice(deviceName, "CO", Units::PPM, count);
                // doc["manifest"]["topology"]["devices"][count]["ssid"] = deviceName; 
                // doc["manifest"]["topology"]["devices"][count]["name"] = "CO";
                // doc["manifest"]["topology"]["devices"][count]["units"] = Units::PPM;

            // Device device;
            // SSID.toCharArray(device.ssid, SSID.length()+1);
            // name.toCharArray(device.name, name.length()+1);
            // device.units = units;
            // return device;
            //     manifest.topology.devices[count] = buildTopologyDevice(deviceName, "CO", Units::PPM);
                count++;
                break;
            case 1:
                // doc["manifest"]["topology"]["devices"][count]["ssid"] = deviceName; 
                // doc["manifest"]["topology"]["devices"][count]["name"] = "CO2";
                // doc["manifest"]["topology"]["devices"][count]["units"] = Units::PPM;

                buildTopologyDevice(deviceName, "CO2", Units::PPM, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "CO2", Units::PPM);
                count++;
                break;
            case 2:
            if (NO2_enabled == true)
            {
                buildTopologyDevice(deviceName, "NO2", Units::PPM, count);
                //manifest.topology.devices[count] = buildTopologyDevice(deviceName, "NO2", Units::PPM);
                count++;
                break;
            }
            case 3:
                buildTopologyDevice(deviceName, "PM1", Units::PPM, count);
                //manifest.topology.devices[count] = buildTopologyDevice(deviceName, "PM1", Units::PPM);
                count++;
                break;
            case 4:
                buildTopologyDevice(deviceName, "PM2.5", Units::PPM, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "PM2.5", Units::PPM);
                count++;
                break;
            case 5:
                buildTopologyDevice(deviceName, "PM10", Units::PPM, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "PM10", Units::PPM);
                count++;
                break;
            case 6:
                buildTopologyDevice(deviceName, "Temperature", Units::OTHER, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Temperature", Units::OTHER);
                count++;
                break;
            case 7:
                buildTopologyDevice(deviceName, "Pressure", Units::OTHER, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Pressure", Units::OTHER);
                count++;
                break;
            case 8:
                buildTopologyDevice(deviceName, "Humidity", Units::OTHER, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Relative Humidity", Units::OTHER);
                count++;
                break;
            case 9: 
                buildTopologyDevice(deviceName, "Battery", Units::OTHER, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Battery", Units::OTHER);
                break;
            case 10:
                buildTopologyDevice(deviceName, "Latitude", Units::OTHER, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Latitude", Units::OTHER);
                count++;
                break;
            case 11:
                buildTopologyDevice(deviceName, "Longitude", Units::OTHER, count);
                // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Longitude", Units::OTHER);
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    buildTopologyDevice(deviceName, "Ozone", Units::PPB, count);
                    // manifest.topology.devices[count] = buildTopologyDevice(deviceName, "Ozone", Units::PPB);
                    count++;
                }
                break;
            default:
                break;
        }
    }
}

// build SystemManifest object for protobuf
SystemManifest BuildProto::buildSystemManifest()
{
    return manifest;
}

