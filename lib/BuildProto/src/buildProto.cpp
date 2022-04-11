#include "buildProto.h"

BuildProto::BuildProto(int DEVICE_id, bool ozoneEnabled, bool NO2Enabled) : doc(7000)
{
    Serial.println("Starting initialization");
    String placeHolder = "PAM-"+String(DEVICE_id);
    placeHolder.toCharArray(deviceName, placeHolder.length()+1);
    ozone_enabled = ozoneEnabled;
    NO2_enabled = NO2Enabled;
    doc["manifest"]["sid"] = deviceName;
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

    buildSystemTopology(deviceSize);

    buildSystemSettings(deviceSize);

    // Serial.println("All done");
    doc["manifest"]["terminating"] = false;
    Serial.println("Finsihed terminating");
    doc["manifest"]["composition"] = (int)Composition::Multi;
    Serial.println("Finsihed composition");
}

void BuildProto::buildSettingsCalibration(char * name, CalParamType calParamTypes[2], int count)
{

    doc["manifest"]["settings"]["calParams"][count]["name"] = name;
    doc["manifest"]["settings"]["calParams"][count]["parameters"][0] = (int)calParamTypes[0];
    doc["manifest"]["settings"]["calParams"][count]["parameters"][1] = (int)calParamTypes[1];
}

void BuildProto::buildSystemSettings(int deviceSize)
{
    doc["manifest"]["settings"]["name"] = deviceName;
    doc["manifest"]["settings"]["ssid"] = deviceName;

    doc["manifest"]["settings"]["type"] = (int)SystemType::PAM;

    doc["manifest"]["settings"]["primaryUploadFrequency"] = 60*2;
    doc["manifest"]["settings"]["diagnosticUploadFrequency"] = 0;

    CalParamType paramType[2] = {CalParamType::Slope, CalParamType::Zero};
    int count = 0;
    for (int i = 0; i < deviceSize; i++)
    {
        switch(i)
        {
            case 0:
            {
                buildSettingsCalibration("CO", paramType, count);
                count++;
                break;
            }

            case 1:
                buildSettingsCalibration("CO2", paramType, count);
                count++;
                break;
            case 2:
                if (NO2_enabled == true)
                {
                    buildSettingsCalibration("NO2", paramType, count);
                    count++;
                    break;
                }
            case 3:
                buildSettingsCalibration("PM1", paramType, count);
                count++;
                break;
            case 4:
                buildSettingsCalibration("PM2.5", paramType, count);
                count++;
                break;
            case 5:
                buildSettingsCalibration("PM10", paramType, count);
                count++;
                break;
            case 6:
                buildSettingsCalibration("Temperature", paramType, count);
                count++;
                break;
            case 7:
                buildSettingsCalibration("Pressure", paramType, count);
                count++;
                break;
            case 8:
                buildSettingsCalibration("Relative Humidity", paramType, count);
                count++;
                break;
            case 9: 
                paramType[0] = CalParamType::Voltage;
                paramType[1] = CalParamType::Other;
                buildSettingsCalibration("Battery", paramType, count);
                paramType[0] = CalParamType::Slope;
                paramType[1] = CalParamType::Zero;
                count++;
                break;
            case 10:
                paramType[0] = CalParamType::Other;
                paramType[1] = CalParamType::Other;
                buildSettingsCalibration("Latitude", paramType, count);
                paramType[0] = CalParamType::Slope;
                paramType[1] = CalParamType::Zero;
                count++;
                break;
            case 11:
                paramType[0] = CalParamType::Other;
                paramType[1] = CalParamType::Other;
                buildSettingsCalibration("Longitude", paramType, count);
                paramType[0] = CalParamType::Slope;
                paramType[1] = CalParamType::Zero;
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    buildSettingsCalibration("Ozone", paramType, count);
                    count++;
                }
                break;
            default:
                break;
        }
    }
}

void BuildProto::buildTopologyDevice(char * SSID, char * name, Units units, int count)
{
    doc["manifest"]["topology"]["devices"][count]["ssid"] = SSID; 
    doc["manifest"]["topology"]["devices"][count]["name"] = name;
    doc["manifest"]["topology"]["devices"][count]["units"] = (int)units;
}

void BuildProto::buildSystemTopology(int deviceSize)
{
    int count = 0;
    for (int i = 0; i <  deviceSize; i++)
    {
        switch(i)
        {
            case 0:
                buildTopologyDevice(deviceName, "CO", Units::PPM, count);
                count++;
                break;
            case 1:
                buildTopologyDevice(deviceName, "CO2", Units::PPM, count);
                count++;
                break;
            case 2:
            if (NO2_enabled == true)
            {
                buildTopologyDevice(deviceName, "NO2", Units::PPM, count);
                count++;
                break;
            }
            case 3:
                buildTopologyDevice(deviceName, "PM1", Units::PPM, count);
                count++;
                break;
            case 4:
                buildTopologyDevice(deviceName, "PM2.5", Units::PPM, count);
                count++;
                break;
            case 5:
                buildTopologyDevice(deviceName, "PM10", Units::PPM, count);
                count++;
                break;
            case 6:
                buildTopologyDevice(deviceName, "Temperature", Units::OTHER, count);
                count++;
                break;
            case 7:
                buildTopologyDevice(deviceName, "Pressure", Units::OTHER, count);
                count++;
                break;
            case 8:
                buildTopologyDevice(deviceName, "Humidity", Units::OTHER, count);
                count++;
                break;
            case 9: 
                buildTopologyDevice(deviceName, "Battery", Units::OTHER, count);
                count++;
                break;
            case 10:
                buildTopologyDevice(deviceName, "Latitude", Units::OTHER, count);
                count++;
                break;
            case 11:
                buildTopologyDevice(deviceName, "Longitude", Units::OTHER, count);
                count++;
                break;
            case 12: 
                if (ozone_enabled)
                {
                    buildTopologyDevice(deviceName, "Ozone", Units::PPB, count);
                    count++;
                }
                break;
            default:
                break;
        }
    }
}

// build SystemManifest object for protobuf
String BuildProto::buildSystemManifest()
{
    char *output = (char*)malloc(7000);
    memset(output, 0, 7000*sizeof(char));
    String jsonString = "";
    int numOfMsgPacker = serializeJson(doc, output, 7000);
    Serial.println(numOfMsgPacker);
    Serial.printf("%s\n", output);
    Serial.println("");
    Serial.println("");
    numOfMsgPacker = serializeMsgPack(doc, output, 7000);
    // char * jsonPacked = (char*)malloc(numOfMsgPacker+1);
    Serial.println(numOfMsgPacker);
    char buf[3];
    for (int i = 0; i < numOfMsgPacker; i++)
    {
        sprintf(buf, "%02x", (int)output[i]);
        jsonString += buf;
        // jsonPacked[i] = output[i];
        //Serial.printf("%2x", (int)output[i]);
    }
    free(output);
    return jsonString;
}

