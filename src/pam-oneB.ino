
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Telaire_T6713.h"
#include "LMP91000.h"
#include "Serial4/Serial4.h"
#include "Serial5/Serial5.h"
#include "gps.h"
#include "inttypes.h"
#include "Particle.h"
#include "PowerCheck.h"
#include "SdFat.h"
#include "HIH61XX.h"
#include "google-maps-device-locator.h"
#include "CellularHelper.h"

#include "Wiring.h"
#include "Constants.h"

#include "PAMSerial/PAMSerial.h"
#include "PAMSerial/PAMSerialMenu/PAMSerialMenu.h"

#include "PAMSensorManager/PAMSensorManager.h"
#include "Sensors/T6713/T6713.h"
#include "Sensors/TPHFusion/TPHFusion.h"
#include "Sensors/Plantower/Plantower.h"
#include "Sensors/PAMCO/PAMCO.h"

GoogleMapsDeviceLocator locator;

#define APP_VERSION 7
#define BUILD_VERSION 8

//define constants
#define SEALEVELPRESSURE_HPA (1013.25)
#define LOW_PRESSURE_LIMIT (100)
#define HIGH_PRESSURE_LIMIT (1500)
#define VOLTS_PER_UNIT (0.0008)   //3.3V/4096  3.3 is the adc reference voltage and the adc is 12 bits or 4096
#define VOLTS_PER_PPB (0.0125)  //2.5V/200 ppb this is what you divide the voltage reading by to get ppb in ozone if the ozone monitor is set to 2.5V=200ppb

#define CELCIUS 1
#define FARENHEIT 0
#define TMP36_OFFSET 0.5
#define TMP36_VPDC 0.01 //10mV per degree C

//google maps API key:
#define GOOGLE_API_KEY "AIzaSyAfgY0VX3KSMkVoIVvWAr9oVlT-AoQ68e0"

//enable or disable different parts of the firmware by setting the following values to 1 or 0
#define sd_en 1

//enable AFE 2 (just for testing now using a second CO sensor connected to it)
#define AFE2_en 0

//define addresses of eeprom stored variables
#include "PAMEEPROM/PAMEEPROM.h"


//max and min values
#define MIN_DEVICE_ID_NUMBER 1
#define MAX_DEVICE_ID_NUMBER 9999

//#define MEASUREMENTS_TO_AVERAGE 1       //change to 30 for 3 minute uploads



//ble data output

#define BLE_PAYLOAD_SIZE 22     //number of bytes allowed in payload - this is sent to the ESP chip to be output in ble broadcast packets

//define constants for species - to do in future - read from json file!
#define DEVICE_ID_PACKET_CONSTANT 'Z'       //instrument ID number as INTEGER
#define VOC_PACKET_CONSTANT 'g'             //VOCs as IAQ
#define CARBON_MONOXIDE_PACKET_CONSTANT 'M' //CO as PPM
#define CARBON_DIOXIDE_PACKET_CONSTANT 'C'  //CO2 as PPM
#define PM1_PACKET_CONSTANT 'r'             //PM1 as UG/M3
#define PM2PT5_PACKET_CONSTANT 'R'          //PM2.5 as UG/M3
#define PM10_PACKET_CONSTANT 'q'            //PM10 as UG/M3
#define TEMPERATURE_PACKET_CONSTANT 't'     //temperature as DEGREES CELSIUS
#define TEMPERATURE_FAHRENHEIT_PACKET_CONSTANT 'f'
#define PRESSURE_PACKET_CONSTANT 'P'        //pressure as MILLIBARS
#define HUMIDITY_PACKET_CONSTANT 'h'        //humidity as PERCENTAGE
#define SOUND_PACKET_CONSTANT 's'           //sound as DECIBELS
#define LATITUDE_PACKET_CONSTANT 'a'        //Latitude as DEGREES
#define LONGITUDE_PACKET_CONSTANT 'o'       //Longitude as DEGREES
#define ACCURACY_PACKET_CONSTANT 'c'
#define PARTICLE_TIME_PACKET_CONSTANT 'Y'   //result of now()
#define OZONE_PACKET_CONSTANT 'O'           //Ozone
#define BATTERY_PACKET_CONSTANT 'x'         //Battery in percentage

#define HEADER_STRING "DEV,CO(ppm),CO2(ppm),VOCs(IAQ),PM1,PM2_5,PM10,T(C),Press(mBar),RH(%),O3(ppb),Batt(%),Snd(db),Latitude,Longitude,N/A,N/A,Date/Time"


#define NUMBER_OF_SPECIES 11    //total number of species (measurements) being output

#define MAX_COUNTER_INDEX 15000

#define BATTERY_THRESHOLD 20    //if battery is below 20 percent, go into sleep mode

//define pin functions
//fix these so they are more consistent!
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define CS A2               //Chip select for SPI/uSD card
#define SLEEP_EN D3

//constants for parsing ozone string from ozone monitor Model 106
#define NUMBER_OF_FEILDS 7
#define NUMBER_OF_FIELDS_LOGGING 7


//manually control connection to cellular network
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

//global objects

// PAM Sensors
T6713 t6713;
TPHFusion tph_fusion(0x27, false);
Plantower plantower(Serial4);
PAMCO pamco(ADS1115_1_ADDR, LMP91000_1_EN);


FuelGauge fuel;
GPS gps;
PMIC pmic;
PowerCheck powerCheck;
//SerialLogHandler logHandler;

unsigned long lastCheck = 0;
char lastStatus[256];

//sdcard
SdFat sd;
SdFile file;
SdFile log_file;
File file1;
String fileName;
String logFileName;
int file_started = 0;
int log_file_started = 0;

//wifi
String ssid; //wifi network name
String password; //wifi network password

//global variables
int counter = 0;
float CO2_float = 0;
float CO2_float_previous = 0;
int CO2_value = 0;
float O3_float = 0;
int DEVICE_id = 555;       //default value
int sample_counter = 0;
float tempfloat = 0;
int tempValue;
float air_quality_score = 0;
int esp_wifi_connection_status = 0;
int serial_cellular_enabled = 0;
int debugging_enabled = 0;
int ozone_enabled = 0;
int voc_enabled = 0;
int temperature_units = 0;
int output_only_particles = 0;
int new_temperature_sensor_enabled = 0;
int hih8120_enabled = 0;
int ozone_analog_enabled = 0;           //read ozone through analog or from ESP
int abc_logic_enabled = 0;
bool tried_cellular_connect = false;
int battery_threshold_enable;
int CO_socket = 0;
int google_location_en = 0;

char geolocation_latitude[12] = "111.1111111";
char geolocation_longitude[13] = "22.22222222";
char geolocation_accuracy[6] = "255.0";

//used for averaging
float CO_sum = 0;
float CO2_sum = 0;
float O3_sum = 0;

float PM25_sum = 0;
float PM10_sum = 0;
int measurement_count = 0;
double sound_average;


//calibration parameters
int ozone_offset;
int gas_lower_limit = 1200;   // Bad air quality limit
int gas_upper_limit = 50000;  // Good air quality limit
int measurements_to_average = 0;
int co2_calibration_timer = 0;


int sleepInterval = 60;  // This is used below for sleep times and is equal to 60 seconds of time.


//serial menu variables
int addr;
uint16_t value;
char recieveStr[5];

char incomingByte;  //serial connection from user

//Air quality index variables for calculating from humidity and VOC from BME680
float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score

float hum_score, gas_score;
float gas_reference = 250000;
float hum_reference = 40;


union{
    uint16_t status_int;
    char byte[2];
}status_word;

char cellular_status = 0;
char gps_status = 0;

/*
The status is being parsed as the last two bytes that are received from the BLE packet, bytes 19 and 20 (indexed from 0). Bit 15 is the MSB of byte 19, and bit 0 is the LSB of byte 20.

Bits 15-12; Mask 0xF000
	- 4 Bit Revision Number
	- (Status & 0xF000) >> 12

Bits 11-8; Mask 0x0F00
	- 4 Bit Build Number
	- (Status & 0x0F00) >> 8

Bit 3: Mask 0b11100
	GPS Indicator
	First Bit (0b10000): Type
		1: Geolocation
		0: GPS
	Last 2 Bits (0b01100): Quality
		11: Best
		      …
		00: Worst

Bits 1-0: Mask 0b11
	Cellular Indicator
	0b00: Not Enabled
	0b01: Enabled, Not Connected
	0b11: Enabled, Connected
	0b10: TBD

*/

PAMSerialMenu serial_menu;
uint16_t serial_menu_rd;
uint16_t rd = 0;

void testSerialOnData(uint16_t rd, uint8_t *data, uint8_t length) 
{
    Serial.printf("Main received Data!\n\r");
    if (length == 1 && *data == 'm') {
        PAMSerial.pushResponder(serial_menu_rd);
    }
}

void testSerialBecomesResponder(uint16_t rd, bool child_returned)
{
    PAMSerial.printf(rd, "Main became responder!\n\r");
}

class TestSerialConnector: public PAMSerialResponder {

public:
    TestSerialConnector(): PAMSerialResponder(BYTE) {};
    ~TestSerialConnector() {};
    void onData(uint16_t rd, uint8_t *data, uint8_t length) { testSerialOnData(rd, data, length); };
    void becomesResponder(uint16_t rd, bool child_returned) { testSerialBecomesResponder(rd, child_returned); };
};



//function declarations
void readStoredVars(void);
void checkCalFile(void);
size_t readField(File* file, char* str, size_t size, const char* delim);
void checkWifiFile(void);
void serialMenu();
void serialGetDeviceId(void);
void serialGetCo2Zero(void);
void serialGetCo2Zero(void);
void serialGetCoZero(void);
void serialGetCoZero(void);
void serialGetOzoneOffset(void);
void serialResetSettings(void);
// void serialTestRemoteFunction(void);
void serialIncreaseInputCurrent(void);
void writeLogFile(String data);

void outputSerialMenuOptions(void);
void outputToCloud(void);
void echoGps();
void readOzone(void);
// float readCO(void);
float getEspOzoneData(void);
void resetEsp(void);
void sendEspSerialCom(char *serial_command);
// int remoteWriteStoredVars(String addressAndValue);
// int remoteReadStoredVars(String mem_address);
void writeDefaultSettings(void);
void readHIH8120(void);

//gps functions
void enableLowPowerGPS(void);
void enableContinuousGPS(void);
void changeFrequency(void);
void sendPacket(byte *packet, byte len);
void sendPacket(byte *packet, byte len);

//google api callback
void locationCallback(float lat, float lon, float accuracy);



//test for setting up PMIC manually
void writeRegister(uint8_t reg, uint8_t value) {
    // This would be easier if pmic.writeRegister wasn't private
    Wire3.beginTransmission(PMIC_ADDRESS);
    Wire3.write(reg);
    Wire3.write(value);
    Wire3.endTransmission(true);

}


//todo: average everything except ozone
void outputToCloud(String data){
    String webhook_data = " ";
    // CO_sum += CO_float;
    CO_sum += pamco.co.adj_value;
    CO2_sum += t6713.CO2.adj_value;
    O3_sum = O3_float;      //do not average ozone because it is averaged on the ozone monitor
    measurement_count++;

    if(measurement_count == measurements_to_average){
        CO_sum /= measurements_to_average;
        CO2_sum /= measurements_to_average;
        //O3_sum /= measurements_to_average;

        measurement_count = 0;
        // String webhook_data = String(DEVICE_id) + ",VOC: " + String(bme.gas_resistance / 1000.0, 1) + ", CO: " + CO_sum + ", CO2: " + CO2_sum + ", PM1: " + PM01Value + ",PM2.5: " + corrected_PM_25 + ", PM10: " + PM10Value + ",Temp: " + String(readTemperature(), 1) + ",Press: ";
        String webhook_data = String(DEVICE_id) + ",VOC: " + String(tph_fusion.voc->adj_value, 1) + ", CO: " + CO_sum + ", CO2: " + CO2_sum + ", PM1: " + plantower.pm1.adj_value + ",PM2.5: " + plantower.pm2_5.adj_value + ", PM10: " + plantower.pm10.adj_value + ",Temp: " + String(readTemperature(), 1) + ",Press: ";
        // webhook_data += String(bme.pressure / 100.0, 1) + ",HUM: " + String(bme.humidity, 1) + ",Snd: " + String(sound_average) + ",O3: " + O3_sum + "\n\r";
        webhook_data += String(tph_fusion.pressure->adj_value, 1) + ",HUM: " + String(tph_fusion.humidity->adj_value, 1) + ",Snd: " + String(sound_average) + ",O3: " + O3_sum + "\n\r";

        if(Particle.connected() && serial_cellular_enabled){
            status_word.status_int |= 0x0002;
            Particle.publish("pamup", data, PRIVATE);
            Particle.process(); //attempt at ensuring the publish is complete before sleeping
            if(debugging_enabled){
              Serial.println("Published data!");
              writeLogFile("Published data!");
            }
        }else{
            if(serial_cellular_enabled == 0){
                if(debugging_enabled){
                    Serial.println("Cellular is disabled.");
                    writeLogFile("Cellular is disabled.");

                  }
            }else{
                status_word.status_int &= 0xFFFD;   //clear the connected bit
                if(debugging_enabled){
                    Serial.println("Couldn't connect to particle.");
                    writeLogFile("Couldn't connect to particle.");
                  }
            }
        }
        CO_sum = 0;
        CO2_sum = 0;
        O3_sum = 0;
    }
}

//send memory address and value separated by a comma
// int remoteWriteStoredVars(String addressAndValue){
//     uint16_t tempValue = 0;

//     int index_of_comma = addressAndValue.indexOf(',');
//     Serial.print("Full address and value substring: ");
//     Serial.println(addressAndValue);
//     String addressString = addressAndValue.substring(0, index_of_comma);
//     String valueString = addressAndValue.substring(index_of_comma + 1);

//     Serial.printf("address substring: %s\n\r", addressString);
//     Serial.printf("Value substring: %s\n\r", valueString);

//     int numerical_mem_address = addressString.toInt();
//     int numerical_value = valueString.toInt();

//     if(numerical_mem_address >= 0 && numerical_mem_address <= MAX_MEM_ADDRESS){
//         EEPROM.put(numerical_mem_address, numerical_value);
//         return 1;
//     }else{
//         return -1;
//     }

// }

// int remoteReadStoredVars(String mem_address){
//     uint16_t tempValue = 0;
//     int numerical_mem_address = mem_address.toInt();
//     if(numerical_mem_address >= 0 && numerical_mem_address <= MAX_MEM_ADDRESS){
//         EEPROM.get(numerical_mem_address, tempValue);
//         return tempValue;
//     }else{
//         return -1;
//     }
// }
//read all eeprom stored variables
void readStoredVars(void){
    Serial.println("MAIN IS READING STORED VARS");
    int tempValue;
    //just changing the rh calibration for temporary!! -- remove me!!
    //these values were determined by John Birks from 2019 cdphe study at la casa in denver February 2019


    EEPROM.get(DEVICE_ID_MEM_ADDRESS, DEVICE_id);
    return ;
    if(DEVICE_id == -1){
        DEVICE_id = 1555;
        writeDefaultSettings();
    }

    EEPROM.get(CO2_SLOPE_MEM_ADDRESS, tempValue);
    t6713.CO2.slope = tempValue;
    t6713.CO2.slope /= 100;
    EEPROM.get(CO_SLOPE_MEM_ADDRESS, tempValue);
    pamco.co.slope = tempValue;
    pamco.co.slope /= 100;
    EEPROM.get(PM_1_SLOPE_MEM_ADDRESS, tempValue);
    plantower.pm1.slope = tempValue;
    plantower.pm1.slope /= 100;
    EEPROM.get(PM_25_SLOPE_MEM_ADDRESS, tempValue);
    plantower.pm2_5.slope = tempValue;
    plantower.pm2_5.slope /= 100;
    EEPROM.get(PM_10_SLOPE_MEM_ADDRESS, tempValue);
    plantower.pm10.slope = tempValue;
    plantower.pm10.slope /= 100;
    EEPROM.get(TEMP_SLOPE_MEM_ADDRESS, tempValue);  //temperature
    tph_fusion.temperature->slope = tempValue;
    tph_fusion.temperature->slope /= 100;
    EEPROM.get(PRESSURE_SLOPE_MEM_ADDRESS, tempValue);
    tph_fusion.pressure->slope = tempValue;
    tph_fusion.pressure->slope /= 100;
    EEPROM.get(RH_SLOPE_MEM_ADDRESS, tempValue);
    tph_fusion.humidity->slope = tempValue;
    tph_fusion.humidity->slope /= 100;

    EEPROM.get(CO2_ZERO_MEM_ADDRESS, t6713.CO2.zero);
    EEPROM.get(CO_ZERO_MEM_ADDRESS, pamco.co.zero);
    EEPROM.get(PM_1_ZERO_MEM_ADDRESS, plantower.pm1.zero);
    EEPROM.get(PM_25_ZERO_MEM_ADDRESS, plantower.pm2_5.zero);
    EEPROM.get(PM_10_ZERO_MEM_ADDRESS, plantower.pm10.zero);
    EEPROM.get(TEMP_ZERO_MEM_ADDRESS, tph_fusion.temperature->zero);
    EEPROM.get(PRESSURE_ZERO_MEM_ADDRESS, tph_fusion.pressure->zero);
    EEPROM.get(RH_ZERO_MEM_ADDRESS, tph_fusion.humidity->zero);

    EEPROM.get(SERIAL_CELLULAR_EN_MEM_ADDRESS, serial_cellular_enabled);
    EEPROM.get(DEBUGGING_ENABLED_MEM_ADDRESS, debugging_enabled);
    EEPROM.get(OZONE_EN_MEM_ADDRESS, ozone_enabled);
    EEPROM.get(VOC_EN_MEM_ADDRESS, voc_enabled);
    EEPROM.get(GAS_LOWER_LIMIT_MEM_ADDRESS, gas_lower_limit);
    EEPROM.get(GAS_UPPER_LIMIT_MEM_ADDRESS, gas_upper_limit);
    EEPROM.get(TIME_ZONE_MEM_ADDRESS, tempValue);
    Time.zone(tempValue);
    EEPROM.get(TEMPERATURE_UNITS_MEM_ADDRESS, temperature_units);
    EEPROM.get(OUTPUT_PARTICLES_MEM_ADDRESS, output_only_particles);
    EEPROM.get(TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS, new_temperature_sensor_enabled);
    EEPROM.get(OZONE_A_OR_D_MEM_ADDRESS, ozone_analog_enabled);
    EEPROM.get(OZONE_OFFSET_MEM_ADDRESS, ozone_offset);
    EEPROM.get(MEASUREMENTS_TO_AVG_MEM_ADDRESS, measurements_to_average);
    EEPROM.get(BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS, battery_threshold_enable);
    EEPROM.get(ABC_ENABLE_MEM_ADDRESS, abc_logic_enabled);
    EEPROM.get(HIH8120_ENABLE_MEM_ADDRESS, hih8120_enabled);
    if (hih8120_enabled) {
        tph_fusion.enable_hih();
    } else {
        tph_fusion.disable_hih();
    }
    EEPROM.get(CO_SOCKET_MEM_ADDRESS, CO_socket);
    EEPROM.get(GOOGLE_LOCATION_MEM_ADDRESS, google_location_en);

    //measurements_to_average = 5;
    if(measurements_to_average < 1 || measurements_to_average > 5000)
        measurements_to_average = 1;

    //check all values to make sure are within limits
    if(!t6713.CO2.slope)
    {
        t6713.CO2.slope = 1;
    }
    if(!pamco.co.slope)
    {
        pamco.co.slope = 1;
    }
    if(!plantower.pm1.slope)
    {
        plantower.pm1.slope = 1;
    }
    if(!plantower.pm2_5.slope)
    {
        plantower.pm2_5.slope = 1;
    }
    if(!plantower.pm10.slope)
    {
        plantower.pm10.slope = 1;
    }
}

void writeDefaultSettings(void){
    Serial.println("WRITING DEFAULT SETTINGS");
    EEPROM.put(DEVICE_ID_MEM_ADDRESS, 1555);


    EEPROM.put(CO2_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(CO_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(PM_1_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(PM_25_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(PM_10_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(TEMP_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(PRESSURE_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(RH_SLOPE_MEM_ADDRESS, 100);

    EEPROM.put(CO2_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(CO_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(PM_1_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(PM_25_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(PM_10_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(TEMP_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(PRESSURE_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(RH_ZERO_MEM_ADDRESS, 0);

    EEPROM.put(SERIAL_CELLULAR_EN_MEM_ADDRESS, 0);
    EEPROM.put(DEBUGGING_ENABLED_MEM_ADDRESS, 0);
    EEPROM.put(OZONE_EN_MEM_ADDRESS, 0);
    EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
    EEPROM.put(GAS_LOWER_LIMIT_MEM_ADDRESS, 1000);
    EEPROM.put(GAS_UPPER_LIMIT_MEM_ADDRESS, 10000);
    EEPROM.put(TIME_ZONE_MEM_ADDRESS, -7);
    Time.zone(tempValue);
    EEPROM.put(TEMPERATURE_UNITS_MEM_ADDRESS, 0);
    EEPROM.put(OUTPUT_PARTICLES_MEM_ADDRESS, 0);
    EEPROM.put(TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS, 1);
    EEPROM.put(OZONE_A_OR_D_MEM_ADDRESS, 0);
    EEPROM.put(OZONE_OFFSET_MEM_ADDRESS,0);
    EEPROM.put(MEASUREMENTS_TO_AVG_MEM_ADDRESS, 1);
    EEPROM.put(BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS, 1);
    EEPROM.put(ABC_ENABLE_MEM_ADDRESS, 0);
    EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, 1);
    EEPROM.put(CO_SOCKET_MEM_ADDRESS, 0);
    EEPROM.put(GOOGLE_LOCATION_MEM_ADDRESS, 0);
}

size_t readField(File* file, char* str, size_t size, const char* delim) {
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1) {
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch)) {
        break;
    }
  }
  str[n] = '\0';
  return n;
}

void check_wifi_file(void){
    file1 = sd.open("wifi.txt", O_READ);
    size_t n;      // Length of returned field with delimiter.
    char str[50];  // Must hold longest field with delimiter and zero byte.
    // Read the file and print fields.
    int cred = 0;
    int i = 0;
    Serial.println("Contents of wifi file line by line:");
    while(1)
    {
        n = readField(&file1, str, sizeof(str), ",\n");
        // done if Error or at EOF.
        if (n == 0){
            break;
        }
        //Serial.print("I:");
        //Serial.print(i);
        //Serial.print(":");
        //Serial.println(str);
        //the first field is "SSID,PASSWORD", the second is the actual values
        if(i>1)
        {
            if (str[n-1] == ','){
              str[n-1] = 0;
              ssid = str;
              ssid.trim();
              cred++;
            }
            else if (str[n-1] == '\n') {
              str[n-1] = 0;
              password = str;
              password.trim();
              cred++;
            }
            else if(file1.available() == 0) { //There's a better way to do this. Buggy, fix soon!
              password = str;
              cred++;
            }
            else {
              // At eof, too long, or read error.  Too long is error.
              Serial.print(file1.available() ? F("error: ") : F("eof:   "));
            }
            if(cred >= 2){ //at least one pair of ssid and password
              Serial.print("Found SSID:");
              Serial.println(ssid);
              Serial.print("Found password:");
              Serial.println(password);
              break;
            }
            //WiFi.setCredentials(ssid, password);
        }

        i++;
    }
    file1.close();

}

void setup()
{
    Wire.begin();

    //initialize main serial port for debug output
    Serial.begin(9600);


    status_word.status_int  = 0;
    status_word.status_int |= (APP_VERSION << 12) & 0xFF00;
    status_word.status_int |= (BUILD_VERSION << 8) & 0xF00;
    //status_word.status_int |= 0x6500;

    String init_log; //intialization error log


    setADCSampleTime(ADC_SampleTime_480Cycles);
    //setup i/o
    pinMode(FIVE_VOLT_EN, OUTPUT);
    pinMode(PLANTOWER_EN, OUTPUT);
    pinMode(POWER_LED_EN, OUTPUT);
    pinMode(ESP_WROOM_EN, OUTPUT);
    pinMode(BLOWER_EN, OUTPUT);
    pinMode(D4, INPUT);
    pinMode(CO2_EN, OUTPUT);

    //read all stored variables (calibration parameters)
     readStoredVars();

    pmic.begin();
    pmic.setChargeVoltage(4208);      //  Set Li-Po charge termination voltage to 4.21V,
    //pmic.setChargeCurrent(0,1,1,0,0,0);
    //pmic.setInputCurrentLimit(1200);
    pmic.enableCharging();
    writeRegister(0, 0b00110100);
    writeRegister(1, 0b00011011);
    //writeRegister(2, 0b01100000);
    //check power
    powerCheck.loop();

    if((battery_threshold_enable == 1) && (fuel.getSoC() < BATTERY_THRESHOLD) && (powerCheck.getHasPower() == 0)){
        //Serial.println("Going to sleep because battery is below 20% charge");
        goToSleepBattery();
    }
    //if user presses power button during operation, reset and it will go to low power mode
    attachInterrupt(D4, System.reset, RISING);
    if(digitalRead(D4)){
      goToSleep();
    }

    digitalWrite(POWER_LED_EN, HIGH);
    digitalWrite(PLANTOWER_EN, HIGH);
    digitalWrite(ESP_WROOM_EN, HIGH);
    digitalWrite(BLOWER_EN, HIGH);
    digitalWrite(CO2_EN, HIGH);
    digitalWrite(FIVE_VOLT_EN, HIGH);



    // register the cloud function
    // Particle.function("geteepromdata", remoteReadStoredVars);
    //debugging_enabled = 1;  //for testing...
    //initialize serial1 for communication with BLE nano from redbear labs
    Serial1.begin(9600);
    //init serial4 to communicate with Plantower PMS5003
    // Serial4.begin(9600);
    Serial5.begin(9600);        //gps is connected to this serial port
    //set the Timeout to 1500ms, longer than the data transmission periodic time of the sensor
    // Serial4.setTimeout(5000);

    // Setup the PMIC manually (resets the BQ24195 charge controller)
    // REG00 Input Source Control Register  (disabled)
    /*writeRegister(0, 0b00110000);  //0x30

    // REG01 Power-On Configuration Register (charge battery, 3.5 V)
    writeRegister(1, 0b00011011);   //0x1B

    // REG02 Charge Current Control Register (1024 + 512 mA)
    writeRegister(2, 0b01100000);   //0x60

    // REG03 Pre-Charge/Termination Current Control Register (128mA pre charge, 128mA termination current)
    writeRegister(3, 0b00010001);   //0x11

    // REG04 Charge Voltage Control Register Format
    writeRegister(4, 0b10110010);   //0xB2

    // REG05 Charge Termination/Timer Control Register
    writeRegister(5, 0b10011010);   //0x9A
    // REG06 Thermal Regulation Control Register
    writeRegister(6, 0b00000011);   //0x03
    // REG07 Misc Operation Control Register Format
    writeRegister(7, 0b01001011);   //0x4B*/


    //delay for 5 seconds to give time to programmer person for connecting to serial port for debugging
    delay(10000);

    rd = PAMSerial.registerResponder(new TestSerialConnector());
    PAMSerial.pushResponder(rd);
    PAMSerial.printf(rd, "TESTING FROM PAM SERIAL\n\r");
    serial_menu_rd = PAMSerial.registerResponder(&serial_menu);


    #if sd_en
     fileName = String(DEVICE_id) + "_" + String(Time.year()) + String(Time.month()) + String(Time.day()) + "_" + String(Time.hour()) + String(Time.minute()) + String(Time.second()) + ".txt";
     Serial.println("Checking for sd card");
     logFileName = "log_" + fileName;

    if (sd.begin(CS)) { //if uSD is functioning and MCP error has not been logged yet.
      /*file.open("log.txt", O_CREAT | O_APPEND | O_WRITE);
      file.remove();
      file.open("log.txt", O_CREAT | O_APPEND | O_WRITE);
      init_log += "MCP,";
      file.print(init_log);
      file.close();

      //look for a wifi file
      check_wifi_file();
      //look for a calibration file
      check_cal_file();*/

      Serial.print("Created new file to log to uSD card: ");
      Serial.println(fileName);
    }else { //uSD is not functioning
        Serial.println("No uSD card detected.");
    }
    #endif
    
    resetESP();

    Serial.println("ESP reset!");

    Serial.print("FW Version: ");
    Serial.println(APP_VERSION);
    Serial.print("Build: ");
    Serial.println(BUILD_VERSION);



    enableContinuousGPS();

    if(google_location_en){
        Serial.println("Setting up google maps geolocation.");
        locator.withSubscribe(locationCallback).withLocatePeriodic(5); //setup google maps geolocation
    }

    PAMSensorManager *manager = PAMSensorManager::GetInstance();
    manager->addSensor(&t6713);
    manager->addSensor(&tph_fusion);
    manager->addSensor(&plantower);
    manager->addSensor(&pamco);
    serial_menu.addResponder(PAMSensorManager::GetInstance()->serial_menu_rd, "Sensor Settings");

    char *csv_header = manager->csvHeader();
    Serial.println(csv_header);
    free(csv_header);

    
    Log.info("System version: %s", (const char*)System.version());

}

void locationCallback(float lat, float lon, float accuracy) {
  // Handle the returned location data for the device. This method is passed three arguments:
  // - Latitude
  // - Longitude
  // - Accuracy of estimated location (in meters)
  Serial.println("google geolocation:");
  Serial.printlnf("Latitude:%f, longitude:%f, acc:%f", lat, lon, accuracy);
  snprintf(geolocation_latitude, sizeof(geolocation_latitude), "%.6f", lat);
  snprintf(geolocation_longitude, sizeof(geolocation_longitude), "%.6f", lon);
  snprintf(geolocation_accuracy, sizeof(geolocation_accuracy), "%3.2f", accuracy);
  if(gps.get_latitude() == 0){
      status_word.status_int |= 0x0008;
      status_word.status_int &= 0xFFF3;
      if(accuracy < 2){
          status_word.status_int |= 0x000C;
      }else if(accuracy < 5){
          status_word.status_int |= 0x0008;
      }else if(accuracy < 20){
          status_word.status_int |= 0x0004;
      }
  }
}

void loop() {


    //Serial.println("locator loop");
    locator.loop();

    PAMSensorManager::GetInstance()->loop();
    PAMSerial.loop();


    if(output_only_particles == 1){
        outputParticles();
    }
    readGpsStream();

    CO2_float = readCO2();


    //correct for altitude
    // float pressure_correction = bme.pressure/100;
    float pressure_correction = tph_fusion.pressure->adj_value;
    if(pressure_correction > LOW_PRESSURE_LIMIT && pressure_correction < HIGH_PRESSURE_LIMIT){
        pressure_correction /= SEALEVELPRESSURE_HPA;
        if(debugging_enabled){
            Serial.printf("pressure correction factor for CO2:%1.2f\n\r", pressure_correction);

        }
        CO2_float *= pressure_correction;
    }else{
        // Serial.println("Error: Pressure out of range, not using pressure correction for CO2.");
        // Serial.printf("Pressure=%1.2f\n\r", pressure_correction);

    }


    if(ozone_enabled){
        readOzone();
    }


    //sound_average = 0;
    calculateAQI();
    sound_average = readSound();

    //getEspWifiStatus();
    outputDataToESP();

    sample_counter = ++sample_counter;
    if(sample_counter == 99)    {
          sample_counter = 0;
    }

    // if (Serial.available() > 0) {
    // if (false) {
    //     // read the incoming byte:
    //     incomingByte = Serial.read();
    //     if(debugging_enabled){
    //         Serial.print("incomming byte:");
    //         Serial.println(incomingByte);

    //     }
    //     Serial.println(incomingByte);
    //     if(incomingByte == 'm'){
    //       serialMenu();
    //     }
    // }

    if(serial_cellular_enabled){
        status_word.status_int |= 0x01;
        //Serial.println("Cellular is enabled.");
      if (Particle.connected() == false && tried_cellular_connect == false) {
        tried_cellular_connect = true;
          if(debugging_enabled){
            Serial.println("Connecting to cellular network");
            writeLogFile("Connecting to cellular network");
          }
          Cellular.on();
          if(debugging_enabled){
            Serial.println("after cellularOn");
            writeLogFile("After cellularOn");
          }
          Particle.connect();
          if(debugging_enabled){
            Serial.println("After particle connect");
            writeLogFile("After particle connect");
          }
      }else if(Particle.connected() == true){  //this means that it is already connected
        if(debugging_enabled){
          Serial.println("setting tried_cellular_connect to false");
        }
        tried_cellular_connect = false;
      }
    }else{
        //Serial.println("Cellular is disabled.");
      if (Particle.connected() == true) {
          if(debugging_enabled){
            Serial.println("Disconnecting from cellular network");
            writeLogFile("Disconnecting from cellular network");
          }
          Cellular.off();
      }
    }

    //check power
    powerCheck.loop();

	//Serial.printf("hasPower=%d hasBattery=%d isCharging=%d\n\r", powerCheck.getHasPower(), powerCheck.getHasBattery(), powerCheck.getIsCharging());
    if((battery_threshold_enable == 1) && (fuel.getSoC() < BATTERY_THRESHOLD) && (powerCheck.getHasPower() == 0)){
        Serial.println("Going to sleep because battery is below 20% charge");
        goToSleepBattery();
    }

}

void calculateAQI(void){
    //Calculate humidity contribution to IAQ index
        // gas_reference = bme.gas_resistance/100;
    gas_reference = tph_fusion.voc->adj_value;
      float current_humidity = readHumidity();
      if(debugging_enabled){
          Serial.printf("gas resistance: %1.0f, humidity: %1.2f\n\r", gas_reference, current_humidity);

      }
      if (current_humidity >= 38 && current_humidity <= 42)
        hum_score = 0.25*100; // Humidity +/-5% around optimum
      else
      { //sub-optimal
        if (current_humidity < 38)
          hum_score = 0.25/hum_reference*current_humidity*100;
        else
        {
          hum_score = ((-0.25/(100-hum_reference)*current_humidity)+0.416666)*100;
        }
      }

      //Calculate gas contribution to IAQ index

      if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit;
      if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
      gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;
      if(debugging_enabled){
        Serial.print("Gas score: ");
        Serial.println(gas_score);
        Serial.print("Humidity score: ");
        Serial.println(hum_score);
    }

      //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
      air_quality_score = hum_score + gas_score;


}

void echoGps(){
    char gps_byte = 0;
    while(!Serial.available()){
        if(Serial5.available() > 0){
            gps_byte = Serial5.read();
            Serial.print(gps_byte);
        }

    }
}

/*void disableGPS(void){
    Serial.println("Turning off gps");
    String disableString = "";
    Serial5.write()
}*/
void readGpsStream(void){
    String gps_sentence = "init";
    int stringFound = 0;
    int error = 0;
    int comma_counter = 0;
    while(!stringFound && !error){
        gps_sentence = Serial5.readStringUntil('\r');
        String prefix_string = gps_sentence.substring(4,7);
        if(prefix_string.equals("GGA")){

            //
            //Serial.print("prefix string: ");
            //Serial.println(prefix_string);
            //Serial.print("String:");
            //Serial.println(gps_sentence);
            stringFound = 1;
        }else if(gps_sentence.equals("init")){
            error = 1;
            Serial.println("Error reading GPS");
            writeLogFile("Error reading GPS");
        }
    }
    if(stringFound){

        //parse the gps string into latitude, longitude
        //UTC time is after first comma
        //Latitude is after second comma (ddmm.mmmm)
        //N/S indicator is after 3rd comma
        //longitude is after 4th comma (dddmm.mmmm)
        //E/W indicator is after 5th comma
        //quality is after 6th comma
        //gps_sentence = String("$GNGGA,011545.00,3945.81586,N,10514.09384,W,1,08,1.28,1799.4,M,-21.5,M,,*40");
        //
        comma_counter = 0;

        for(int a = 0; a<gps_sentence.length(); a++){
            if(gps_sentence.charAt(a) == ','){
                if(comma_counter == TIME_FIELD_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String utc_string = gps_sentence.substring(a+1,a+11);
                        //Serial.print("GPS utc string: ");
                        if(debugging_enabled){
                            Serial.print("GPS utc string: ");
                            Serial.println(utc_string);

                        }
                        //Serial.println(utc_string);
                    }
                }else if(comma_counter == LATITUDE_FIELD_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String latitudeString = gps_sentence.substring(a+1,a+10);
                        if(debugging_enabled){
                          Serial.print("Latitude string: ");
                          Serial.print(latitudeString);
                        }
                        //Serial.print("Latitude string: ");
                        //Serial.print(latitudeString);
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+12));
                        gps.set_lat_decimal(latitudeString, gps_sentence.charAt(a+12));
                        status_word.status_int &= 0xFFF7;
                        //Serial.print("Latitude decimal: ");
                        //Serial.println(gps.get_latitude(), 5);
                    }
                }else if(comma_counter == LONGITUDE_FIELD_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String longitudeString = gps_sentence.substring(a+1,a+11);
                        if(debugging_enabled){
                          Serial.print("longitude string: ");
                          Serial.print(longitudeString);
                        }
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+13));
                        gps.set_long_decimal(longitudeString, gps_sentence.charAt(a+13));
                        //Serial.print("Longitude decimal: ");
                        //Serial.println(gps.get_longitude(), 5);
                    }
                }else if(comma_counter == NUMBER_OF_SATELLITES_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String numberOfSatellitesString = gps_sentence.substring(a+1,a+3);
                        gps.set_satellites(numberOfSatellitesString);
                    }
                }else if(comma_counter == HORZONTAL_DILLUTION_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String hdString = gps_sentence.substring(a+1,a+3);
                        gps.set_horizontalDillution(hdString);
                        status_word.status_int &= 0xFFF3;
                        if(gps.get_horizontalDillution() < 2){
                            status_word.status_int |= 0x000C;
                        }else if(gps.get_horizontalDillution() < 5){
                            status_word.status_int |= 0x0008;
                        }else if(gps.get_horizontalDillution() < 20){
                            status_word.status_int |= 0x0004;
                        }


                    }
                }
                comma_counter++;
            }
        }
    }

}

void readGpsStreamDate(void){
    String gps_sentence = "init";
    int stringFound = 0;
    int error = 0;
    int comma_counter = 0;
    while(!stringFound && !error){
        gps_sentence = Serial5.readStringUntil('\r');
        String prefix_string = gps_sentence.substring(4,7);
        if(prefix_string.equals("RMC")){

            //
            //Serial.print("prefix string: ");
            //Serial.println(prefix_string);
            //Serial.print("String:");
            //Serial.println(gps_sentence);
            stringFound = 1;
        }else if(gps_sentence.equals("init")){
            error = 1;
            Serial.println("Error reading GPS RMC");
            writeLogFile("Error reading GPS RMC");
        }
    }
    if(stringFound){

        //parse the gps string into latitude, longitude
        //UTC time is after first comma
        //Latitude is after second comma (ddmm.mmmm)
        //N/S indicator is after 3rd comma
        //longitude is after 4th comma (dddmm.mmmm)
        //E/W indicator is after 5th comma
        //quality is after 6th comma
        //gps_sentence = String("$GNGGA,011545.00,3945.81586,N,10514.09384,W,1,08,1.28,1799.4,M,-21.5,M,,*40");
        //
        comma_counter = 0;

        for(int a = 0; a<gps_sentence.length(); a++){
            if(gps_sentence.charAt(a) == ','){
                if(comma_counter == DATE_FIELD_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String utc_string = gps_sentence.substring(a+1,a+11);
                        //Serial.print("GPS utc string: ");
                        if(debugging_enabled){
                            Serial.print("GPS utc string: ");
                            Serial.println(utc_string);

                        }
                        //Serial.println(utc_string);
                    }
                }else if(comma_counter == LATITUDE_FIELD_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String latitudeString = gps_sentence.substring(a+1,a+10);
                        if(debugging_enabled){
                          Serial.print("Latitude string: ");
                          Serial.print(latitudeString);
                        }
                        //Serial.print("Latitude string: ");
                        //Serial.print(latitudeString);
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+12));
                        gps.set_lat_decimal(latitudeString, gps_sentence.charAt(a+12));
                        status_word.status_int &= 0xFFF7;
                        //Serial.print("Latitude decimal: ");
                        //Serial.println(gps.get_latitude(), 5);
                    }
                }else if(comma_counter == LONGITUDE_FIELD_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String longitudeString = gps_sentence.substring(a+1,a+11);
                        if(debugging_enabled){
                          Serial.print("longitude string: ");
                          Serial.print(longitudeString);
                        }
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+13));
                        gps.set_long_decimal(longitudeString, gps_sentence.charAt(a+13));
                        //Serial.print("Longitude decimal: ");
                        //Serial.println(gps.get_longitude(), 5);
                    }
                }else if(comma_counter == NUMBER_OF_SATELLITES_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String numberOfSatellitesString = gps_sentence.substring(a+1,a+3);
                        gps.set_satellites(numberOfSatellitesString);
                    }
                }else if(comma_counter == HORZONTAL_DILLUTION_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String hdString = gps_sentence.substring(a+1,a+3);
                        gps.set_horizontalDillution(hdString);
                        status_word.status_int &= 0xFFF3;
                        if(gps.get_horizontalDillution() < 2){
                            status_word.status_int |= 0x000C;
                        }else if(gps.get_horizontalDillution() < 5){
                            status_word.status_int |= 0x0008;
                        }else if(gps.get_horizontalDillution() < 20){
                            status_word.status_int |= 0x0004;
                        }


                    }
                }
                comma_counter++;
            }
        }
    }

}
// Send a packet to the receiver to change frequency to 100 ms.
void changeFrequency()
{
    // CFG-RATE packet.
    byte packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x08, // id
        0x06, // length
        0x00, // length
        0x64, // payload
        0x00, // payload
        0x01, // payload
        0x00, // payload
        0x01, // payload
        0x00, // payload
        0x7A, // CK_A
        0x12, // CK_B
    };

    sendPacket(packet, sizeof(packet));
}

void enableContinuousGPS()
{
    // CFG-MSG packet.
    byte packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x11, // id
        0x02, // length
        0x00, // length
        0x00, // payload
        0x00, // payload
        0x1A, // CK_A
        0x83, // CK_B
    };

    sendPacket(packet, sizeof(packet));
}

void enableLowPowerGPS()
{
    // CFG-MSG packet.
    byte packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x11, // id
        0x02, // length
        0x00, // length
        0x01, // payload
        0x00, // payload
        0x1A, // CK_A
        0x83, // CK_B
    };

    sendPacket(packet, sizeof(packet));
}

// Send the packet specified to the receiver.
void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
    {
        Serial5.write(packet[i]);
    }

    printPacket(packet, len);
}

// Print the packet specified to the PC serial in a hexadecimal form.
void printPacket(byte *packet, byte len)
{
    char temp[3];

    for (byte i = 0; i < len; i++)
    {
        sprintf(temp, "%.2X", packet[i]);
        Serial.print(temp);

        if (i != len - 1)
        {
            Serial.print(' ');
        }
    }

    Serial.println();
}

float readTemperature(void){
    return tph_fusion.temperature->adj_value;
}

float readHumidity(void){
    return tph_fusion.humidity->adj_value;
}
//read sound from
double readSound(void){
    int val;
    double sum = 0;
    float average = 0;
    for(int i=0; i< 10;i++){
        val = analogRead(SOUND_INPUT);
        sum += val;
        //Serial.print("Sound level: ");
        //Serial.println(val);
    }
    sum = sum/10;
    sum /= 4095;
    sum *= 100;
    return sum;
}


float readCO2(void){
    t6713.measure();
    if (t6713.CO2.adj_value != 0 && t6713.CO2.adj_value != VALUE_UNKNOWN) {
        CO2_float = t6713.CO2.adj_value;
    }
    return CO2_float;
}

void readOzone(void){
    int tempValue = 0;
    if(ozone_analog_enabled){
        tempValue = analogRead(A0);  // read the analogPin for ozone voltage
        if(debugging_enabled){
            Serial.print("Ozone Raw analog in:");
            Serial.println(tempValue);

        }
        O3_float = tempValue;
        O3_float *= VOLTS_PER_UNIT;           //convert digital reading to voltage
        O3_float /= VOLTS_PER_PPB;            //convert voltage to ppb of ozone
        O3_float += ozone_offset;
    }else{
        O3_float = getEspOzoneData();
    }
}

void writeLogFile(String data){
  if (sd.begin(CS)){
      Serial.println("Writing data to log file.");
      log_file.open(logFileName, O_CREAT | O_APPEND | O_WRITE);
      if(log_file_started == 0){
          log_file.println("File Start timestamp: ");
          log_file.println(Time.timeStr());
          log_file_started = 1;
      }
      log_file.println(data);

      log_file.close();
  }else{
    Serial.println("Unable to write to log file");
  }
}

void outputDataToESP(void){
    //used for converting double to bytes for latitude and longitude
    char buffer[2];
    union{
	       double myDouble;
	       unsigned char bytes[sizeof(double)];
    } doubleBytes;
    //doubleBytes.myDouble = double;
    //

    //used for converting float to bytes for measurement value
    union {
        float myFloat;
        unsigned char bytes[4];
    } floatBytes;

    //used for converting word to bytes for lat and longitude
    union {
        int16_t myWord;
        unsigned char bytes[2];
    }wordBytes;


    //get a current time string
    time_t time = Time.now();
    Time.setFormat(TIME_FORMAT_ISO8601_FULL);



    //************Fill the cloud output array and file output array for row in csv file on usd card*****************************/
    //This is different than the ble packet in that we are putting all of the data that we have in one packet
    //"$1:D555g47.7M-22.050533C550.866638r1R1q2T45.8P844.9h17.2s1842.700000&"
    String cloud_output_string = "";    //create a clean string
    String csv_output_string = "";
    cloud_output_string += '^';         //start delimeter
    cloud_output_string += String(1) + ";";           //header
    cloud_output_string += String(DEVICE_ID_PACKET_CONSTANT) + String(DEVICE_id);   //device id
    csv_output_string += String(DEVICE_id) + ",";
    // cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(CO_float, 3);
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(pamco.co.adj_value, 3);
    // csv_output_string += String(CO_float, 3) + ",";
    csv_output_string += String(pamco.co.adj_value, 3) + ",";
    #if AFE2_en
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(CO_float_2, 3);
    csv_output_string += String(CO_float_2, 3) + ",";
    #endif
    cloud_output_string += String(CARBON_DIOXIDE_PACKET_CONSTANT) + String(CO2_float, 0);
    csv_output_string += String(CO2_float, 0) + ",";
    if(voc_enabled){
        cloud_output_string += String(VOC_PACKET_CONSTANT) + String(air_quality_score, 1);
        csv_output_string += String(air_quality_score, 1) + ",";
    }
    // cloud_output_string += String(PM1_PACKET_CONSTANT) + String(PM01Value);
    cloud_output_string += String(PM1_PACKET_CONSTANT) + String(plantower.pm1.adj_value);
    // csv_output_string += String(PM01Value) + ",";
    csv_output_string += String(plantower.pm1.adj_value) + ",";
    // cloud_output_string += String(PM2PT5_PACKET_CONSTANT) + String(corrected_PM_25, 0);
    cloud_output_string += String(PM2PT5_PACKET_CONSTANT) + String(plantower.pm2_5.adj_value, 0);
    // csv_output_string += String(corrected_PM_25, 0) + ",";
    csv_output_string += String(plantower.pm2_5.adj_value, 0) + ",";
    // cloud_output_string += String(PM10_PACKET_CONSTANT) + String(PM10Value);
    cloud_output_string += String(PM10_PACKET_CONSTANT) + String(plantower.pm10.adj_value);
    // csv_output_string += String(PM10Value) + ",";
    csv_output_string += String(plantower.pm10.adj_value) + ",";

    cloud_output_string += String(TEMPERATURE_PACKET_CONSTANT) + String(readTemperature(), 1);
    csv_output_string += String(readTemperature(), 1) + ",";
    // cloud_output_string += String(PRESSURE_PACKET_CONSTANT) + String(bme.pressure / 100.0, 1);
    cloud_output_string += String(PRESSURE_PACKET_CONSTANT) + String(tph_fusion.pressure->adj_value, 1);
    // csv_output_string += String(bme.pressure / 100.0, 1) + ",";
    csv_output_string += String(tph_fusion.pressure->adj_value, 1) + ",";
    cloud_output_string += String(HUMIDITY_PACKET_CONSTANT) + String(readHumidity(), 1);
    csv_output_string += String(readHumidity(), 1) + ",";
    if(ozone_enabled){
        cloud_output_string += String(OZONE_PACKET_CONSTANT) + String(O3_float, 1);
        csv_output_string += String(O3_float, 1) + ",";
    }
    cloud_output_string += String(BATTERY_PACKET_CONSTANT) + String(fuel.getSoC(), 1);
    csv_output_string += String(fuel.getSoC(), 1) + ",";
    cloud_output_string += String(SOUND_PACKET_CONSTANT) + String(sound_average, 0);

    csv_output_string += String(sound_average, 0) + ",";
    cloud_output_string += String(LATITUDE_PACKET_CONSTANT);

    if(gps.get_latitude() != 0){
        if(gps.get_nsIndicator() == 0){
            csv_output_string += "-";
            cloud_output_string += "-";
        }
        csv_output_string += String(gps.get_latitude()) + ",";
        cloud_output_string += String(gps.get_latitude());
    }else{
        csv_output_string += String(geolocation_latitude)+ ",";
        cloud_output_string += String(geolocation_latitude);
    }

    cloud_output_string += String(LONGITUDE_PACKET_CONSTANT);

    if(gps.get_longitude() != 0){
        if(gps.get_ewIndicator() == 0x01){
            csv_output_string += "-";
            cloud_output_string += "-";
        }
        csv_output_string += String(gps.get_longitude()) + ",";
        cloud_output_string += String(gps.get_longitude());
    }else{
        csv_output_string += String(geolocation_longitude) + ",";
        cloud_output_string += String(geolocation_longitude);
    }

    cloud_output_string += String(ACCURACY_PACKET_CONSTANT);
    if (gps.get_longitude() != 0) {
        csv_output_string += String(gps.get_horizontalDillution() / 10.0) + ",";
        cloud_output_string += String(gps.get_horizontalDillution() / 10.0);
    } else {
        csv_output_string += String(geolocation_accuracy) + ",";
        cloud_output_string += String(geolocation_accuracy);
    }

    csv_output_string += String(status_word.status_int) + ",";
    csv_output_string += String(Time.format(time, "%d/%m/%y,%H:%M:%S"));
    cloud_output_string += String(PARTICLE_TIME_PACKET_CONSTANT) + String(Time.now());
    cloud_output_string += '&';
    if(debugging_enabled){
        Serial.println("Line to write to cloud:");
        Serial.println(cloud_output_string);
    }
    if(!esp_wifi_connection_status){
        if(debugging_enabled){
            Serial.println("No wifi from esp so trying cellular function...");
          }
        outputToCloud(cloud_output_string);
    }else{
        if(debugging_enabled){
            Serial.println("Sending data to esp to upload via wifi...");
            writeLogFile("Sending data to esp to upload via wifi");
          }
        Serial1.println(cloud_output_string);
    }
    PAMSerial.println(rd, csv_output_string);

    //write data to file
    if (sd.begin(CS)){
        if(debugging_enabled)
            Serial.println("Writing row to file.");
        file.open(fileName, O_CREAT | O_APPEND | O_WRITE);
        if(file_started == 0){
            file.println("File Start timestamp: ");
            file.println(Time.timeStr());
            file.println(String(HEADER_STRING));
            file_started = 1;
        }
        file.println(csv_output_string);

        file.close();
    }
    //delay(5000);

    //Serial.print("Successfully output Cloud string to ESP: ");
    //Serial.println(cloud_output_string);

    //create an array of binary data to store and send all data at once to the ESP
    //Each "section" in the array is separated by a #
    //we are using binary for the ble packets so we can compress the data into 19 bytes for the small payload

    byte ble_output_array[NUMBER_OF_SPECIES*BLE_PAYLOAD_SIZE];     //19 bytes per data line and 12 species to output


    for(int i=0; i<NUMBER_OF_SPECIES; i++){

        //************Fill the ble output array**********************//
        //Serial.printf("making array[%d]\n", i);
        //byte 0 - version
        ble_output_array[0 + i*(BLE_PAYLOAD_SIZE)] = 1;

        //bytes 1,2 - Device ID
        //DEVICE_id = 555;
        wordBytes.myWord = DEVICE_id;
        ble_output_array[1 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[2 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        //byte 3 - Measurement number
        ble_output_array[3 + i*(BLE_PAYLOAD_SIZE)] = sample_counter;

        //byte 4 - Identifier (B:battery, a:Latitude, o:longitude,
        //t:Temperature, P:Pressure, h:humidity, s:Sound, O:Ozone,
        //C:CO2, M:CO, r:PM1, R:PM2.5, q:PM10, g:VOCs)
        /*
        0-CO_float
        1-CO2_float
        2-bme.gas_resistance / 1000.0
        3-PM01Value
        4-PM2_5Value
        5-PM10Value
        6-bme.temperature
        7-bme.pressure / 100.0
        8-bme.humidity
        9-O3_float
        10-fuel.getSoC()
        11-sound_average



        */
        if(i == 0){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_MONOXIDE_PACKET_CONSTANT;
            // floatBytes.myFloat = CO_float;
            floatBytes.myFloat = pamco.co.adj_value;
        }else if(i == 1){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_DIOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = CO2_float;
        }else if(i == 2){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
            floatBytes.myFloat = fuel.getSoC();
        }else if(i == 3){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
            // floatBytes.myFloat = PM01Value;
            floatBytes.myFloat = plantower.pm1.adj_value;
        }else if(i == 4){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
            // floatBytes.myFloat = corrected_PM_25;
            floatBytes.myFloat = plantower.pm2_5.adj_value;
        }else if(i == 5){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
            // floatBytes.myFloat = PM10Value;
            floatBytes.myFloat = plantower.pm10.adj_value;
        }else if(i == 6){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = TEMPERATURE_PACKET_CONSTANT;
            floatBytes.myFloat = readTemperature();
        }else if(i == 7){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PRESSURE_PACKET_CONSTANT;
            // floatBytes.myFloat = bme.pressure / 100.0;
            floatBytes.myFloat = tph_fusion.pressure->adj_value;
        }else if(i == 8){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = HUMIDITY_PACKET_CONSTANT;
            floatBytes.myFloat = readHumidity();
        }else if(i == 9){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = SOUND_PACKET_CONSTANT;
            floatBytes.myFloat = sound_average;
        }else if(i == 10){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = VOC_PACKET_CONSTANT;
            floatBytes.myFloat = air_quality_score;
        }/*else if(i == 11){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = OZONE_PACKET_CONSTANT;
            floatBytes.myFloat = O3_float;
        }*/

        //bytes 5,6,7,8 - Measurement Value
        ble_output_array[5 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[0];
        ble_output_array[6 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[1];
        ble_output_array[7 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[2];
        ble_output_array[8 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[3];


        //bytes 9-12 - latitude
        wordBytes.myWord = gps.get_latitudeWhole();
        ble_output_array[9 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[10 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        wordBytes.myWord = gps.get_latitudeFrac();
        ble_output_array[11 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[12 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        //bytes 14-17 - longitude
        wordBytes.myWord = gps.get_longitudeWhole();
        ble_output_array[13 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[14 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        wordBytes.myWord = gps.get_longitudeFrac();
        ble_output_array[15 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[16 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];


        //byte 18 - east west and north south indicator
        //  LSB 0 = East, LSB 1 = West
        //  MSB 0 = South, MSB 1 = North
        int northSouth = gps.get_nsIndicator();
        int eastWest = gps.get_ewIndicator();

        ble_output_array[17 + i*(BLE_PAYLOAD_SIZE)] = northSouth | eastWest;
        ble_output_array[18 + i*(BLE_PAYLOAD_SIZE)] = gps.get_horizontalDillution();
        ble_output_array[19 + i*(BLE_PAYLOAD_SIZE)] = status_word.byte[1];
        ble_output_array[20 + i*(BLE_PAYLOAD_SIZE)] = status_word.byte[0];

        ble_output_array[21 + i*(BLE_PAYLOAD_SIZE)] = '#';     //delimeter for separating species

    }

    //send start delimeter to ESP
    Serial1.print("$");
    //send the packaged data with # delimeters in between packets
    Serial1.write(ble_output_array, NUMBER_OF_SPECIES*BLE_PAYLOAD_SIZE);

    //send ending delimeter
    Serial1.print("&");

    /*Serial.println("Successfully output BLE string to ESP");
    for(int i=0;i<NUMBER_OF_SPECIES*BLE_PAYLOAD_SIZE;i++){
        Serial.printf("array[%d]:%X ", i, ble_output_array[i]);
        if(ble_output_array[i]=='#')
            Serial.printf("\n\r");
    }
    Serial.println("End of array");*/

}

//ask the ESP if it has a wifi connection
void getEspWifiStatus(void){
    //command to ask esp for wifi status
    String doYouHaveWifi = "!&";
    char yes_or_no = ' ';
    //if esp doesn't answer, keep going
    //Serial1.setTimeout(5000);

    Serial1.print(doYouHaveWifi);
    while(!Serial1.available());
    //delay(1000);
    yes_or_no = Serial1.read();
    if(debugging_enabled){
        Serial.print("ESP Wifi connection status is: ");

      }
    //Serial.println(yes_or_no);
    if(yes_or_no == 'y'){
        if(debugging_enabled){
            Serial.println("Connected!");
            writeLogFile("ESP wifi connected");
          }
        esp_wifi_connection_status = 1;
    }else{
        if(debugging_enabled){
            Serial.println("No Connection");
            writeLogFile("ESP wifi not connected");
          }
        esp_wifi_connection_status = 0;
    }
}
//send wifi information to the ESP
void sendWifiInfo(void){
    String wifiCredentials = "@!" + String(ssid) + "," + String(password) + "&";
    Serial.println("Sending new wifi credentials to ESP");
    Serial1.println(wifiCredentials);
    Serial.println("Success!");
}

float getEspOzoneData(void){
    float ozone_value = 0.0;
    String getOzoneData = "Z&";
    String recievedData = " ";
    bool timeOut = false;
    double counterIndex = 0;
    //if esp doesn't answer, keep going
    Serial1.setTimeout(3000);
    if(debugging_enabled){
        Serial.println("Getting ozone data from esp");
        writeLogFile("Getting ozone data from esp");
      }
    Serial1.print(getOzoneData);
    while(!Serial1.available() && timeOut == false){
      //delay(1);
      counterIndex++;
      if(counterIndex > MAX_COUNTER_INDEX){
        if(debugging_enabled){
          Serial.printf("Unable to get ozone data from ESP, counter index: %1.1f\n\r", counterIndex);
        }
        timeOut = true;
      }
    }


    delay(10);

    recievedData = Serial1.readString();
    //recievedData = "0.1,1.2,3.3,4.5,1.234,10/12/18,9:22:18";
    if(debugging_enabled)
    {
        Serial.print("RECIEVED DATA FROM ESP: ");
        Serial.println(recievedData);
        writeLogFile("Recieved data from ESP");
    }
    //parse data if not null
    int comma_count = 0;
    int from_index = 0;
    int index_of_comma = 0;
    bool still_searching_for_commas = true;
    String stringArray[NUMBER_OF_FEILDS];

    while(still_searching_for_commas && comma_count < NUMBER_OF_FEILDS){
        //Serial.printf("From index: %d\n\r", from_index);

        index_of_comma = recievedData.indexOf(',', from_index);
        if(debugging_enabled){
          Serial.print("comma index: ");
          Serial.println(index_of_comma);
          //writeLogFile("got a comma");

        }

        //if the index of the comma is not zero, then there is data.
        if(index_of_comma > 0){
            stringArray[comma_count] = recievedData.substring(from_index, index_of_comma);
            if(debugging_enabled){
                Serial.printf("String[%d]:", comma_count);
                Serial.println(stringArray[comma_count]);
                //writeLogFile(stringArray[comma_count]);
            }
            comma_count++;
            from_index = index_of_comma;
            from_index += 1;
        }else{
            int index_of_cr = recievedData.indexOf('\r', from_index);
            if(index_of_cr > 0){
                stringArray[comma_count] = recievedData.substring(from_index, index_of_cr);
                if(debugging_enabled){
                    Serial.printf("String[%d]:", comma_count);
                    Serial.println(stringArray[comma_count]);
                }
            }
            still_searching_for_commas = false;
        }
    }
    if(comma_count == NUMBER_OF_FIELDS_LOGGING){
        ozone_value = stringArray[1].toFloat();
        if(debugging_enabled){
            Serial.println("using string array index 1 due to logging");
            //writeLogFile("using string array index 1 due to logging");
          }
    }else if(comma_count == (NUMBER_OF_FIELDS_LOGGING - 1)){
        ozone_value = stringArray[0].toFloat();
        if(debugging_enabled){
            Serial.println("using string array index 0, not logging");
            //writeLogFile("using string array index 0, not logging");
          }
    }
    return ozone_value;
    //parseOzoneString(recievedData);
}



/***start of all plantower functions***/

void outputParticles(){
    union{
	       double myDouble;
	       unsigned char bytes[sizeof(double)];
    } doubleBytes;
    //doubleBytes.myDouble = double;
    //

    //used for converting float to bytes for measurement value
    union {
        float myFloat;
        unsigned char bytes[4];
    } floatBytes;

    //used for converting word to bytes for lat and longitude
    union {
        int16_t myWord;
        unsigned char bytes[2];
    }wordBytes;

    while(!Serial.available()){
        // if (! bme.performReading()) {
        //   Serial.println("Failed to read BME680");

        // }
        // readPlantower();
        readGpsStream();
        // CO2_float = t6713.readPPM();
        CO2_float = readCO2();
        //correct for altitude
        // float pressure_correction = bme.pressure/100;
        float pressure_correction = tph_fusion.pressure->adj_value;
        if(pressure_correction > LOW_PRESSURE_LIMIT && pressure_correction < HIGH_PRESSURE_LIMIT){
            pressure_correction /= SEALEVELPRESSURE_HPA;
            CO2_float *= pressure_correction;
        }
        // pm_25_correction_factor = PM_25_CONSTANT_A + (PM_25_CONSTANT_B*(readHumidity()/100))/(1 - (readHumidity()/100));
        // corrected_PM_25 = PM2_5Value * pm_25_correction_factor;

        byte ble_output_array[NUMBER_OF_SPECIES*BLE_PAYLOAD_SIZE];     //19 bytes per data line and 12 species to output


        for(int i=0; i<5; i++){

            //************Fill the ble output array**********************//
            //Serial.printf("making array[%d]\n", i);
            //byte 0 - version
            ble_output_array[0 + i*(BLE_PAYLOAD_SIZE)] = 1;

            //bytes 1,2 - Device ID
            //DEVICE_id = 555;
            wordBytes.myWord = DEVICE_id;
            ble_output_array[1 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
            ble_output_array[2 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

            //byte 3 - Measurement number
            ble_output_array[3 + i*(BLE_PAYLOAD_SIZE)] = sample_counter;

            //byte 4 - Identifier (B:battery, a:Latitude, o:longitude,
            //t:Temperature, P:Pressure, h:humidity, s:Sound, O:Ozone,
            //C:CO2, M:CO, r:PM1, R:PM2.5, q:PM10, g:VOCs)
            /*
            0-battery
            1-PM01Value
            2-PM2_5Value
            3-PM10Value


            */

            if(i == 0){
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
                floatBytes.myFloat = fuel.getSoC();
            }else if(i == 1){
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
                // floatBytes.myFloat = PM01Value;
                floatBytes.myFloat = plantower.pm1.adj_value;
            }else if(i == 2){
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
                // floatBytes.myFloat = corrected_PM_25;
                floatBytes.myFloat = plantower.pm2_5.adj_value;
            }else if(i == 3){
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
                // floatBytes.myFloat = PM10Value;
                floatBytes.myFloat = plantower.pm10.adj_value;
            }else if(i == 4){
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_DIOXIDE_PACKET_CONSTANT;
                floatBytes.myFloat = CO2_float;
            }

            //bytes 5,6,7,8 - Measurement Value
            ble_output_array[5 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[0];
            ble_output_array[6 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[1];
            ble_output_array[7 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[2];
            ble_output_array[8 + i*(BLE_PAYLOAD_SIZE)] = floatBytes.bytes[3];


            //bytes 9-12 - latitude
            wordBytes.myWord = gps.get_latitudeWhole();
            ble_output_array[9 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
            ble_output_array[10 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

            wordBytes.myWord = gps.get_latitudeFrac();
            ble_output_array[11 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
            ble_output_array[12 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

            //bytes 14-17 - longitude
            wordBytes.myWord = gps.get_longitudeWhole();
            ble_output_array[13 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
            ble_output_array[14 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

            wordBytes.myWord = gps.get_longitudeFrac();
            ble_output_array[15 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
            ble_output_array[16 + i*(BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];


            //byte 18 - east west and north south indicator
            //  LSB 0 = East, LSB 1 = West
            //  MSB 0 = South, MSB 1 = North
            int northSouth = gps.get_nsIndicator();
            int eastWest = gps.get_ewIndicator();

            ble_output_array[17 + i*(BLE_PAYLOAD_SIZE)] = northSouth | eastWest;
            ble_output_array[18 + i*(BLE_PAYLOAD_SIZE)] = gps.get_horizontalDillution();

            ble_output_array[19 + i*(BLE_PAYLOAD_SIZE)] = '#';     //delimeter for separating species

        }

        //send start delimeter to ESP
        Serial1.print("$");
        //send the packaged data with # delimeters in between packets
        Serial1.write(ble_output_array, 5*BLE_PAYLOAD_SIZE);

        //send ending delimeter
        Serial1.print("&");
        sample_counter += 1;
    }
}

void goToSleep(void){
    //Serial.println("Going to sleep:)");
    digitalWrite(POWER_LED_EN, LOW);
    digitalWrite(PLANTOWER_EN, LOW);
    digitalWrite(ESP_WROOM_EN, LOW);
    digitalWrite(BLOWER_EN, LOW);
    digitalWrite(CO2_EN, LOW);
    digitalWrite(FIVE_VOLT_EN, LOW);
    enableLowPowerGPS();
    System.sleep(D4, FALLING, sleepInterval * 2);     //every 2 minutes wake up and check if battery voltage is too low
    System.reset();
}

void goToSleepBattery(void){
    digitalWrite(POWER_LED_EN, HIGH);   // Sets the LED on
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, HIGH);   // Sets the LED on
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, HIGH);   // Sets the LED on
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, HIGH);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, HIGH);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, HIGH);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(POWER_LED_EN, LOW);    // Sets the LED off

    //Serial.println("Turning off batfet");
    writeRegister(7, 0b01101011);   //turn off batfet
    //System.sleep(D4, RISING,sleepInterval * 2); //Put the Electron into Sleep Mode for 2 Mins + leave the Modem in Sleep Standby mode so when you wake up the modem is ready to send data vs a full reconnection process.

    /*digitalWrite(plantower_en, LOW);
    digitalWrite(esp_wroom_en, LOW);
    digitalWrite(blower_en, LOW);
    digitalWrite(co2_en, LOW);
    digitalWrite(fiveVolt_en, LOW);
    enableLowPowerGPS();

    Serial.print("Sleeping...  Battery charge level:");
    Serial.println(fuel.getSoC());
    System.sleep(D4, RISING, sleepInterval * 10);

    digitalWrite(plantower_en, HIGH);
    digitalWrite(esp_wroom_en, HIGH);
    digitalWrite(blower_en, HIGH);
    digitalWrite(co2_en, HIGH);
    digitalWrite(fiveVolt_en, HIGH);*/
    //System.sleep(SLEEP_MODE_DEEP, 600);

}

void resetESP(void){
  digitalWrite(ESP_WROOM_EN, LOW);
  digitalWrite(PLANTOWER_EN, LOW);
  digitalWrite(BLOWER_EN, LOW);
  digitalWrite(CO2_EN, LOW);
  delay(1000);
  digitalWrite(ESP_WROOM_EN, HIGH);
  digitalWrite(PLANTOWER_EN, HIGH);
  digitalWrite(BLOWER_EN, HIGH);
  digitalWrite(CO2_EN, HIGH);
  delay(1000);
}

/************************Serial menu stuff******************/

void serialMenu(){
  incomingByte = '0';
  while(incomingByte!= 'x')
  {
    Serial.print("Menu>");
    Serial.flush();
    while(!Serial.available());
    incomingByte = Serial.read();
    if(incomingByte == 'a'){
        serialGetCo2Slope();
    }else if(incomingByte == 'b'){
        serialGetCo2Zero();
    }else if(incomingByte == 'c'){
        serialGetCoSlope();
    }else if(incomingByte == 'd'){
        serialGetCoZero();
    }else if(incomingByte == 'e'){
        serialGetPm1Slope();
    }else if(incomingByte == 'f'){
         serialGetPm1Zero();
    }else if(incomingByte == 'g'){
        serialGetPm25Slope();
    }else if(incomingByte == 'h'){
        serialGetPm25Zero();
    }else if(incomingByte == 'i'){
        serialGetPm10Slope();
    }else if(incomingByte == 'j'){
        serialGetPm10Zero();
    }else if(incomingByte == 'k'){
        serialGetTemperatureSlope();
    }else if(incomingByte == 'l'){
        serialGetTemperatureZero();
    }else if(incomingByte == 'm'){
        serialGetPressureSlope();
    }else if(incomingByte == 'n'){
        serialGetPressureZero();
    }else if(incomingByte == 'o'){
        serialGetHumiditySlope();
    }else if(incomingByte == 'p'){
        serialGetHumidityZero();
    }else if(incomingByte == 'q'){
        Serial.println("Serial debugging enabled.");
        debugging_enabled = 1;
        EEPROM.put(DEBUGGING_ENABLED_MEM_ADDRESS, debugging_enabled);
    }else if(incomingByte == 'r'){
        Serial.println("Serial debugging disabled.");
        debugging_enabled = 0;
        EEPROM.put(DEBUGGING_ENABLED_MEM_ADDRESS, debugging_enabled);
    }else if(incomingByte == 's'){
        Serial.println(String(HEADER_STRING));
    }else if(incomingByte == 't'){
        serialGetTimeDate();
    }else if(incomingByte == 'u'){
        serialGetZone();
    }else if(incomingByte == 'v'){
        serialGetDeviceId();
    }else if(incomingByte == 'w'){
        serialGetWifiCredentials();
    }else if(incomingByte == 'y'){
        if(serial_cellular_enabled == 0){
            Serial.println("Enabling Cellular.");
        }else{
            Serial.println("Cellular already enabled.");
        }
        serial_cellular_enabled = 1;
        EEPROM.put(SERIAL_CELLULAR_EN_MEM_ADDRESS, serial_cellular_enabled);
    }else if(incomingByte == 'z'){
        if(serial_cellular_enabled == 1){
            Serial.println("Disabling Cellular");
            Cellular.off();
        }else{
            Serial.println("Cellular already disabled.");
        }
        serial_cellular_enabled = 0;
        EEPROM.put(SERIAL_CELLULAR_EN_MEM_ADDRESS, serial_cellular_enabled);
    }else if(incomingByte == 'F'){
        if(temperature_units == CELCIUS){
            temperature_units = FARENHEIT;

        }else{
            Serial.println("Temperature units already set to Fareneit.");
        }

        EEPROM.put(TEMPERATURE_UNITS_MEM_ADDRESS, temperature_units);

    }else if(incomingByte == 'C'){
        if(temperature_units == FARENHEIT){
            temperature_units = CELCIUS;

        }else{
            Serial.println("Temperature units already set to Celcius.");
        }

        EEPROM.put(TEMPERATURE_UNITS_MEM_ADDRESS, temperature_units);
    }else if(incomingByte == 'D'){
        if(new_temperature_sensor_enabled == 1){
            new_temperature_sensor_enabled = 0;
            Serial.println("Disabling new temperature sensor");
        }else{

            Serial.println("Temperature sensor already disabled");
        }
        EEPROM.put(TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS, new_temperature_sensor_enabled);

    }else if(incomingByte == 'E'){
        if(new_temperature_sensor_enabled == 1){
            Serial.println("Temperature sensor already enabled");
        }else{
            new_temperature_sensor_enabled = 1;
            Serial.println("Temperatue sensor now enabled");
        }
        EEPROM.put(TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS, new_temperature_sensor_enabled );

    }else if(incomingByte == 'G'){      //enable analog reading of ozone and disable esp reading of ozone
        if(ozone_analog_enabled == 1){
            Serial.println("Analog reading of ozone already enabled");
        }else{
            ozone_analog_enabled = 1;
            Serial.println("Analog reading of ozone now enabled");
        }
        EEPROM.put(OZONE_A_OR_D_MEM_ADDRESS, ozone_analog_enabled);

    }else if(incomingByte == 'H'){      //disable analog reading of ozone and read from esp
        if(ozone_analog_enabled == 0){
            Serial.println("Digital reading of ozone already enabled");
        }else{
            ozone_analog_enabled = 0;
            Serial.println("Digital reading of ozone now enabled");
        }
        EEPROM.put(OZONE_A_OR_D_MEM_ADDRESS, ozone_analog_enabled);

    }else if(incomingByte == 'I'){      //disable analog reading of ozone and read from esp
        serialGetAverageTime();
    }else if(incomingByte == 'J'){
        resetESP();
        Serial.println("ESP reset!");
    }else if(incomingByte == 'K'){
      Serial.println("Outputting GPS continuously");
      echoGps();
    }else if(incomingByte == 'L'){
      serialResetSettings();
    }else if(incomingByte == 'M'){
      //serialTestRemoteFunction();
      if(battery_threshold_enable == 1){
          Serial.println("Battery threshold already enabled");
      }else{
          Serial.println("Enabling battery threshold limiting");
          battery_threshold_enable = 1;
          EEPROM.put(BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS, battery_threshold_enable);
      }

    }else if(incomingByte == 'N'){
      //serialTestRemoteFunction();
      if(battery_threshold_enable == 0){
          Serial.println("Battery threshold already disabled");
      }else{
          Serial.println("Disabling battery threshold limiting");
          battery_threshold_enable = 0;
          EEPROM.put(BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS, battery_threshold_enable);
      }

    }else if(incomingByte == 'O'){
        //Serial.println("Changing frequency for gps");
        //changeFrequency();
        Serial.println("Enabling low power for gps");
        enableLowPowerGPS();
    }else if(incomingByte  == 'P'){
        Serial.println("Turning off batfet");
        writeRegister(7, 0b01101011);   //turn off batfet
    }else if(incomingByte == 'Q'){

        Serial.println("Allowing batfet to turn on");
        writeRegister(7, 0b01001011);   //allow batfet to turn on
    }else if(incomingByte == 'R'){
        if(abc_logic_enabled){
            Serial.println("Disabling ABC logic for CO2 sensor");
            abc_logic_enabled = 0;
            EEPROM.put(ABC_ENABLE_MEM_ADDRESS, abc_logic_enabled);
            // t6713.disableABCLogic();
            t6713.t6713.disableABCLogic();
        }else{
            Serial.println("ABC logic already disabled");
        }

    }else if(incomingByte == 'S'){
        if(!abc_logic_enabled){
            Serial.println("Enabling abc logic for CO2 sensor");
            abc_logic_enabled = 1;
            EEPROM.put(ABC_ENABLE_MEM_ADDRESS, abc_logic_enabled);
            // t6713.enableABCLogic();
            t6713.t6713.enableABCLogic();

        }else{
            Serial.println("ABC logic already enabled");
        }
    }else if(incomingByte == 'T'){
        if (!hih8120_enabled) {
            Serial.println("Enabling HIH8120 RH sensor");
            hih8120_enabled = 1;
            tph_fusion.enable_hih();
            EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, hih8120_enabled);

        } else {
            Serial.println("Disabling HIH8120 RH sensor");
            hih8120_enabled = 0;
            tph_fusion.disable_hih();
            EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, hih8120_enabled);
        }

    }else if(incomingByte == 'U'){
        if(!CO_socket){
            Serial.println("Now reading CO from U20-Alpha2");
            CO_socket = 1;
            EEPROM.put(CO_SOCKET_MEM_ADDRESS, CO_socket);

        }else{
            Serial.println("Now reading CO from U19-Alpha1");
            CO_socket = 0;
            EEPROM.put(CO_SOCKET_MEM_ADDRESS, CO_socket);
        }
    }else if(incomingByte == 'V'){
        Serial.println("Reseting the CO2 sensor");
        // t6713.resetSensor();
        t6713.t6713.resetSensor();
    }else if(incomingByte == '1'){
        serialGetLowerLimit();
    }else if(incomingByte == '2'){
        serialGetUpperLimit();
    }else if(incomingByte == '3'){
        Serial.print("APP Version: ");
        Serial.println(APP_VERSION);
        Serial.print("Build: ");
        Serial.println(BUILD_VERSION);
    }else if(incomingByte == '4'){
        if(ozone_enabled == 0){
            Serial.println("Enabling Ozone");
        }else{
            Serial.println("Ozone already enabled");
        }
        ozone_enabled = 1;
        EEPROM.put(OZONE_EN_MEM_ADDRESS, ozone_enabled);
    }else if(incomingByte == '5'){
        if(ozone_enabled == 1){
            Serial.println("Disabling Ozone");
        }else{
            Serial.println("Ozone already disabled");
        }
        ozone_enabled = 0;
        EEPROM.put(OZONE_EN_MEM_ADDRESS, ozone_enabled);
    }else if(incomingByte == '6'){
        if(voc_enabled == 0){
            Serial.println("Enabling VOC's");
        }else{
            Serial.println("VOC's already enabled");
        }
        voc_enabled = 1;
        EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
    }else if(incomingByte == '7'){
        if(voc_enabled == 1){
            Serial.println("Disabling VOC's");
        }else{
            Serial.println("VOC's already disabled");
        }
        voc_enabled = 0;
        EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
    }else if(incomingByte == '8'){
        Serial.print("Fault: ");
        byte fault = pmic.getFault();
        Serial.println(fault);
        Serial.print("System status: ");
        byte systemStatus = pmic.getSystemStatus();
        Serial.println(systemStatus);

    }else if(incomingByte == '9'){
        serialIncreaseChargeCurrent();
    }else if(incomingByte == '0'){
        serialIncreaseInputCurrent();
    }else if(incomingByte == 'A'){
        readAlpha1Constantly();
    }else if(incomingByte == 'B'){
        if(output_only_particles == 1){
            output_only_particles = 0;
            Serial.println("Outputting normally");
        }else{
            output_only_particles = 1;
            Serial.println("Outputting only PM");
        }
        EEPROM.put(OUTPUT_PARTICLES_MEM_ADDRESS, output_only_particles);

    }else if(incomingByte == '!'){

        Serial.println("Outputting VOCs continuously!  Press any button to exit...");
        while(!Serial.available()){
            // if (! bme.performReading()) {
            if (! tph_fusion.measure()) {
              Serial.println("Failed to read BME680");
              return;
            }else{
                // Serial.printf("TVocs=%1.0f, Temp=%1.1f, press=%1.1f, rh=%1.1f\n\r", bme.gas_resistance/100, bme.temperature, bme.pressure, bme.humidity);
                Serial.printf("TVocs=%1.0f, Temp=%1.1f, press=%1.1f, rh=%1.1f\n\r", tph_fusion.voc->adj_value, tph_fusion.temperature->adj_value, tph_fusion.pressure->adj_value, tph_fusion.humidity->adj_value);
            }
        }
    }else if(incomingByte == 'W'){
        if(google_location_en == 1){
            Serial.println("Disabling google location services.");
            google_location_en = 0;
            EEPROM.put(GOOGLE_LOCATION_MEM_ADDRESS, google_location_en);
        }else{
            Serial.println("Enabling google location services.");
            google_location_en = 1;
            EEPROM.put(GOOGLE_LOCATION_MEM_ADDRESS, google_location_en);
        }
        
    }else if(incomingByte == 'X'){
        //calibrate CO2 sensor
        //if(debugging_enabled){
            // t6713.calibrate(1);
            t6713.t6713.calibrate(1);
        //}else{
         //   t6713.calibrate(0);
        //}
        
        co2_calibration_timer = 180;        //6 minutes if measurement cycle is 2 seconds
        
    
    }else if(incomingByte == 'Z'){
        Serial.println("Getting cellular information, this may take a while...");

        Log.info("IMEI=%s", CellularHelper.getIMEI().c_str());

        Log.info("IMSI=%s", CellularHelper.getIMSI().c_str());

        Log.info("ICCID=%s", CellularHelper.getICCID().c_str());
        //if(serial_cellular_enabled){

        //}else{
        //    Serial.println("Cellular not enabled.  Please enable cellular first!");
        //}
    }else if(incomingByte == '?'){
        outputSerialMenuOptions();
    }
  }
  Serial.println("Exiting serial menu...");

}

// void serialTestRemoteFunction(void){

//   Serial.println("Enter string (address,value)");
//   Serial.setTimeout(50000);
//   String tempString = Serial.readStringUntil('\r');
//   int response = remoteWriteStoredVars(tempString);
//   if(response){
//     Serial.println("sucess in writing");
//   }else{
//     Serial.println("failed writing string");
//   }

// }

void serialIncreaseInputCurrent(void){
    int inputCurrent = pmic.getInputCurrentLimit();
    Serial.printf("Old input current limit: %d\n\r", inputCurrent);

    if(inputCurrent == 100){
        inputCurrent = 150;
    }else if(inputCurrent == 100){
        inputCurrent = 150;
    }else if(inputCurrent == 150){
        inputCurrent = 500;
    }else if(inputCurrent == 500){
        inputCurrent = 900;
    }else if(inputCurrent == 900){
        inputCurrent = 1200;
    }else if(inputCurrent == 1200){
        inputCurrent = 1500;
    }else if(inputCurrent == 1500){
        inputCurrent = 2000;
    }else if(inputCurrent == 2000){
        inputCurrent = 3000;
    }
    //delay(2000);
    pmic.setInputCurrentLimit(inputCurrent);
    Serial.printf("New input current limit: %d\n\r", inputCurrent);
}

void serialIncreaseChargeCurrent(void){
    int total_current = 0;
    bool bit7 = 0;
    bool bit6 = 0;
    bool bit5 = 0;
    bool bit4 = 0;
    bool bit3 = 0;
    bool bit2 = 0;

    byte chargeCurrent = pmic.getChargeCurrent();
    //bit 7
    if(chargeCurrent & 0x80){
        total_current += 2048;
    }
    //bit 6
    if(chargeCurrent & 0x40){
        total_current += 1024;
    }
    //bit 5
    if(chargeCurrent & 0x20){
        total_current += 512;
    }
    //bit 4
    if(chargeCurrent & 0x10){
        total_current += 256;
    }
    //bit 3
    if(chargeCurrent & 0x08){
        total_current += 128;
    }
    //bit 2
    if(chargeCurrent & 0x04){
        total_current += 64;
    }
    Serial.printf("Increasing Charge current from %d mA to ", total_current);
    chargeCurrent += 4;
    total_current = 0;

    if(chargeCurrent & 0x80){
        total_current += 2048;
        bit7 = 1;
    }
    //bit 6
    if(chargeCurrent & 0x40){
        total_current += 1024;
        bit6 = 1;
    }
    //bit 5
    if(chargeCurrent & 0x20){
        total_current += 512;
        bit5 = 1;
    }
    //bit 4
    if(chargeCurrent & 0x10){
        total_current += 256;
        bit4 = 1;
    }
    //bit 3
    if(chargeCurrent & 0x08){
        total_current += 128;
        bit3 = 1;
    }
    //bit 2
    if(chargeCurrent & 0x04){
        total_current += 64;
        bit2 = 1;
    }

    pmic.setChargeCurrent(bit7, bit6, bit5, bit4, bit3, bit2);
    chargeCurrent = pmic.getChargeCurrent();
    Serial.printf("new charge current of %d mA\n\r", total_current);
}

void serialGetWifiCredentials(void){
    Serial.print("Current stored ssid: ");
    Serial.println(ssid);
    Serial.print("Current stored password: ");
    Serial.println(password);
    Serial.println("Please enter password in order to make changes.\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    if(tempString.equals("bould")){
        Serial.println("Password correct!");
        Serial.println("Enter new ssid:");
        Serial.setTimeout(50000);
        String tempSsid = Serial.readStringUntil('\r');
        Serial.print("Your new ssid will be: ");
        Serial.println(tempSsid);
        Serial.println("Is this okay?(y or n)");
        String ok = Serial.readStringUntil('\r');
        if(ok.equals("y")){
            Serial.println("Saving new ssid");
            ssid = tempSsid;
            Serial.println("Enter new password");
            String tempPassword = Serial.readStringUntil('\r');
            Serial.print("Your new password will be: ");
            Serial.println(tempPassword);
            String ok = Serial.readStringUntil('\r');
            if(ok.equals("y")){
                Serial.println("Saving new password");
                password = tempPassword;
                sendWifiInfo();
            }else{
                Serial.println("okay, no problem\n\r");
            }
        }else{
            Serial.println("okay, no problem\n\r");
            return;
        }
    }
}

void serialGetDeviceId(void){

    Serial.println();
    Serial.print("Current Device ID:");
    Serial.println(DEVICE_id);
    Serial.println("Please enter password in order to change the ID");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');


    if(tempString == "bould"){
        Serial.println("Password correct!");
        Serial.println("Enter new Device ID:");
        String tempString = Serial.readStringUntil('\r');
        int tempValue = tempString.toInt();
        Serial.println("");
        if(tempValue > MIN_DEVICE_ID_NUMBER && tempValue < MAX_DEVICE_ID_NUMBER){
            Serial.print("\n\rNew Device ID:");
            Serial.println(tempValue);
            DEVICE_id = tempValue;
            EEPROM.put(DEVICE_ID_MEM_ADDRESS, DEVICE_id);
        }else{
            Serial.println("\n\rInvalid value!");
        }
    }else{
        Serial.println("\n\rIncorrect password!");
    }
}

void serialResetSettings(void){

    Serial.println();
    Serial.println("Please enter password in order to apply default settings");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');


    if(tempString == "bould"){
        Serial.println("Password correct, resetting all settings to default!  Please reset your ID to the one shown on your enclosure.");
        writeDefaultSettings();
    }else{
        Serial.println("\n\rIncorrect password!");
    }
}

void serialGetTimeDate(void){
    Serial.println("Enter new Device time and date (10 digit epoch timestamp):");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();
    Serial.println("");
    if(tempValue > 966012661 && tempValue < 4121686261){       //min is the year 2000, max is the year 2100
        Time.setTime(tempValue);
        Serial.print("\n\rNew Device Time:");
        Serial.println(Time.timeStr());
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetZone(void){
    Serial.println("Enter new Device time zone (-12.0 to 14.0)");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();
    Serial.println("");
    if(tempValue >= -12 && tempValue <= 14){       //min is the year 2000, max is the year 2100
        Time.zone(tempValue);
        Serial.print("\n\rNew Device time zone:");
        Serial.println(tempValue);
        EEPROM.put(TIME_ZONE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetAverageTime(void){
    Serial.println();
    Serial.print("Current Frequency: ");
    Serial.print(measurements_to_average);
    Serial.println("(~2 second) measurements");
    Serial.print("Enter new amount\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= 1 && tempValue < 10000){
        Serial.print("\n\rNew Frequency: ");
        Serial.println(tempValue);
        Serial.println("(~2 second) measurements");
        measurements_to_average = tempValue;
        EEPROM.put(MEASUREMENTS_TO_AVG_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCo2Slope(void){

    Serial.println();
    Serial.print("Current CO2 slope:");
    Serial.print(String(t6713.CO2.slope, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO2 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 10.0){
        t6713.CO2.slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew CO2 slope: ");
        Serial.println(String(t6713.CO2.slope,2));

        EEPROM.put(CO2_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCo2Zero(void){
    Serial.println();
    Serial.print("Current CO2 zero:");
    Serial.print(t6713.CO2.zero);
    Serial.println(" ppm");
    Serial.print("Enter new CO2 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew CO2 zero: ");
        Serial.println(tempValue);
        t6713.CO2.zero = tempValue;
        EEPROM.put(CO2_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoSlope(void){

    Serial.println();
    Serial.print("Current CO slope:");
    Serial.print(String(pamco.co.slope, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.1 && tempfloat < 2.0){
        pamco.co.slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew CO slope: ");
        Serial.println(String(pamco.co.slope,2));

        EEPROM.put(CO_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoZero(void){
    Serial.println();
    Serial.print("Current CO zero:");
    Serial.print(pamco.co.zero);
    Serial.println(" ppb");
    Serial.print("Enter new CO Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -5000 && tempValue < 5000){
        Serial.print("\n\rNew CO zero: ");
        Serial.println(tempValue);
        pamco.co.zero = tempValue;
        EEPROM.put(CO_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm1Slope(void){
    Serial.println();
    Serial.print("Current PM1 slope:");
    Serial.print(String(plantower.pm1.slope, 2));
    Serial.println(" ");
    Serial.print("Enter new PM1 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        plantower.pm1.slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew PM1 slope: ");
        Serial.println(String(plantower.pm1.slope, 2));

        EEPROM.put(PM_1_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm1Zero(void){
    Serial.println();
    Serial.print("Current PM1 zero:");
    Serial.print(plantower.pm1.zero);
    Serial.println(" ug/m3");
    Serial.print("Enter new PM1 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew PM1 zero: ");
        Serial.println(tempValue);
        plantower.pm1.zero = tempValue;
        EEPROM.put(PM_1_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm25Slope(void){
    Serial.println();
    Serial.print("Current PM2.5 slope:");
    Serial.print(String(plantower.pm2_5.slope, 2));
    Serial.println(" ");
    Serial.print("Enter new PM2.5 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        plantower.pm2_5.slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew PM2.5 slope: ");
        Serial.println(String(plantower.pm2_5.slope,2));

        EEPROM.put(PM_25_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm25Zero(void){
    Serial.println();
    Serial.print("Current PM2.5 zero:");
    Serial.print(plantower.pm2_5.zero);
    Serial.println(" ug/m3");
    Serial.print("Enter new PM2.5 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew PM2.5 zero: ");
        Serial.println(tempValue);
        plantower.pm2_5.zero = tempValue;
        EEPROM.put(PM_25_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm10Slope(void){
    Serial.println();
    Serial.print("Current PM10 slope:");
    Serial.print(String(plantower.pm10.slope, 2));
    Serial.println(" ");
    Serial.print("Enter new PM10 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        plantower.pm10.slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew PM10 slope: ");
        Serial.println(String(plantower.pm10.slope,2));

        EEPROM.put(PM_10_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm10Zero(void){
    Serial.println();
    Serial.print("Current PM10 zero:");
    Serial.print(plantower.pm10.zero);
    Serial.println(" um/m3");
    Serial.print("Enter new PM10 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew PM10 zero: ");
        Serial.println(tempValue);
        plantower.pm10.zero = tempValue;
        EEPROM.put(PM_10_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetTemperatureSlope(void){
    Serial.println();
    Serial.print("Current Temperature slope:");
    Serial.print(String(tph_fusion.temperature->slope, 2));
    Serial.println(" Degrees C");
    Serial.print("Enter new Temperature slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        tph_fusion.temperature->slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew Temperature slope: ");
        Serial.println(String(tph_fusion.temperature->slope,2));

        EEPROM.put(TEMP_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetTemperatureZero(void){
    Serial.println();
    Serial.print("Current Temperature zero:");
    Serial.print(tph_fusion.temperature->zero);
    Serial.println(" Degrees C");
    Serial.print("Enter new Temperature Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -30 && tempValue < 30){
        Serial.print("\n\rNew Temperature zero: ");
        Serial.println(tempValue);
        tph_fusion.temperature->zero = tempValue;
        EEPROM.put(TEMP_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPressureSlope(void){
    Serial.println();
    Serial.print("Current Pressure slope:");
    Serial.print(String(tph_fusion.pressure->slope, 2));
    Serial.println(" torr");
    Serial.print("Enter new Pressure slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        tph_fusion.pressure->slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew Pressure slope: ");
        Serial.println(String(tph_fusion.pressure->slope,2));

        EEPROM.put(PRESSURE_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPressureZero(void){
    Serial.println();
    Serial.print("Current Pressure zero:");
    Serial.print(tph_fusion.pressure->zero);
    Serial.println(" ppm");
    Serial.print("Enter new Pressure Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew Pressure zero: ");
        Serial.println(tempValue);
        tph_fusion.pressure->zero = tempValue;
        EEPROM.put(PRESSURE_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetHumiditySlope(void){
    Serial.println();
    Serial.print("Current RH slope:");
    Serial.print(String(tph_fusion.humidity->slope, 2));
    Serial.println(" %");
    Serial.print("Enter new RH slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 10){
        tph_fusion.humidity->slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew RH slope: ");
        Serial.println(String(tph_fusion.humidity->slope,2));

        EEPROM.put(RH_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetHumidityZero(void){
    Serial.println();
    Serial.print("Current RH zero:");
    Serial.print(tph_fusion.humidity->zero);
    Serial.println(" %");
    Serial.print("Enter new RH Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -50 && tempValue < 50){
        Serial.print("\n\rNew RH zero: ");
        Serial.println(tempValue);
        tph_fusion.humidity->zero = tempValue;
        EEPROM.put(RH_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetOzoneOffset(void){
    Serial.println();
    Serial.print("Current O3 analog offset:");
    Serial.print(ozone_offset);
    Serial.println(" ppb");
    Serial.print("Enter new ozone offset\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -50 && tempValue < 50){
        Serial.print("\n\rNew ozone offset: ");
        Serial.println(tempValue);
        ozone_offset = tempValue;
        EEPROM.put(OZONE_OFFSET_MEM_ADDRESS, ozone_offset);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetLowerLimit(void){
    Serial.println();
    Serial.print("Current lower limit:");
    Serial.println(gas_lower_limit);
    Serial.println("Please enter password in order to change the lower limit");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');


    if(tempString == "bould"){
        Serial.println("Password correct!");
        Serial.println("Enter new lower limit:\n\r");
        String tempString = Serial.readStringUntil('\r');
        int tempValue = tempString.toInt();
        Serial.println("");
        if(tempValue > 0 && tempValue < 20000){
            Serial.print("\n\rNew lower limit:");
            Serial.println(tempValue);
            gas_lower_limit = tempValue;
            EEPROM.put(GAS_LOWER_LIMIT_MEM_ADDRESS, gas_lower_limit);
        }else{
            Serial.println("\n\rInvalid value!");
        }
    }else{
        Serial.println("\n\rIncorrect password!");
    }
}
void serialGetUpperLimit(void){
    Serial.println();
    Serial.print("Current upper limit:");
    Serial.println(gas_upper_limit);
    Serial.println("Please enter password in order to change the upper limit");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');


    if(tempString == "bould"){
        Serial.println("Password correct!");
        Serial.println("Enter new upper limit:\n\r");
        String tempString = Serial.readStringUntil('\r');
        int tempValue = tempString.toInt();
        Serial.println("");
        if(tempValue > 0 && tempValue < 50000){
            Serial.print("\n\rNew upper limit:");
            Serial.println(tempValue);
            gas_upper_limit = tempValue;
            EEPROM.put(GAS_UPPER_LIMIT_MEM_ADDRESS, gas_upper_limit);
        }else{
            Serial.println("\n\rInvalid value!");
        }
    }else{
        Serial.println("\n\rIncorrect password!");
    }
}

void readAlpha1Constantly(void){
    while(!Serial.available()) {
        // CO_float = readCO();
        float value = pamco.co.adj_value;
        Serial.printf("CO: %1.3f ppm\n\r", value);
    }
}
void outputSerialMenuOptions(void){
    Serial.println("Command:  Description");
    Serial.println("a:  Adjust CO2 slope");
    Serial.println("b:  Adjust CO2 zero");
    Serial.println("c:  Adjust CO slope");
    Serial.println("d:  Adjust CO zero");
    Serial.println("e:  Adjust PM1 slope");
    Serial.println("f:  Adjust PM1 zero");
    Serial.println("g:  Adjust PM2.5 slope");
    Serial.println("h:  Adjust PM2.5 zero");
    Serial.println("i:  Adjust PM10 slope");
    Serial.println("j:  Adjust PM10 zero");
    Serial.println("k:  Adjust Temperature slope");
    Serial.println("l:  Adjust Temperature zero");
    Serial.println("m:  Adjust Pressure slope");
    Serial.println("n:  Adjust Pressure zero");
    Serial.println("o:  Adjust Humidity slope");
    Serial.println("p:  Adjust Humidity zero");
    Serial.println("q:  Enable serial debugging");
    Serial.println("r:  Disable serial debugging");
    Serial.println("s:  Output header string");
    Serial.println("t:  Enter new time and date");
    Serial.println("u:  Enter new time zone");
    Serial.println("v:  Adjust the Device ID");
    Serial.println("w:  Get wifi credentials");
    Serial.println("y:  Enable cellular");
    Serial.println("z:  Disable cellular");
    Serial.println("1:  Adjust gas lower limit");
    Serial.println("2:  Adjust gas upper limit");
    Serial.println("3:  Get build version");
    Serial.println("4:  Enable Ozone");
    Serial.println("5:  Disable Ozone");
    Serial.println("6:  Enable VOC's");
    Serial.println("7:  Disable VOC's");
    Serial.println("8:  Output the PMIC system configuration");
    Serial.println("9:  Increase the charge current by 64 mA");
    Serial.println("0:  Increase the current input limit by 100 mA");
    Serial.println("A:  Ouptput CO constantly and rapidly");
    Serial.println("B:  Output PM constantly and rapidly");
    Serial.println("C:  Change temperature units to Celcius");
    Serial.println("D:  Disable TMP36 temperature sensor and use BME680 temperature");
    Serial.println("E:  Enable TMP36 temperature sensor and disable BME680 temperature");
    Serial.println("F:  Change temperature units to Farenheit");
    Serial.println("G:  Read ozone from analog input (not digitally - board dependent)");
    Serial.println("H:  Read ozone digitally (not through analog input - board dependent)");
    Serial.println("I:  Adjust frequency for uploading through cellular");
    Serial.println("J:  Reset ESP, CO2, Plantower");
    Serial.println("K:  Continuous serial output of GPS");
    Serial.println("L:  Write default settings");
    Serial.println("M:  Enable 20% battery threshold limiting");
    Serial.println("N:  Disable 20% battery threshold limiting WARNING!!");
    Serial.println("O:  Enable low power for GPS module");
    Serial.println("P:  Turn off BATFET");
    Serial.println("Q:  Allow BATFET to turn on");
    Serial.println("R:  Disable ABC logic for CO2 sensor");
    Serial.println("S:  Enable ABC logic for CO2 sensor");
    Serial.println("T:  Enable/disable HIH8120 RH sensor");
    Serial.println("U:  Switch socket where CO is read from");
    Serial.println("W:  Enable/Disable google location services");
    Serial.println("X:  Calibrate CO2 sensor - must supply ambient level (go outside!)");
    Serial.println("Z:  Output cellular information (CCID, IMEI, etc)");
    Serial.println("!:  Continuous serial output of VOC's");
    Serial.println("?:  Output this menu");
    Serial.println("x:  Exits this menu");
  }
