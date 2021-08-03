#include <string>
//#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Telaire_T6713.h"
#include "Adafruit_ADS1X15.h"
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
#include "vector"

// THIS IS SO WE GET A LARGER SERIAL BUFFER
#include "SerialBufferRK.h"

PRODUCT_ID(15083);
PRODUCT_VERSION(1);

bool haveOfflineData = false;
String diagnosticData = "";

#define SERIAL_PASSWORD "bould"

GoogleMapsDeviceLocator locator;

#define APP_VERSION 70
#define BUILD_VERSION 13

//define constants
#define SEALEVELPRESSURE_HPA (1013.25)
#define LOW_PRESSURE_LIMIT (100)
#define HIGH_PRESSURE_LIMIT (1500)
#define VOLTS_PER_UNIT (0.0008)   //3.3V/4096  3.3 is the adc reference voltage and the adc is 12 bits or 4096
#define VOLTS_PER_PPB (0.0125)  //2.5V/200 ppb this is what you divide the voltage reading by to get ppb in ozone if the ozone monitor is set to 2.5V=200ppb
#define PM_25_CONSTANT_A (1.19)
#define PM_25_CONSTANT_B (0.119)
#define CELSIUS 1
#define FAHRENHEIT 0
#define TMP36_OFFSET 0.5
#define TMP36_VPDC 0.01 //10mV per degree C

//google maps API key:
#define GOOGLE_API_KEY "AIzaSyAfgY0VX3KSMkVoIVvWAr9oVlT-AoQ68e0"

float ads_bitmv = 0.1875; //Bits per mV at defined bit resolution, used to convert ADC value to voltage
#define ALPHA_ADC_READ_AMOUNT 10
//float ads_bitmv = 0.1920;

//enable or disable different parts of the firmware by setting the following values to 1 or 0
#define sd_en 1

//enable AFE 2 (just for testing now using a second CO sensor connected to it)
#define AFE2_en 0

//define addresses of eeprom stored variables
#define DEVICE_ID_MEM_ADDRESS 0
#define CO2_ZERO_MEM_ADDRESS 4
#define CO2_SLOPE_MEM_ADDRESS 8
#define CO_ZERO_A_MEM_ADDRESS 12
#define CO_SLOPE_A_MEM_ADDRESS 16
#define PM_1_ZERO_MEM_ADDRESS 20
#define PM_1_SLOPE_MEM_ADDRESS 24
#define PM_25_ZERO_MEM_ADDRESS 28
#define PM_25_SLOPE_MEM_ADDRESS 32
#define PM_10_ZERO_MEM_ADDRESS 36
#define PM_10_SLOPE_MEM_ADDRESS 40
#define TEMP_ZERO_MEM_ADDRESS 44
#define TEMP_SLOPE_MEM_ADDRESS 48
#define PRESSURE_ZERO_MEM_ADDRESS 52
#define PRESSURE_SLOPE_MEM_ADDRESS 56
#define RH_ZERO_MEM_ADDRESS 60
#define RH_SLOPE_MEM_ADDRESS 64
#define SERIAL_CELLULAR_EN_MEM_ADDRESS 68
#define DEBUGGING_ENABLED_MEM_ADDRESS  72
#define GAS_LOWER_LIMIT_MEM_ADDRESS 76
#define GAS_UPPER_LIMIT_MEM_ADDRESS 80
#define TIME_ZONE_MEM_ADDRESS 84
#define OZONE_EN_MEM_ADDRESS 88
#define VOC_EN_MEM_ADDRESS 92
#define TEMPERATURE_UNITS_MEM_ADDRESS 96
#define OUTPUT_PARTICLES_MEM_ADDRESS 100
#define TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS 104
#define OZONE_A_OR_D_MEM_ADDRESS 108
#define OZONE_OFFSET_MEM_ADDRESS 112
#define MEASUREMENTS_TO_AVG_MEM_ADDRESS 116
#define BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS 120
#define ABC_ENABLE_MEM_ADDRESS 124
#define HIH8120_ENABLE_MEM_ADDRESS 128
#define CO_SOCKET_MEM_ADDRESS 132
#define GOOGLE_LOCATION_MEM_ADDRESS 136
#define SENSIBLE_IOT_ENABLE_MEM_ADDRESS 140
#define CAR_TOPPER_POWER_MEM_ADDRESS 144
#define CO_ZERO_B_MEM_ADDRESS 148
#define CO_SLOPE_B_MEM_ADDRESS 152
#define MAX_MEM_ADDRESS 152

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

#define NUMBER_OF_SPECIES 11        //total number of species (measurements) being output

#define MAX_COUNTER_INDEX 15000

#define BATTERY_THRESHOLD 20        //if battery is below 20 percent, go into sleep mode

//define pin functions
//fix these so they are more consistent!
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define CS A2               //Chip select for SPI/uSD card
#define SLEEP_EN D3

//constants for parsing ozone string from ozone monitor Model 106
#define NUMBER_OF_FIELDS 7
#define NUMBER_OF_FIELDS_LOGGING 7

#define SERIAL_MENU_TIMEOUT 500000 //In system ticks


int lmp91000_1_en = B0;         //enable line for the lmp91000 AFE chip for measuring CO
int lmp91000_2_en = B2;
int fiveVolt_en = D5;
int serial4Enabler = B4;
int power_led_en = D6;
int kill_power = WKP;
int esp_wroom_en = D7;
int blower_en = D2;
int sound_input = B5;       //ozone monitor's voltage output is connected to this input
int co2_en = C5;            //enables the CO2 sensor power

std::allocator<String> alloc;
std::vector<String> diagnostics;

SerialBuffer<4096> serBuf(Serial4); // This is how we setup getting a bigger buffer for Serial4

//manually control connection to cellular network
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

//global objects
Adafruit_BME680 bme; // I2C
Telaire_T6713 t6713;  //CO2 sensor
LMP91000 lmp91000_1;
LMP91000 lmp91000_2;
Adafruit_ADS1115 ads1(0x49); //Set I2C address of ADC1
Adafruit_ADS1115 ads2(0x4A); //Set I2C address of ADC2
FuelGauge fuel;
GPS gps;
PMIC pmic;
PowerCheck powerCheck;
time_t systemTime;
//SerialLogHandler logHandler;
HIH61XX hih(0x27);
unsigned long lastCheck = 0;
char lastStatus[256];

//sdcard
SdFat sd;
SdFile file;
SdFile log_file;
File file1;
String fileName;

// These go together so I can keep track of how many lines i've put into the sd card. This is so I can push to particle data that
//the PAM recieved when it didn't have cell service.
String dataFileName;
long numberOfLines = 0;
bool sending_offline = false;


// String diagnosticFileName;
String logFileName;
int file_started = 0;
int log_file_started = 0;

//wifi
String ssid;        //wifi network name
String password;    //wifi network password

//global variables
int counter = 0;
float CO_float_A = 0;
float CO_float_B = 0;
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
int serial_cellular_enabled = 1;
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
int sensible_iot_en = 0;
int car_topper_power_en = 0;
double measurement_number = 0;

char geolocation_latitude[12] = "999.9999999";
char geolocation_longitude[13] = "99.9999999";
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
float CO2_slope;
int CO2_zero;
float CO_slopeA;
float CO_zeroA;
float CO_slopeB;
float CO_zeroB;
float PM_1_slope;
float PM_25_slope;
float PM_10_slope;
int PM_1_zero;
int PM_25_zero;
int PM_10_zero;
int ozone_offset;
float temp_slope;
int temp_zero;
float pressure_slope;
int pressure_zero;
float rh_slope;
int rh_zero;
int gas_lower_limit = 1200;         // Bad air quality limit
int gas_upper_limit = 50000;        // Good air quality limit
float pm_25_correction_factor;      //based on rh, this corrects pm2.5 according to Zheng et. al 2018
int measurements_to_average = 0;
int co2_calibration_timer = 0;

int sleepInterval = 60;  // This is used below for sleep times and is equal to 60 seconds of time.

//serial menu variables
int addr;
uint16_t value;
char receiveStr[5];

//plantower PMS5003 vars
int PM01Value = 0;
int PM2_5Value = 0;
int PM10Value = 0;
float corrected_PM_25 = 0;
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes, length of buffer sent from PMS1003 Particulate Matter sensor
char buf[LENG]; //Serial buffer sent from PMS1003 Particulate Matter sensor
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

//function declarations
void readStoredVars(void);
void checkCalFile(void);
size_t readField(File* file, char* str, size_t size, const char* delim);
void checkWifiFile(void);
void serialMenu();
void serialGetDeviceId(void);
void serialGetCo2Zero(void);
void serialGetCo2Slope(void);
void serialGetCoZeroA(void);
void serialGetCoZeroB(void);
void serialGetCoSlopeA(void);
void serialGetCoSlopeB(void);
void serialGetOzoneOffset(void);
void serialResetSettings(void);
void serialTestRemoteFunction(void);
void serialIncreaseInputCurrent(void);
void writeLogFile(String data);

void outputSerialMenuOptions(void);
void outputToCloud(void);
void echoGps();
void readOzone(void);
float readCO_A(void);
float readCO_B(void);
float getEspOzoneData(void);
void getEspAQSyncData(void);
void resetEsp(void);
void sendEspSerialCom(char *serial_command);
int remoteWriteStoredVars(String addressAndValue);
int remoteReadStoredVars(String mem_address);
void writeDefaultSettings(void);
void readHIH8120(void);
void sendAqsyncData(String data);
void saveDiagnosticData(String data);
String checksumMaker(String data);

//gps functions
void enableLowPowerGPS(void);
void enableContinuousGPS(void);
void changeFrequency(void);
void sendPacket(byte *packet, byte len);
void sendPacket(byte *packet, byte len);

//google api callback
void locationCallback(float lat, float lon, float accuracy);

//void testsensible();

//test for setting up PMIC manually
void writeRegister(uint8_t reg, uint8_t value) 
{
    // This would be easier if pmic.writeRegister wasn't private
    Wire3.beginTransmission(PMIC_ADDRESS);
    Wire3.write(reg);
    Wire3.write(value);
    Wire3.endTransmission(true);
}

void outputToCloud(String data, String sensible_data)
{
    String webhook_data = " ";
    if(Particle.connected() && serial_cellular_enabled)
    {
        status_word.status_int |= 0x0002;
        Particle.publish("pamup", data, PRIVATE);
        Particle.process(); //attempt at ensuring the publish is complete before sleeping
        if(debugging_enabled)
        {
            Serial.println("Published pamup data!");
            writeLogFile("Published pamup data!");
        }
    }
    else
    {
        if(serial_cellular_enabled == 0)
        {
            if(debugging_enabled)
            {
                Serial.println("Cellular is disabled.");
                writeLogFile("Cellular is disabled.");
            }
        }
        else
        {
            status_word.status_int &= 0xFFFD;   //clear the connected bit
            if(debugging_enabled)
            {
                Serial.println("Couldn't connect to particle.");
                writeLogFile("Couldn't connect to particle.");
            }
        }
    }
}

//send memory address and value separated by a comma
int remoteWriteStoredVars(String addressAndValue)
{
    int index_of_comma = addressAndValue.indexOf(',');
    Serial.print("Full address and value substring: ");
    Serial.println(addressAndValue);
    String addressString = addressAndValue.substring(0, index_of_comma);
    String valueString = addressAndValue.substring(index_of_comma + 1);

    int numerical_mem_address = addressString.toInt();
    int numerical_value = valueString.toInt();

    if (numerical_mem_address >= 0 && numerical_mem_address <= MAX_MEM_ADDRESS) 
    {
        EEPROM.put(numerical_mem_address, numerical_value);
        return 1;
    }
    else 
    {
        return -1;
    }
}

int remoteReadStoredVars(String mem_address) 
{
    uint16_t tempValue = 0;
    int numerical_mem_address = mem_address.toInt();
    if (numerical_mem_address >= 0 && numerical_mem_address <= MAX_MEM_ADDRESS) 
    {
        EEPROM.get(numerical_mem_address, tempValue);
        return tempValue;
    }
    else 
    {
        return -1;
    }
}

//read all eeprom stored variables
void readStoredVars(void) 
{
    int tempValue;
    //just changing the rh calibration for temporary!! -- remove me!!
    //these values were determined by John Birks from 2019 cdphe study at la casa in denver February 2019

    EEPROM.get(DEVICE_ID_MEM_ADDRESS, DEVICE_id);
    if (DEVICE_id == -1) 
    {
        DEVICE_id = 1555;
        writeDefaultSettings();
    }

    EEPROM.get(CO2_SLOPE_MEM_ADDRESS, tempValue);
    CO2_slope = tempValue;
    CO2_slope /= 100;
    EEPROM.get(CO_SLOPE_A_MEM_ADDRESS, tempValue);
    CO_slopeA = tempValue;
    CO_slopeA /= 100;
    EEPROM.get(CO_SLOPE_B_MEM_ADDRESS, tempValue);
    CO_slopeB = tempValue;
    CO_slopeB /= 100;
    EEPROM.get(PM_1_SLOPE_MEM_ADDRESS, tempValue);
    PM_1_slope = tempValue;
    PM_1_slope /= 100;
    EEPROM.get(PM_25_SLOPE_MEM_ADDRESS, tempValue);
    PM_25_slope = tempValue;
    PM_25_slope /= 100;
    EEPROM.get(PM_10_SLOPE_MEM_ADDRESS, tempValue);
    PM_10_slope = tempValue;
    PM_10_slope /= 100;
    EEPROM.get(TEMP_SLOPE_MEM_ADDRESS, tempValue);  //temperature
    temp_slope = tempValue;
    temp_slope /= 100;
    EEPROM.get(PRESSURE_SLOPE_MEM_ADDRESS, tempValue);
    pressure_slope = tempValue;
    pressure_slope /= 100;
    EEPROM.get(RH_SLOPE_MEM_ADDRESS, tempValue);
    rh_slope = tempValue;
    rh_slope /= 100;

    EEPROM.get(CO2_ZERO_MEM_ADDRESS, CO2_zero);
    EEPROM.get(CO_ZERO_A_MEM_ADDRESS, CO_zeroA);
    EEPROM.get(CO_ZERO_B_MEM_ADDRESS, CO_zeroB);
    EEPROM.get(PM_1_ZERO_MEM_ADDRESS, PM_1_zero);
    EEPROM.get(PM_25_ZERO_MEM_ADDRESS, PM_25_zero);
    EEPROM.get(PM_10_ZERO_MEM_ADDRESS, PM_10_zero);
    EEPROM.get(TEMP_ZERO_MEM_ADDRESS, temp_zero);
    EEPROM.get(PRESSURE_ZERO_MEM_ADDRESS, pressure_zero);
    EEPROM.get(RH_ZERO_MEM_ADDRESS, rh_zero);

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
    EEPROM.get(CO_SOCKET_MEM_ADDRESS, CO_socket);
    EEPROM.get(GOOGLE_LOCATION_MEM_ADDRESS, google_location_en);
    EEPROM.get(SENSIBLE_IOT_ENABLE_MEM_ADDRESS, sensible_iot_en);
    EEPROM.get(CAR_TOPPER_POWER_MEM_ADDRESS, car_topper_power_en);

    if (sensible_iot_en) 
    {
        Time.zone(0);       //use UTC if using sensible iot upload
    }

    //measurements_to_average = 5;
    if (measurements_to_average < 1 || measurements_to_average > 5000)
        measurements_to_average = 1;

    //check all values to make sure are within limits
    if (!CO2_slope)
    {
        CO2_slope = 1;
    }
    if (!CO_slopeA)
    {
        CO_slopeA = 1;
    }
    if (!CO_slopeB)
    {
        CO_slopeB = 1;
    }
    if (!PM_1_slope)
    {
        PM_1_slope = 1;
    }
    if (!PM_25_slope)
    {
        PM_25_slope = 1;
    }
    if (!PM_10_slope)
    {
        PM_10_slope = 1;
    }
}

void writeDefaultSettings(void)
{
    EEPROM.put(DEVICE_ID_MEM_ADDRESS, 1555);

    EEPROM.put(CO2_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(CO_SLOPE_A_MEM_ADDRESS, 100);
    EEPROM.put(CO_SLOPE_B_MEM_ADDRESS, 100);
    EEPROM.put(PM_1_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(PM_25_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(PM_10_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(TEMP_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(PRESSURE_SLOPE_MEM_ADDRESS, 100);
    EEPROM.put(RH_SLOPE_MEM_ADDRESS, 100);

    EEPROM.put(CO2_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(CO_ZERO_A_MEM_ADDRESS, 0);
    EEPROM.put(CO_ZERO_B_MEM_ADDRESS, 0);
    EEPROM.put(PM_1_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(PM_25_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(PM_10_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(TEMP_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(PRESSURE_ZERO_MEM_ADDRESS, 0);
    EEPROM.put(RH_ZERO_MEM_ADDRESS, 0);

    EEPROM.put(SERIAL_CELLULAR_EN_MEM_ADDRESS, serial_cellular_enabled);
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
    EEPROM.put(OZONE_OFFSET_MEM_ADDRESS, 0);
    EEPROM.put(MEASUREMENTS_TO_AVG_MEM_ADDRESS, 1);
    EEPROM.put(BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS, 1);
    EEPROM.put(ABC_ENABLE_MEM_ADDRESS, 0);
    EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, 1);
    EEPROM.put(CO_SOCKET_MEM_ADDRESS, 0);
    EEPROM.put(GOOGLE_LOCATION_MEM_ADDRESS, 0);
    EEPROM.put(SENSIBLE_IOT_ENABLE_MEM_ADDRESS, 0);
    EEPROM.put(CAR_TOPPER_POWER_MEM_ADDRESS, 0);
}

void setup()
{

    serial_cellular_enabled = 1;
    status_word.status_int = 0;
    status_word.status_int |= (APP_VERSION << 12) & 0xFF00;
    status_word.status_int |= (BUILD_VERSION << 8) & 0xF00;
    //status_word.status_int |= 0x6500;

    //Initialization error log
    String init_log;

    setADCSampleTime(ADC_SampleTime_480Cycles);
    //setup i/o
    pinMode(lmp91000_1_en, OUTPUT);
    pinMode(lmp91000_2_en, OUTPUT);
    pinMode(fiveVolt_en, OUTPUT);
    pinMode(serial4Enabler, OUTPUT);
    pinMode(power_led_en, OUTPUT);
    pinMode(esp_wroom_en, OUTPUT);
    pinMode(blower_en, OUTPUT);
    //pinMode(D4, INPUT);
    pinMode(co2_en, OUTPUT);

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

    if (powerCheck.getHasPower() == 0) 
    {
        goToSleepBattery();
    }
    else if ((battery_threshold_enable == 1) && (fuel.getSoC() < BATTERY_THRESHOLD) && (powerCheck.getHasPower() == 0)) 
    {
        //Serial.println("Going to sleep because battery is below 20% charge");
        goToSleepBattery();
    }
    //if user presses power button during operation, reset and it will go to low power mode
    //attachInterrupt(D4, System.reset, RISING);
    //if(digitalRead(D4)){
    //  goToSleep();
    //}

    digitalWrite(lmp91000_1_en, HIGH);
    digitalWrite(lmp91000_2_en, HIGH);
    digitalWrite(power_led_en, HIGH);
    digitalWrite(serial4Enabler, HIGH);
    digitalWrite(esp_wroom_en, HIGH);
    digitalWrite(blower_en, HIGH);
    digitalWrite(co2_en, HIGH);
    digitalWrite(fiveVolt_en, HIGH);

    // register the cloud function
    Particle.function("RebootADevice", rebootDevice);
    Particle.function("diagnostics", sendDiagnostics);
    //Particle.variable("CO_zeroA", CO_zeroA);
    //debugging_enabled = 1;  //for testing...
    //initialize serial1 for communication with BLE nano from redbear labs
    Serial1.begin(9600);
    //init serial4 to communicate with Plantower PMS5003
    Serial4.begin(9600);
    Serial5.begin(9600);        //gps is connected to this serial port

    // this sets up serial1 buffer size to be larger for receiveing pi data
    serBuf.setup();

    //delay for 5 seconds to give time to programmer person for connecting to serial port for debugging
    delay(10000);
    //initialize main serial port for debug output
    Serial.begin(9600);

#if sd_en
    fileName = String(DEVICE_id) + "_" + String(Time.year()) + String(Time.month()) + String(Time.day()) + "_" + String(Time.hour()) + String(Time.minute()) + String(Time.second()) + ".txt";
    dataFileName = String(DEVICE_id) + "_AQSyncData_" + String(Time.year())+ '_' + String(Time.month()) + '_' + String(Time.day());
    //diagnosticFileName = String(DEVICE_id) + "_AQSyncDiagnostic_" + String(Time.year()) + String(Time.month()) + String(Time.day());
    Serial.println("Checking for sd card");

    //if uSD is functioning and MCP error has not been logged yet.
    if (sd.begin(CS)) 
    {
        Serial.print("Created new file to log to uSD card: ");
        Serial.println(dataFileName);
        //Serial.println(diagnosticFileName);
    }
    //uSD is not functioning
    else 
    { 
        Serial.println("No uSD card detected.");
    }
#endif

    //setup the AFE
    Serial.println("Starting LMP91000 CO initialization");
    if (debugging_enabled)
        writeLogFile("Starting LMP91000 CO initialization");
    Wire.begin();   //this must be done for the LMP91000
    digitalWrite(lmp91000_1_en, LOW); //enable the chip

    if (lmp91000_1.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0) 
    {
        Serial.println("Couldn't communicate with LMP91000_1 for CO");
        if (debugging_enabled) 
        {
            writeLogFile("Couldn't communicate with LMP91000_1 for CO");
        }
    }
    else 
    {
        Serial.println("Initialized LMP91000_1 for CO");
        if (debugging_enabled) 
        {
            writeLogFile("Initialized LMP91000 for CO");
        }
        /*Serial.print("STATUS: ");
        Serial.println(lmp91000.read(LMP91000_STATUS_REG),HEX);
        Serial.print("TIACN: ");
        Serial.println(lmp91000.read(LMP91000_TIACN_REG),HEX);
        Serial.print("REFCN: ");
        Serial.println(lmp91000.read(LMP91000_REFCN_REG),HEX);
        Serial.print("MODECN: ");
        Serial.println(lmp91000.read(LMP91000_MODECN_REG),HEX);*/
        digitalWrite(lmp91000_1_en, HIGH);  //disable
    }
    ads1.begin();
    //if can't get 1 byte from ADC1, add it to the init error log
    if (Wire.requestFrom(0x49, 1) == 0) 
    { 
        //init_log += "AD1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
        Serial.println("Could not communicate with Adafruit_ADS1115 for CO");
        if (debugging_enabled)
            writeLogFile("Could not communicate with Adafruit_ADS1115 for CO");
    }
    else 
    {
        ads1.setGain(GAIN_TWOTHIRDS);
    }

    //AFE 2 setup
    //#if AFE2_en
    Serial.println("Starting LMP91000_2 initialization");
    if (debugging_enabled)
        writeLogFile("Starting LMP91000_2 initialization");
    Wire.begin();   //this must be done for the LMP91000
    digitalWrite(lmp91000_2_en, LOW); //enable the chip

    if (lmp91000_2.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0)
    {
        Serial.println("Couldn't communicate with LMP91000 for 2");
        writeLogFile("Couldn't communicate with LMP91000 for 2");
    }
    else 
    {
        Serial.println("Initialized LMP91000 for CO 2");
        if (debugging_enabled)
            writeLogFile("Initialized LMP91000 for CO 2");
        /*Serial.print("STATUS: ");
        Serial.println(lmp91000.read(LMP91000_STATUS_REG),HEX);
        Serial.print("TIACN: ");
        Serial.println(lmp91000.read(LMP91000_TIACN_REG),HEX);
        Serial.print("REFCN: ");
        Serial.println(lmp91000.read(LMP91000_REFCN_REG),HEX);
        Serial.print("MODECN: ");
        Serial.println(lmp91000.read(LMP91000_MODECN_REG),HEX);*/
        digitalWrite(lmp91000_2_en, HIGH);  //disable
    }
    ads2.begin();
    //if can't get 1 byte from ADC1, add it to the init error log
    if (Wire.requestFrom(0x4A, 1) == 0) 
    {
      //init_log += "AD1,";
      //digitalWrite(red_status_led, HIGH);
      //delay(200);
      //digitalWrite(red_status_led, LOW);
      //delay(200);
        Serial.println("Could not communicate with Adafruit_ADS1115 for CO");
        if (debugging_enabled)
            writeLogFile("Could not communicate with Adafruit_ADS1115 for CO");
    }
    else 
    {
        ads2.setGain(GAIN_TWOTHIRDS);
    }
    //#endif

    resetESP();

    Serial.println("ESP reset!");

    Serial.print("FW Version: ");
    Serial.println(APP_VERSION);
    Serial.print("Build: ");
    Serial.println(BUILD_VERSION);

    enableContinuousGPS();

    Log.info("System version: %s", (const char*)System.version());
}

void loop() 
{
    measurement_number++;

    //read CO values and apply calibration factors
    CO_float_A = readCO_A();
    CO_float_B = readCO_B();
    readGpsStream();
    readGpsStreamDate();        //get the gps date and time along with the cellular time and determine which one to output
                                //if no gps connection, use the cellular time.

    systemTime = Time.now();
    Time.setFormat(TIME_FORMAT_ISO8601_FULL);

    if (serBuf.available() > 0)
    {
        Serial.println("readSerBufUntil");
        String receivedData = serBuf.readStringUntil('\n');
        Serial.println("check string is valisd");
        bool isValid = checkStringIsValid(receivedData);
        Serial.println(isValid);
        if (isValid)
        {
            receivedData = receivedData.substring(0, receivedData.indexOf('*')-1);
            processAqsyncMessage(receivedData);
        }
    }
    Serial.println("Print co to pi");
    outputCOtoPI();
    Serial.println("done");
    if (serial_cellular_enabled) 
    {
        status_word.status_int |= 0x01;
        //Serial.println("Cellular is enabled.");
        if (!Particle.connected() && !tried_cellular_connect) 
        {
            tried_cellular_connect = true;
            if (debugging_enabled) 
            {
                Serial.println("Connecting to cellular network");
                writeLogFile("Connecting to cellular network");
            }
            Serial.println("Turning on cellular");
            Cellular.on();
            if (debugging_enabled) 
            {
                Serial.println("after cellularOn");
                writeLogFile("After cellularOn");
            }
            Particle.connect();
            if (debugging_enabled)
            {
                Serial.println("After particle connect");
                writeLogFile("After particle connect");
            }
        }
        //this means that it is already connected
        else if (Particle.connected()) 
        {  
            if (debugging_enabled) 
            {
                Serial.println("setting tried_cellular_connect to false");
            }
            tried_cellular_connect = false;
        }
    }
    else 
    {
        //Serial.println("Cellular is disabled.");
        if (Particle.connected()) 
        {
            if (debugging_enabled) 
            {
                Serial.println("Disconnecting from cellular network");
                writeLogFile("Disconnecting from cellular network");
            }
            Cellular.off();
        }
    }

    //check power
    powerCheck.loop();

    //Serial.printf("hasPower=%d hasBattery=%d isCharging=%d\n\r", powerCheck.getHasPower(), powerCheck.getHasBattery(), powerCheck.getIsCharging());
    if ((battery_threshold_enable == 1) && (fuel.getSoC() < BATTERY_THRESHOLD) && (powerCheck.getHasPower() == 0)) 
    {
        Serial.println("Going to sleep because battery is below 20% charge");
        goToSleepBattery();
    }
}

void echoGps() 
{
    char gps_byte = 0;
    while (!serBuf.available()) 
    {
        if (Serial5.available() > 0) 
        {
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

void readGpsStream(void) 
{
    String gps_sentence = "init";
    int stringFound = 0;
    int error = 0;
    int comma_counter = 0;
    while (!stringFound && !error) 
    {
        gps_sentence = Serial5.readStringUntil('\r');
        String prefix_string = gps_sentence.substring(4, 7);
        if (prefix_string.equals("GGA")) 
        {
            //Serial.print("prefix string: ");
            //Serial.println(prefix_string);
            //Serial.print("String:");
            //Serial.println(gps_sentence);
            stringFound = 1;
        }
        else if (gps_sentence.equals("init")) 
        {
            error = 1;
            Serial.println("Error reading GPS");
            writeLogFile("Error reading GPS");
        }
    }
    if (stringFound) 
    {
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

        String tempStr;
        for (uint16_t a = 0; a < gps_sentence.length(); a++)
        {
            if (gps_sentence.charAt(a) == ',')
            {
                switch (comma_counter)
                {
                case TIME_FIELD_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 11);
                        //Serial.print("GPS utc string: ");
                        if (debugging_enabled)
                        {
                            Serial.print("GPS utc string: ");
                            Serial.println(tempStr);
                        }
                        //Serial.println(utc_string);
                    }
                    break;

                case LATITUDE_FIELD_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 10);
                        if (debugging_enabled)
                        {
                            Serial.print("Latitude string: ");
                            Serial.print(tempStr);
                        }
                        //Serial.print("Latitude string: ");
                        //Serial.print(latitudeString);
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+12));
                        gps.set_lat_decimal(tempStr, gps_sentence.charAt(a + 12));
                        status_word.status_int &= 0xFFF7;
                        //Serial.print("Latitude decimal: ");
                        //Serial.println(gps.get_latitude(), 5);
                    }
                    break;

                case LONGITUDE_FIELD_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 11);
                        if (debugging_enabled) {
                            Serial.print("longitude string: ");
                            Serial.print(tempStr);
                        }
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+13));
                        gps.set_long_decimal(tempStr, gps_sentence.charAt(a + 13));
                        //Serial.print("Longitude decimal: ");
                        //Serial.println(gps.get_longitude(), 5);
                    }
                    break;

                case NUMBER_OF_SATELLITES_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 3);
                        gps.set_satellites(tempStr);
                    }
                    break;

                case HORIZONTAL_DILUTION_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 3);
                        gps.set_horizontalDilution(tempStr);
                        status_word.status_int &= 0xFFF3;
                        if (gps.get_horizontalDilution() < 2)
                        {
                            status_word.status_int |= 0x000C;
                        }
                        else if (gps.get_horizontalDilution() < 5)
                        {
                            status_word.status_int |= 0x0008;
                        }
                        else if (gps.get_horizontalDilution() < 20)
                        {
                            status_word.status_int |= 0x0004;
                        }
                    }
                    break;

                default:
                    //Serial.printf("BAD index in readGpsStream\n");
                    break;
                }
                comma_counter++;
            }
        }
    }
}

void readGpsStreamDate(void)
{
    String gps_sentence = "init";
    int stringFound = 0;
    int error = 0;
    int comma_counter = 0;
    String prefix_string;
    while (!stringFound && !error)
    {
        gps_sentence = Serial5.readStringUntil('\r');
        prefix_string = gps_sentence.substring(4, 7);
        if (prefix_string.equals("RMC"))
        {
            //Serial.print("prefix string: ");
            //Serial.println(prefix_string);
            //Serial.print("String:");
            //Serial.println(gps_sentence);
            stringFound = 1;
        }
        else if (gps_sentence.equals("init"))
        {
            error = 1;
            Serial.println("Error reading GPS RMC");
            writeLogFile("Error reading GPS RMC");
        }
    }
    if (stringFound)
    {
        //parse the gps string into latitude, longitude
        //UTC time is after first comma
        //Latitude is after second comma (ddmm.mmmm)
        //N/S indicator is after 3rd comma
        //longitude is after 4th comma (dddmm.mmmm)
        //E/W indicator is after 5th comma
        //quality is after 6th comma
        //gps_sentence = String("$GNGGA,011545.00,3945.81586,N,10514.09384,W,1,08,1.28,1799.4,M,-21.5,M,,*40");
        comma_counter = 0;

        String tempStr;
        for (uint16_t a = 0; a < gps_sentence.length(); a++)
        {
            if (gps_sentence.charAt(a) == ',')
            {
                switch (comma_counter)
                {
                case DATE_FIELD_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 11);
                        //Serial.print("GPS utc string: ");
                        if (debugging_enabled)
                        {
                            Serial.print("GPS utc string: ");
                            Serial.println(tempStr);
                        }
                        //Serial.println(utc_string);
                    }
                    break;

                case LATITUDE_FIELD_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 10);
                        if (debugging_enabled) {
                            Serial.print("Latitude string: ");
                            Serial.print(tempStr);
                        }
                        //Serial.print("Latitude string: ");
                        //Serial.print(latitudeString);
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+12));
                        gps.set_lat_decimal(tempStr, gps_sentence.charAt(a + 12));
                        status_word.status_int &= 0xFFF7;
                        //Serial.print("Latitude decimal: ");
                        //Serial.println(gps.get_latitude(), 5);
                    }
                    break;

                case LONGITUDE_FIELD_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 11);
                        if (debugging_enabled)
                        {
                            Serial.print("longitude string: ");
                            Serial.print(tempStr);
                        }
                        //Serial.print(" ");
                        //Serial.println(gps_sentence.charAt(a+13));
                        gps.set_long_decimal(tempStr, gps_sentence.charAt(a + 13));
                        //Serial.print("Longitude decimal: ");
                        //Serial.println(gps.get_longitude(), 5);
                    }
                    break;

                //TODO, make sure this is OK
                //case NUMBER_OF_SATELLITES_INDEX:
                case 5:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 3);
                        gps.set_satellites(tempStr);
                    }
                    break;

                case HORIZONTAL_DILUTION_INDEX:
                    if (gps_sentence.charAt(a + 1) != ',')
                    {
                        tempStr = gps_sentence.substring(a + 1, a + 3);
                        gps.set_horizontalDilution(tempStr);
                        status_word.status_int &= 0xFFF3;
                        if (gps.get_horizontalDilution() < 2)
                        {
                            status_word.status_int |= 0x000C;
                        }
                        else if (gps.get_horizontalDilution() < 5)
                        {
                            status_word.status_int |= 0x0008;
                        }
                        else if (gps.get_horizontalDilution() < 20)
                        {
                            status_word.status_int |= 0x0004;
                        }
                    }
                    break;

                default:
                    //Serial.println("Received bad index in readGpsStreamDate");
                    break;
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
void sendPacket(byte* packet, byte len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        Serial5.write(packet[i]);
    }

    printPacket(packet, len);
}

// Print the packet specified to the PC serial in a hexadecimal form.
void printPacket(byte* packet, byte len)
{
    char temp[3];

    for (uint8_t i = 0; i < len; i++)
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

float readTemperature(void)
{
    float temperature = 0;
    if (hih8120_enabled)
    {
        temperature = hih.temperature();
        if (debugging_enabled)
        {
            Serial.println("Temperature reading from HIH8120");
        }
    }
    else if (new_temperature_sensor_enabled)
    {
        if (debugging_enabled)
        {
            Serial.println("Temperature reading from TMP36");
        }
        temperature = analogRead(A1);
        temperature *= VOLTS_PER_UNIT;
        temperature -= TMP36_OFFSET;
        temperature /= TMP36_VPDC;
    }
    else
    {
        if (debugging_enabled)
        {
            Serial.println("Temperature reading from BME for Alphasense");
        }
        temperature = bme.temperature;
    }
    temperature *= temp_slope;
    temperature += temp_zero;       //user input zero offset
    return temperature;
}

float readHumidity(void)
{
    float humidity;
    if (hih8120_enabled)
    {
        humidity = hih.humidity();
        humidity *= 100;
        if (debugging_enabled)
        {
            Serial.println("Humidity reading from HIH8120");
        }
    }
    else
    {
        humidity = bme.humidity;
        if (debugging_enabled)
        {
            Serial.println("Humidity reading from BME");
        }
    }

    humidity *= rh_slope;
    humidity += rh_zero;       //user input zero offset
    if (humidity > 100)
        humidity = 100;
    return humidity;
}

//read Carbon monoxide alphasense sensor
float readCO_A(void)
{
    //float float_offset;
    float CO_float;

    CO_float = readAlpha1();

    // float_offset = CO_zeroA;
    // float_offset /= 1000;

    CO_float *= CO_slopeA;
    CO_float += CO_zeroA;
    // CO_float += float_offset;

    return CO_float;
}

//read Carbon monoxide alphasense sensor
float readCO_B(void)
{
    //float float_offset;
    float CO_float;

    CO_float = readAlpha2();

    // float_offset = CO_zeroB;
    // float_offset /= 1000;

    CO_float *= CO_slopeB;
    CO_float += CO_zeroB;
    // CO_float += float_offset;

    return CO_float;
}

float readAlpha1(void)
{
    //read from CO sensor
    int32_t A0_gas = 0; //gas
    int32_t A1_aux = 0; //aux out
    int32_t A2_temperature = 0; //temperature
    int32_t half_Vref = 0; //half of Vref
    float volt0_gas = 0;
    float volt1_aux = 0;
    float volt2_temperature = 0; //need to code to be able to ensure correct sigfigs..
    float volt_half_Vref = 0;
    float sensorCurrent = 0; // Working Electrode current in microamps (millivolts / Kohms)
    float auxCurrent = 0;
    float correctedCurrent = 0;
    float alpha1_ppmraw = 0;
    String alpha1_ppmRounded = "";

    if (debugging_enabled)
    {
        Serial.println("Start of alpha read");
    }
    digitalWrite(lmp91000_1_en, LOW);   //enable

    if (Wire.requestFrom(0x49, 1) == 0)
    {
        if (debugging_enabled)
        {
            Serial.println("Couldn't communicate with LMP91000_1");
            writeLogFile("Couldn't communicate with LMP91000_1");
        }
        //operation_log += "AD1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }
    else
    {
        half_Vref = ads1.readADC_SingleEnded(3); //half of Vref
        volt_half_Vref = half_Vref * ads_bitmv;
        if (abs((volt_half_Vref) / 1000 - 1.25) > 0.5)
        {
            if (debugging_enabled)
            {
                Serial.printf("Halfvolt: %1.2f\n\r", volt_half_Vref / 1000);
                writeLogFile("Halfvolt higher than 0.5");
            }
        }
    }

    if (lmp91000_1.read(LMP91000_STATUS_REG) == 0)
    {
        if (debugging_enabled)
        {
            Serial.println("Status = 0 from LMP91000 status reg");
            writeLogFile("LMP1000 status = 0");
        }
        //operation_log += "AFE1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }

    //if(Wire.requestFrom(0x49,1) == 0 || lmp91000.read(LMP91000_STATUS_REG) == 0 || (abs((volt_half_Vref)/1000 - 1.25) > 0.5)){
    if (Wire.requestFrom(0x49, 1) == 0 || lmp91000_1.read(LMP91000_STATUS_REG) == 0)
    {
        alpha1_ppmRounded = "-99";
        volt0_gas = -99;
        volt1_aux = -99;
        volt_half_Vref = -99;
        sensorCurrent = -99;
        auxCurrent = -99;
    }
    else
    {
        A0_gas = 0;
        A1_aux = 0;
        A2_temperature = 0;
        half_Vref = 0;
        for (uint8_t i = 0; i < ALPHA_ADC_READ_AMOUNT; i++)
        {
            A0_gas += ads1.readADC_SingleEnded(0); //gas
            A1_aux += ads1.readADC_SingleEnded(1); //aux out
            A2_temperature += ads1.readADC_SingleEnded(2); //temperature
            half_Vref += ads1.readADC_SingleEnded(3); //half of Vref
        }

        A0_gas = A0_gas / ALPHA_ADC_READ_AMOUNT;
        A1_aux = A1_aux / ALPHA_ADC_READ_AMOUNT;
        A2_temperature = A2_temperature / ALPHA_ADC_READ_AMOUNT;
        half_Vref = half_Vref / ALPHA_ADC_READ_AMOUNT;

        volt0_gas = A0_gas * ads_bitmv;
        volt1_aux = A1_aux * ads_bitmv;
        volt2_temperature = A2_temperature * ads_bitmv;
        volt_half_Vref = half_Vref * ads_bitmv;

        sensorCurrent = (volt_half_Vref - volt0_gas) / (-1 * 120); // Working Electrode current in microamps (millivolts / Kohms)
        auxCurrent = (volt_half_Vref - volt1_aux) / (-1 * 150);
        //{1, -1, -0.76}, //CO-A4 (<=10C, 20C, >=30C)
        if (readTemperature() <= 15)
        {
            correctedCurrent = ((sensorCurrent)-(auxCurrent));
        }
        else if (readTemperature() <= 25)
        {
            correctedCurrent = ((sensorCurrent)-(-1) * (auxCurrent));
        }
        else
        {
            correctedCurrent = ((sensorCurrent)-(-0.76) * (auxCurrent));
        }
        alpha1_ppmraw = (correctedCurrent / 0.358); //sensitivity .358 nA/ppb - from Alphasense calibration certificate, So .358 uA/ppm
        alpha1_ppmRounded = String(alpha1_ppmraw, 2);
    }

    digitalWrite(lmp91000_1_en, HIGH);  //disable

    if (debugging_enabled)
    {
        Serial.print("CO measurements:  \n\r");
        // Serial.printf("A0_gas: %d\n\r", A0_gas);
        // Serial.printf("A1_aux: %d\n\r", A1_aux);
        // Serial.printf("A2_temp: %d\n\r", A2_temperature);
        // Serial.printf("half_vref: %d\n\r", half_Vref);
    }
    return alpha1_ppmraw;
}

float readAlpha2(void)
{
    //read from CO sensor
    int32_t A0_gas = 0; //gas
    int32_t A1_aux = 0; //aux out
    int32_t A2_temperature = 0; //temperature
    int32_t half_Vref = 0; //half of Vref
    float volt0_gas = 0;
    float volt1_aux = 0;
    float volt2_temperature = 0; //need to code to be able to ensure correct sigfigs..
    float volt_half_Vref = 0;
    float sensorCurrent = 0; // Working Electrode current in microamps (millivolts / Kohms)
    float auxCurrent = 0;
    float correctedCurrent = 0;
    float alpha2_ppmraw = 0;
    String alpha2_ppmRounded = "";

    if (debugging_enabled)
    {
        Serial.println("Start of alpha 2 read");
    }
    digitalWrite(lmp91000_2_en, LOW);   //enable

    if (Wire.requestFrom(0x4A, 1) == 0)
    {
        Serial.println("Couldn't communicate with LMP91000 2");
        //operation_log += "AD1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }
    else
    {
        half_Vref = ads2.readADC_SingleEnded(3); //half of Vref
        volt_half_Vref = half_Vref * ads_bitmv;
        if (abs((volt_half_Vref) / 1000 - 1.25) > 0.5)
        {
            //operation_log += "AD1_VREF2,";
            //digitalWrite(red_status_led, HIGH);
            //delay(200);
            //digitalWrite(red_status_led, LOW);
            //delay(200);
            Serial.print("half vref2 ads1");
            Serial.println(volt_half_Vref / 1000);
        }
    }

    if (lmp91000_2.read(LMP91000_STATUS_REG) == 0)
    {
        if (debugging_enabled)
            Serial.println("Status == 0 from LMP91000 2 status reg");
        //operation_log += "AFE1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }

    if (Wire.requestFrom(0x4A, 1) == 0 || lmp91000_2.read(LMP91000_STATUS_REG) == 0 || (abs((volt_half_Vref) / 1000 - 1.25) > 0.5))
    {
        alpha2_ppmRounded = "-99";
        volt0_gas = -99;
        volt1_aux = -99;
        volt_half_Vref = -99;
        sensorCurrent = -99;
        auxCurrent = -99;
    }
    else
    {
        A0_gas = 0;
        A1_aux = 0;
        A2_temperature = 0;
        half_Vref = 0;
        for (int i = 0; i < ALPHA_ADC_READ_AMOUNT; i++)
        {
            A0_gas += ads2.readADC_SingleEnded(0); //gas
            A1_aux += ads2.readADC_SingleEnded(1); //aux out
            A2_temperature += ads2.readADC_SingleEnded(2); //temperature
            half_Vref += ads2.readADC_SingleEnded(3); //half of Vref
        }

        A0_gas = A0_gas / ALPHA_ADC_READ_AMOUNT;
        A1_aux = A1_aux / ALPHA_ADC_READ_AMOUNT;
        A2_temperature = A2_temperature / ALPHA_ADC_READ_AMOUNT;
        half_Vref = half_Vref / ALPHA_ADC_READ_AMOUNT;

        volt0_gas = A0_gas * ads_bitmv;
        volt1_aux = A1_aux * ads_bitmv;
        volt2_temperature = A2_temperature * ads_bitmv;
        volt_half_Vref = half_Vref * ads_bitmv;

        sensorCurrent = (volt_half_Vref - volt0_gas) / (-1 * 120); // Working Electrode current in microamps (millivolts / Kohms)
        auxCurrent = (volt_half_Vref - volt1_aux) / (-1 * 150);
        //{1, -1, -0.76}, //CO-A4 (<=10C, 20C, >=30C)
        if (readTemperature() <= 15)
        {
            correctedCurrent = ((sensorCurrent)-(auxCurrent));
        }
        else if (readTemperature() <= 25)
        {
            correctedCurrent = ((sensorCurrent)-(-1) * (auxCurrent));
        }
        else if (readTemperature() > 25)
        {
            correctedCurrent = ((sensorCurrent)-(-0.76) * (auxCurrent));
        }
        alpha2_ppmraw = (correctedCurrent / 0.358); //sensitivity .358 nA/ppb - from Alphasense calibration certificate, So .358 uA/ppm
        alpha2_ppmRounded = String(alpha2_ppmraw, 2);
    }

    digitalWrite(lmp91000_2_en, HIGH);  //disable

    /*Serial.print("CO:  ");
    Serial.print(volt0_gas);
    Serial.print(", ");
    Serial.print(volt2_temperature);
    Serial.print(", ");
    Serial.print(volt3_half_Vref);
    Serial.print(", ");
    Serial.print(sensorCurrent);
    Serial.print(", ");
    Serial.println(alpha4_ppmraw);
    Serial.println();

    Serial.print("Volt1 Aux:");
    Serial.print(volt1_aux);
    Serial.println("Volts");*/
    return alpha2_ppmraw;
}

void writeLogFile(String data)
{
    if (sd.begin(CS))
    {
        Serial.println("Writing data to log file.");
        log_file.open(logFileName, O_CREAT | O_APPEND | O_WRITE);
        if (log_file_started == 0)
        {
            log_file.println("File Start timestamp: ");
            log_file.println(Time.timeStr());
            log_file_started = 1;
        }
        log_file.println(data);

        log_file.close();
    }
    else
    {
        Serial.println("Unable to write to log file");
    }
}

void outputDataToESP(void)
{
    //used for converting double to bytes for latitude and longitude
    //char buffer[2];
    /*union {
        double myDouble;
        unsigned char bytes[sizeof(double)];
    } doubleBytes;*/
    //doubleBytes.myDouble = double;

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
    String sensible_string = "";
    String latitude_string = "";
    String longitude_string = "";

    cloud_output_string += '&';

    cloud_output_string += '^';         //start delimeter
    cloud_output_string += String(1) + ";";           //header
    cloud_output_string += String(DEVICE_ID_PACKET_CONSTANT) + String(DEVICE_id);   //device id
    csv_output_string += String(DEVICE_id) + ",";

    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(CO_float_A, 3);
    csv_output_string += String(CO_float_A, 3) + ",";

    csv_output_string += String(CO_float_B, 3) + ",";

    cloud_output_string += String(LATITUDE_PACKET_CONSTANT);

    if (gps.get_latitude() != 0)
    {
        if (gps.get_nsIndicator() == 0)
        {
            csv_output_string += "-";
            cloud_output_string += "-";
        }
        csv_output_string += String(gps.get_latitude()) + ",";
        cloud_output_string += String(gps.get_latitude());
    }
    else
    {
        csv_output_string += String(geolocation_latitude) + ",";
        cloud_output_string += String(geolocation_latitude);
    }

    cloud_output_string += String(LONGITUDE_PACKET_CONSTANT);

    if (gps.get_longitude() != 0)
    {
        if (gps.get_ewIndicator() == 0x01)
        {
            csv_output_string += "-";
            cloud_output_string += "-";
        }
        csv_output_string += String(gps.get_longitude()) + ",";
        cloud_output_string += String(gps.get_longitude());
    }
    else
    {
        csv_output_string += String(geolocation_longitude) + ",";
        cloud_output_string += String(geolocation_longitude);
    }

    cloud_output_string += String(ACCURACY_PACKET_CONSTANT);
    if (gps.get_longitude() != 0)
    {
        csv_output_string += String(gps.get_horizontalDilution() / 10.0) + ",";
        cloud_output_string += String(gps.get_horizontalDilution() / 10.0);
    }
    else
    {
        csv_output_string += String(geolocation_accuracy) + ",";
        cloud_output_string += String(geolocation_accuracy);
    }

    csv_output_string += String(status_word.status_int) + ",";
    csv_output_string += String(Time.format(time, "%d/%m/%y,%H:%M:%S"));
    cloud_output_string += String(PARTICLE_TIME_PACKET_CONSTANT) + String(Time.now());
    if (debugging_enabled)
    {
        Serial.println("Line to write to cloud:");
        Serial.println(cloud_output_string);
    }

    outputToCloud(cloud_output_string, "blahfornow");

    if (esp_wifi_connection_status)
    {
        if (debugging_enabled)
        {
            Serial.println("Sending data to esp to upload via wifi...");
            writeLogFile("Sending data to esp to upload via wifi");
        }
        Serial1.println(cloud_output_string);
    }
    Serial.println(csv_output_string);

    //write data to file
    if (sd.begin(CS))
    {
        if (debugging_enabled)
        {
            Serial.println("Writing row to file.");
        }
        file.open(fileName, O_CREAT | O_APPEND | O_WRITE);
        if (file_started == 0)
        {
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

    byte ble_output_array[NUMBER_OF_SPECIES * BLE_PAYLOAD_SIZE];     //19 bytes per data line and 12 species to output

    for (uint8_t i = 0; i < NUMBER_OF_SPECIES; i++)
    {
        //************Fill the ble output array**********************//
        //Serial.printf("making array[%d]\n", i);
        //byte 0 - version
        ble_output_array[0 + i * (BLE_PAYLOAD_SIZE)] = 1;

        //bytes 1,2 - Device ID
        //DEVICE_id = 555;
        wordBytes.myWord = DEVICE_id;
        ble_output_array[1 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[2 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        //byte 3 - Measurement number
        ble_output_array[3 + i * (BLE_PAYLOAD_SIZE)] = sample_counter;

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

        switch (i)
        {
        case 0:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = CARBON_MONOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = CO_float_A;
            break;

        case 1:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = CARBON_DIOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = CO2_float;
            break;

        case 2:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
            floatBytes.myFloat = fuel.getSoC();
            break;

        case 3:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
            floatBytes.myFloat = PM01Value;
            break;

        case 4:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
            floatBytes.myFloat = corrected_PM_25;
            break;

        case 5:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
            floatBytes.myFloat = PM10Value;
            break;

        case 6:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = TEMPERATURE_PACKET_CONSTANT;
            floatBytes.myFloat = readTemperature();
            break;

        case 7:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = PRESSURE_PACKET_CONSTANT;
            floatBytes.myFloat = bme.pressure / 100.0;
            break;

        case 8:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = HUMIDITY_PACKET_CONSTANT;
            floatBytes.myFloat = readHumidity();
            break;

        case 9:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = SOUND_PACKET_CONSTANT;
            floatBytes.myFloat = sound_average;
            break;

        case 10:
            ble_output_array[4 + i * (BLE_PAYLOAD_SIZE)] = VOC_PACKET_CONSTANT;
            floatBytes.myFloat = air_quality_score;
            break;

        default:
            break;
        }

        //bytes 5,6,7,8 - Measurement Value
        ble_output_array[5 + i * (BLE_PAYLOAD_SIZE)] = floatBytes.bytes[0];
        ble_output_array[6 + i * (BLE_PAYLOAD_SIZE)] = floatBytes.bytes[1];
        ble_output_array[7 + i * (BLE_PAYLOAD_SIZE)] = floatBytes.bytes[2];
        ble_output_array[8 + i * (BLE_PAYLOAD_SIZE)] = floatBytes.bytes[3];

        //bytes 9-12 - latitude
        wordBytes.myWord = gps.get_latitudeWhole();
        ble_output_array[9 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[10 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        wordBytes.myWord = gps.get_latitudeFrac();
        ble_output_array[11 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[12 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        //bytes 14-17 - longitude
        wordBytes.myWord = gps.get_longitudeWhole();
        ble_output_array[13 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[14 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        wordBytes.myWord = gps.get_longitudeFrac();
        ble_output_array[15 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[0];
        ble_output_array[16 + i * (BLE_PAYLOAD_SIZE)] = wordBytes.bytes[1];

        //byte 18 - east west and north south indicator
        //  LSB 0 = East, LSB 1 = West
        //  MSB 0 = South, MSB 1 = North
        int northSouth = gps.get_nsIndicator();
        int eastWest = gps.get_ewIndicator();

        ble_output_array[17 + i * (BLE_PAYLOAD_SIZE)] = northSouth | eastWest;
        ble_output_array[18 + i * (BLE_PAYLOAD_SIZE)] = gps.get_horizontalDilution();
        ble_output_array[19 + i * (BLE_PAYLOAD_SIZE)] = status_word.byte[1];
        ble_output_array[20 + i * (BLE_PAYLOAD_SIZE)] = status_word.byte[0];

        ble_output_array[21 + i * (BLE_PAYLOAD_SIZE)] = '#';     //delimeter for separating species
    }

    //send start delimeter to ESP
    Serial1.print("$");
    //send the packaged data with # delimeters in between packets
    Serial1.write(ble_output_array, NUMBER_OF_SPECIES * BLE_PAYLOAD_SIZE);

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
void getEspWifiStatus(void)
{
    //command to ask esp for wifi status
    String doYouHaveWifi = "!&";
    char yes_or_no = ' ';
    //if esp doesn't answer, keep going
    //Serial1.setTimeout(5000);

    Serial1.print(doYouHaveWifi);
    while (!Serial1.available());
    //delay(1000);
    yes_or_no = Serial1.read();
    if (debugging_enabled)
    {
        Serial.print("ESP Wifi connection status is: ");
    }
    //Serial.println(yes_or_no);
    if (yes_or_no == 'y')
    {
        if (debugging_enabled)
        {
            Serial.println("Connected!");
            writeLogFile("ESP wifi connected");
        }
        esp_wifi_connection_status = 1;
    }
    else
    {
        if (debugging_enabled)
        {
            Serial.println("No Connection");
            writeLogFile("ESP wifi not connected");
        }
        esp_wifi_connection_status = 0;
    }
}

//send wifi information to the ESP
void sendWifiInfo(void)
{
    String wifiCredentials = "@!" + String(ssid) + "," + String(password) + "&";
    Serial.println("Sending new wifi credentials to ESP");
    Serial1.println(wifiCredentials);
    Serial.println("Success!");
}


void getEspAQSyncData(char incomingByte)
{
    String receivedData = "";
    char marker;
    int tracking = 0;
    while (tracking == 0)
    {
        marker = serBuf.read();
        if (marker == '{')
        {
            tracking++;
            receivedData += marker;
        }
    }
    while(tracking != 0)
    {
        marker = serBuf.read();
        if (marker == '{')
        {
            tracking++;
        }
        if (marker == '}')
        {
            tracking--;
        }
        receivedData += marker;
    }
    char buffer[receivedData.length()];
    receivedData.toCharArray(buffer, receivedData.length());
    
    receivedData.replace("\\", "");

    //This removes the newline characted at the end of the string so it is properly formatted.
    if (receivedData[receivedData.length()-1] == '\r')
    {
        receivedData = receivedData.substring(0, receivedData.length()-2);
    }

    if (incomingByte == 'Y')
    {
        sendToDataFile(receivedData);
        Serial.println(receivedData);
        if(Particle.connected())
        {
            Particle.publish("AQSync", receivedData, PRIVATE);
            Particle.process(); //attempt at ensuring the publish is complete before sleeping
            // if (sending_offline == true)
            // {
            //     if (haveOfflineData) 
            //     {
            //         uploadOfflineData();
            //         haveOfflineData = false;
            //     }
            //     else 
            //     {
            //         sendToUploadLater(receivedData);
            //         haveOfflineData = true;
            //     }
            // }
        }  
    }
    if (incomingByte == 'Q')
    {
        int s = receivedData.indexOf(':');
        String deviceName = receivedData.substring(2, s-1);
        Serial.print("The device name for diangostics: ");
        Serial.println(deviceName);

        bool foundMatch = false;
        for(int i = 0; i < diagnostics.size(); i++)
        {
            int checkDevice = diagnostics[i].indexOf(deviceName);
            if (checkDevice > 0)
            {
                diagnostics[i] = receivedData;
                foundMatch = true;
            }
        }
        if (foundMatch == false)
        {
            diagnostics.push_back(receivedData);
        }
        Serial.print("How many entries into Diagnostics: ");
        Serial.println(diagnostics.size());
    }
    char next;
    bool done = false;
    while (next != 'Y' && next != 'Q' && done == false)
    {
        if (!serBuf.available())
        {
            done = true;
        }
        else 
        {
            next = serBuf.read();
        }
    }
    if (done == true)
    {
        return ;
    }
    else if (next == 'Y' || next == 'Q')
    {
        getEspAQSyncData(next);
    }
}

int rebootDevice(String deviceName)
{
    String command = "R PowerRelay|R " + deviceName; 
    serBuf.write('f R');
    return 1;
}

int sendDiagnostics(String nothing)
{
    bool Done = false;

    while (Done == false)
    {
        String sendUpData = diagnosticData.substring(0, diagnosticData.indexOf('&')-1);
        Particle.publish("UploadAQSyncDiagnostic", sendUpData, PRIVATE);
        Particle.process(); //attempt at ensuring the publish is complete before sleeping
        diagnosticData = diagnosticData.substring(diagnosticData.indexOf('&')+1, diagnosticData.length());
        if (diagnosticData.length() <= 2)
        {
            Done = true;
        }
    }
    // This is in case the AQSync sent data while it was uploading the diagnostic data.
    // Any data lost here is negligable because we are pushing so quickly. When we start going the speed we want, this will almost never happen.
    serBuf.flush();
    return 1;
}

void readHIH8120(void)
{
    hih.start();
    //  request an update of the humidity and temperature
    hih.update();
}

void goToSleep(void)
{
    //Serial.println("Going to sleep:)");
    digitalWrite(power_led_en, LOW);
    //digitalWrite(serial4Enabler, LOW);
    digitalWrite(esp_wroom_en, LOW);
    digitalWrite(blower_en, LOW);
    digitalWrite(co2_en, LOW);
    digitalWrite(fiveVolt_en, LOW);
    enableLowPowerGPS();
    //System.sleep(D4, FALLING, sleepInterval * 2);     //every 2 minutes wake up and check if battery voltage is too low
    System.reset();
}

void goToSleepBattery(void)
{
    digitalWrite(power_led_en, HIGH);   // Sets the LED on
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, HIGH);   // Sets the LED on
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, HIGH);   // Sets the LED on
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, HIGH);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, HIGH);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, LOW);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, HIGH);    // Sets the LED off
    delay(250);                   // waits for a second
    digitalWrite(power_led_en, LOW);    // Sets the LED off

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

void resetESP(void)
{
    digitalWrite(esp_wroom_en, LOW);
    //digitalWrite(serial4Enabler, LOW);
    digitalWrite(blower_en, LOW);
    digitalWrite(co2_en, LOW);
    delay(1000);
    digitalWrite(esp_wroom_en, HIGH);
    //digitalWrite(serial4Enabler, HIGH);
    digitalWrite(blower_en, HIGH);
    digitalWrite(co2_en, HIGH);
    delay(1000);
}

/************************Serial menu stuff******************/

void serialMenu()
{
    byte fault;
    byte systemStatus;
    incomingByte = '0';
    while (incomingByte != 'x')
    {
        Serial.print("Menu>");
        Serial.flush();
        while (!serBuf.available());
        String receivedData = serBuf.readStringUntil('\n');
        bool isValid = checkStringIsValid(receivedData);
        if (isValid)
        {
            incomingByte = receivedData[0];
            switch (incomingByte)
            {
            case 'a':
                serialGetCoSlopeA();
                break;

            case 'b':
                serialGetCoZeroA();
                break;

            case 'c':
                serialGetCoSlopeB();
                break;

            case 'd':
                serialGetCoZeroB();
                break;
            case 'e':
                serBuf.printf("S %i", DEVICE_id);
                break;
            case 'f':
                serBuf.printf("f %s", System.deviceID());
                break;
            case 'g':
                Serial.println("Turning on cellular (should already be on)");
                EEPROM.put(SERIAL_CELLULAR_EN_MEM_ADDRESS, serial_cellular_enabled);
                break;

            case 'q':
                Serial.println("Serial debugging enabled.");
                debugging_enabled = 1;
                EEPROM.put(DEBUGGING_ENABLED_MEM_ADDRESS, debugging_enabled);
                break;

            case 'r':
                Serial.println("Serial debugging disabled.");
                debugging_enabled = 0;
                EEPROM.put(DEBUGGING_ENABLED_MEM_ADDRESS, debugging_enabled);
                break;

            case 's':
                Serial.println("activating saving offline data to send later.");
                sending_offline = true;
                break;

            case 't':
                serialGetTimeDate();
                break;

            case 'u':
                serialGetZone();
                break;

            case 'v':
                serialGetDeviceId();
                break;

            case 'w':
                serialGetWifiCredentials();
                break;

            // case 'y':
            //     deleteFiles();
            //     break;
            case 'y':
                if (serial_cellular_enabled == 0)
                {
                    Serial.println("Enabling Cellular.");
                }
                else
                {
                    Serial.println("Cellular already enabled.");
                }
                serial_cellular_enabled = 1;
                EEPROM.put(SERIAL_CELLULAR_EN_MEM_ADDRESS, serial_cellular_enabled);
                break;

            case 'z':
                printFileToSerial();
                break;

            case 'B':
                if (output_only_particles == 1)
                {
                    output_only_particles = 0;
                    Serial.println("Outputting normally");
                }
                else
                {
                    output_only_particles = 1;
                    Serial.println("Outputting only PM");
                }
                EEPROM.put(OUTPUT_PARTICLES_MEM_ADDRESS, output_only_particles);
                break;

            case 'C':
                if (temperature_units == FAHRENHEIT)
                {
                    temperature_units = CELSIUS;
                }
                else
                {
                    Serial.println("Temperature units already set to Celsius.");
                }

                EEPROM.put(TEMPERATURE_UNITS_MEM_ADDRESS, temperature_units);
                break;

            case 'D':
                if (new_temperature_sensor_enabled == 1)
                {
                    new_temperature_sensor_enabled = 0;
                    Serial.println("Disabling new temperature sensor");
                }
                else
                {

                    Serial.println("Temperature sensor already disabled");
                }
                EEPROM.put(TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS, new_temperature_sensor_enabled);
                break;

            case 'E':
                if (new_temperature_sensor_enabled == 1)
                {
                    Serial.println("Temperature sensor already enabled");
                }
                else
                {
                    new_temperature_sensor_enabled = 1;
                    Serial.println("Temperatue sensor now enabled");
                }
                EEPROM.put(TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS, new_temperature_sensor_enabled);
                break;

            case 'F':
                if (temperature_units == CELSIUS)
                {
                    temperature_units = FAHRENHEIT;
                }
                else
                {
                    Serial.println("Temperature units already set to Fahrenheit.");
                }
                EEPROM.put(TEMPERATURE_UNITS_MEM_ADDRESS, temperature_units);
                break;

                //Enable analog reading of ozone and disable esp reading of ozone
            case 'G':
                if (ozone_analog_enabled == 1)
                {
                    Serial.println("Analog reading of ozone already enabled");
                }
                else
                {
                    ozone_analog_enabled = 1;
                    Serial.println("Analog reading of ozone now enabled");
                }
                EEPROM.put(OZONE_A_OR_D_MEM_ADDRESS, ozone_analog_enabled);
                break;

                //disable analog reading of ozone and read from esp
            case 'H':
                if (ozone_analog_enabled == 0)
                {
                    Serial.println("Digital reading of ozone already enabled");
                }
                else
                {
                    ozone_analog_enabled = 0;
                    Serial.println("Digital reading of ozone now enabled");
                }
                EEPROM.put(OZONE_A_OR_D_MEM_ADDRESS, ozone_analog_enabled);
                break;

                //disable analog reading of ozone and read from esp
            case 'I':
                serialGetAverageTime();
                break;

            case 'J':
                resetESP();
                Serial.println("ESP reset!");
                break;

            case 'K':
                Serial.println("Outputting GPS continuously");
                echoGps();
                break;

            case 'L':
                serialResetSettings();
                break;

            case 'M':
                //serialTestRemoteFunction();
                if (battery_threshold_enable == 1)
                {
                    Serial.println("Battery threshold already enabled");
                }
                else
                {
                    Serial.println("Enabling battery threshold limiting");
                    battery_threshold_enable = 1;
                    EEPROM.put(BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS, battery_threshold_enable);
                }
                break;

            case 'N':
                //serialTestRemoteFunction();
                if (battery_threshold_enable == 0)
                {
                    Serial.println("Battery threshold already disabled");
                }
                else
                {
                    Serial.println("Disabling battery threshold limiting");
                    battery_threshold_enable = 0;
                    EEPROM.put(BATTERY_THRESHOLD_ENABLE_MEM_ADDRESS, battery_threshold_enable);
                }
                break;

            case 'O':
                //Serial.println("Changing frequency for gps");
                //changeFrequency();
                Serial.println("Enabling low power for gps");
                enableLowPowerGPS();
                break;

                //turn off batfet
            case 'P':
                Serial.println("Turning off batfet");
                writeRegister(7, 0b01101011);
                break;

                //allow batfet to turn on
            /*case 'Q':
                Serial.println("Allowing batfet to turn on");
                writeRegister(7, 0b01001011);
                break;*/

            case 'R':
                if (abc_logic_enabled)
                {
                    Serial.println("Disabling ABC logic for CO2 sensor");
                    abc_logic_enabled = 0;
                    EEPROM.put(ABC_ENABLE_MEM_ADDRESS, abc_logic_enabled);
                    t6713.disableABCLogic();
                }
                else
                {
                    Serial.println("ABC logic already disabled");
                }
                break;

            case 'S':
                if (!abc_logic_enabled)
                {
                    Serial.println("Enabling abc logic for CO2 sensor");
                    abc_logic_enabled = 1;
                    EEPROM.put(ABC_ENABLE_MEM_ADDRESS, abc_logic_enabled);
                    t6713.enableABCLogic();
                }
                else
                {
                    Serial.println("ABC logic already enabled");
                }
                break;

            case 'T':
                if (!hih8120_enabled)
                {
                    Serial.println("Enabling HIH8120 RH sensor");
                    hih8120_enabled = 1;
                    EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, hih8120_enabled);

                }
                else
                {
                    Serial.println("Disabling HIH8120 RH sensor");
                    hih8120_enabled = 0;
                    EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, hih8120_enabled);
                }
                break;

            case 'U':
                if (!CO_socket)
                {
                    Serial.println("Now reading CO from U20-Alpha2");
                    CO_socket = 1;
                    EEPROM.put(CO_SOCKET_MEM_ADDRESS, CO_socket);
                }
                else
                {
                    Serial.println("Now reading CO from U19-Alpha1");
                    CO_socket = 0;
                    EEPROM.put(CO_SOCKET_MEM_ADDRESS, CO_socket);
                }
                break;

            case 'V':
                Serial.println("Reseting the CO2 sensor");
                t6713.resetSensor();
                break;

            case 'W':
                if (google_location_en == 1)
                {
                    Serial.println("Disabling google location services.");
                    google_location_en = 0;
                    EEPROM.put(GOOGLE_LOCATION_MEM_ADDRESS, google_location_en);
                }
                else
                {
                    Serial.println("Enabling google location services.");
                    google_location_en = 1;
                    EEPROM.put(GOOGLE_LOCATION_MEM_ADDRESS, google_location_en);
                }
                break;

                //calibrate CO2 sensor
            case 'X':
                t6713.calibrate(1);
                //6 minutes if measurement cycle is 2 seconds
                co2_calibration_timer = 180;
                break;

            case 'Z':
                Serial.println("Getting cellular information, this may take a while...");
                Log.info("IMEI=%s", CellularHelper.getIMEI().c_str());
                Log.info("IMSI=%s", CellularHelper.getIMSI().c_str());
                Log.info("ICCID=%s", CellularHelper.getICCID().c_str());
                break;

            case '3':
                Serial.print("APP Version: ");
                Serial.println(APP_VERSION);
                Serial.print("Build: ");
                Serial.println(BUILD_VERSION);
                break;

            case '4':
                if (ozone_enabled == 0)
                {
                    Serial.println("Enabling Ozone");
                }
                else
                {
                    Serial.println("Ozone already enabled");
                }
                ozone_enabled = 1;
                EEPROM.put(OZONE_EN_MEM_ADDRESS, ozone_enabled);
                break;

            case '5':
                if (ozone_enabled == 1)
                {
                    Serial.println("Disabling Ozone");
                }
                else
                {
                    Serial.println("Ozone already disabled");
                }
                ozone_enabled = 0;
                EEPROM.put(OZONE_EN_MEM_ADDRESS, ozone_enabled);
                break;

            case '6':
                if (voc_enabled == 0)
                {
                    Serial.println("Enabling VOCs");
                }
                else
                {
                    Serial.println("VOCs already enabled");
                }
                voc_enabled = 1;
                EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
                break;

            case '7':
                if (voc_enabled == 1)
                {
                    Serial.println("Disabling VOC's");
                }
                else
                {
                    Serial.println("VOC's already disabled");
                }
                voc_enabled = 0;
                EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
                break;

            case '8':
                Serial.print("Fault: ");
                fault = pmic.getFault();
                Serial.println(fault);
                Serial.print("System status: ");
                systemStatus = pmic.getSystemStatus();
                Serial.println(systemStatus);
                break;

            case '9':
                serialIncreaseChargeCurrent();
                break;

            case '0':
                serialIncreaseInputCurrent();
                break;

            case '!':
                Serial.println("Outputting VOCs continuously!  Press any button to exit...");
                while (!Serial.available())
                {
                    if (!bme.performReading())
                    {
                        Serial.println("Failed to read BME680");
                        return;
                    }
                    else
                    {
                        Serial.printf("TVocs=%1.0f, Temp=%1.1f, press=%1.1f, rh=%1.1f\n\r", bme.gas_resistance / 100, bme.temperature, bme.pressure, bme.humidity);
                    }
                }
                break;

            case '@':
                if (sensible_iot_en == 1)
                {
                    Serial.println("Disabling sensible iot data push.");
                    sensible_iot_en = 0;
                    EEPROM.put(SENSIBLE_IOT_ENABLE_MEM_ADDRESS, sensible_iot_en);
                }
                else
                {
                    serialSetSensibleIotEnable();
                }
                break;

            case '#':
                if (car_topper_power_en == 1)
                {
                    car_topper_power_en = 0;
                    Serial.println("Disabling car topper power.  ");
                    EEPROM.put(CAR_TOPPER_POWER_MEM_ADDRESS, car_topper_power_en);
                }
                else
                {
                    car_topper_power_en = 1;
                    Serial.println("Enabling car topper power.  If no external power, system will turn off.");
                    EEPROM.put(CAR_TOPPER_POWER_MEM_ADDRESS, car_topper_power_en);
                }
                break;

            case '*':
                outputCOtoPI();
                break;

            case '?':
                outputSerialMenuOptions();
                break;

            default:
                break;
            }
        }
    }
    Serial.println("Exiting serial menu...");


}

void outputCOtoPI(void)
{
    String CO_string = "";
    Serial.println("Outputting CO to PI.");

    CO_string += String(CO_float_A, 3) + ",";
    CO_string += String(CO_float_B, 3) + ",";
    if (gps.get_latitude() != 0)
    {
        if (gps.get_nsIndicator() == 0)
        {
            CO_string += "-";
        }
        CO_string += String(gps.get_latitude()) + ",";
    }
    else
    {
        CO_string += String(geolocation_latitude) + ",";
    }

    if (gps.get_longitude() != 0)
    {
        if (gps.get_ewIndicator() == 0x01)
        {
            CO_string += "-";
        }
        CO_string += String(gps.get_longitude()) + ",";
    }
    else
    {
        CO_string += String(geolocation_longitude) + ",";
    }

    if (Particle.connected())
    {
        CO_string += '1';
    }
    else 
    {
        CO_string += '0';
    }

    int checksum = 0;
    for (int i = 0; i < CO_string.length(); i++)
    {
        checksum ^= CO_string[i];
    }
    CO_string += '*';
    CO_string += checksum;
    CO_string += '\n';

    serBuf.print(CO_string);
}

void serialTestRemoteFunction(void)
{
    Serial.println("Enter string (address,value)");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);
    String tempString = Serial.readStringUntil('\r');
    int response = remoteWriteStoredVars(tempString);
    if (response)
    {
        Serial.println("Success in writing");
    }
    else
    {
        Serial.println("Failed writing string");
    }
}

void serialIncreaseInputCurrent(void)
{
    int inputCurrent = pmic.getInputCurrentLimit();
    Serial.printf("Old input current limit: %d\n\r", inputCurrent);

    if (inputCurrent == 100)
    {
        inputCurrent = 150;
    }
    else if (inputCurrent == 100)
    {
        inputCurrent = 150;
    }
    else if (inputCurrent == 150)
    {
        inputCurrent = 500;
    }
    else if (inputCurrent == 500)
    {
        inputCurrent = 900;
    }
    else if (inputCurrent == 900)
    {
        inputCurrent = 1200;
    }
    else if (inputCurrent == 1200)
    {
        inputCurrent = 1500;
    }
    else if (inputCurrent == 1500)
    {
        inputCurrent = 2000;
    }
    else if (inputCurrent == 2000)
    {
        inputCurrent = 3000;
    }
    //delay(2000);
    pmic.setInputCurrentLimit(inputCurrent);
    Serial.printf("New input current limit: %d\n\r", inputCurrent);
}

void serialIncreaseChargeCurrent(void)
{
    int total_current = 0;
    bool bit7 = 0;
    bool bit6 = 0;
    bool bit5 = 0;
    bool bit4 = 0;
    bool bit3 = 0;
    bool bit2 = 0;

    byte chargeCurrent = pmic.getChargeCurrent();
    //bit 7
    if (chargeCurrent & 0x80)
    {
        total_current += 2048;
    }
    //bit 6
    if (chargeCurrent & 0x40)
    {
        total_current += 1024;
    }
    //bit 5
    if (chargeCurrent & 0x20)
    {
        total_current += 512;
    }
    //bit 4
    if (chargeCurrent & 0x10) {
        total_current += 256;
    }
    //bit 3
    if (chargeCurrent & 0x08)
    {
        total_current += 128;
    }
    //bit 2
    if (chargeCurrent & 0x04)
    {
        total_current += 64;
    }
    Serial.printf("Increasing Charge current from %d mA to ", total_current);
    chargeCurrent += 4;
    total_current = 0;

    if (chargeCurrent & 0x80)
    {
        total_current += 2048;
        bit7 = 1;
    }
    //bit 6
    if (chargeCurrent & 0x40)
    {
        total_current += 1024;
        bit6 = 1;
    }
    //bit 5
    if (chargeCurrent & 0x20)
    {
        total_current += 512;
        bit5 = 1;
    }
    //bit 4
    if (chargeCurrent & 0x10)
    {
        total_current += 256;
        bit4 = 1;
    }
    //bit 3
    if (chargeCurrent & 0x08)
    {
        total_current += 128;
        bit3 = 1;
    }
    //bit 2
    if (chargeCurrent & 0x04)
    {
        total_current += 64;
        bit2 = 1;
    }

    pmic.setChargeCurrent(bit7, bit6, bit5, bit4, bit3, bit2);
    chargeCurrent = pmic.getChargeCurrent();
    Serial.printf("new charge current of %d mA\n\r", total_current);
}

void serialGetWifiCredentials(void)
{
    Serial.print("Current stored ssid: ");
    Serial.println(ssid);
    Serial.print("Current stored password: ");
    Serial.println(password);
    Serial.println("Please enter password in order to make changes.\n\r");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);
    String tempString = Serial.readStringUntil('\r');
    if (tempString.equals("bould"))
    {
        Serial.println("Password correct!");
        Serial.println("Enter new ssid:");
        Serial.setTimeout(SERIAL_MENU_TIMEOUT);
        String tempSsid = Serial.readStringUntil('\r');
        Serial.print("Your new ssid will be: ");
        Serial.println(tempSsid);
        Serial.println("Is this okay?(y or n)");
        String ok = Serial.readStringUntil('\r');
        if (ok.equals("y"))
        {
            Serial.println("Saving new ssid");
            ssid = tempSsid;
            Serial.println("Enter new password");
            String tempPassword = Serial.readStringUntil('\r');
            Serial.print("Your new password will be: ");
            Serial.println(tempPassword);
            String ok = Serial.readStringUntil('\r');
            if (ok.equals("y"))
            {
                Serial.println("Saving new password");
                password = tempPassword;
                sendWifiInfo();
            }
            else
            {
                Serial.println("okay, no problem\n\r");
            }
        }
        else
        {
            Serial.println("okay, no problem\n\r");
            return;
        }
    }
}

void serialSetSensibleIotEnable(void)
{
    Serial.println("Please enter password in order to enable data push to Sensible Iot");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);
    String tempString = Serial.readStringUntil('\r');
    if (tempString == "imsensible")
    {
        Serial.println("Password correct!");
        Serial.println("Enabling sensible iot data push.");
        sensible_iot_en = 1;
        EEPROM.put(SENSIBLE_IOT_ENABLE_MEM_ADDRESS, sensible_iot_en);
    }
    else
    {
        Serial.println("\n\rIncorrect password!");
    }
}

void serialGetDeviceId(void)
{
    Serial.println();
    Serial.print("Current Device ID:");
    Serial.println(DEVICE_id);
    Serial.println("Please enter password in order to change the ID");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);
    String tempString = readSerBufUntilDone();

    if (tempString == SERIAL_PASSWORD)
    {
        Serial.println("Password correct!");
        Serial.println("Enter new Device ID:");
        int tempValue = readSerBufUntilDone().toInt();
        Serial.println("");
        if (tempValue > MIN_DEVICE_ID_NUMBER && tempValue < MAX_DEVICE_ID_NUMBER)
        {
            Serial.print("\n\rNew Device ID:");
            Serial.println(tempValue);
            DEVICE_id = tempValue;
            EEPROM.put(DEVICE_ID_MEM_ADDRESS, DEVICE_id);
        }
        else
        {
            Serial.println("\n\rInvalid value!");
        }
    }
    else
    {
        Serial.println("\n\rIncorrect password!");
    }
}

void serialResetSettings(void)
{
    Serial.println();
    Serial.println("Please enter password in order to apply default settings");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);
    String tempString = readSerBufUntilDone();

    if (tempString == "bould")
    {
        Serial.println("Password correct, resetting all settings to default!  Please reset your ID to the one shown on your enclosure.");
        writeDefaultSettings();
    }
    else
    {
        Serial.println("\n\rIncorrect password!");
    }
}

void serialGetTimeDate(void)
{
    Serial.println("Enter new Device time and date (10 digit epoch timestamp):");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);

    String inputString = readSerBufUntilDone();
    int tempValue = inputString.toInt();

    //min is the year 2000, max is the year 2100
    if (tempValue > 966012661 && tempValue < 4121686261)
    {
        Time.setTime(tempValue);
        Serial.print("\n\rNew Device Time:");
        Serial.println(Time.timeStr());
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetZone(void)
{
    Serial.println("Enter new Device time zone (-12.0 to 14.0)");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);
    int tempValue = readSerBufUntilDone().toInt();

    Serial.println("");

    //min is the year 2000, max is the year 2100
    if (tempValue >= -12 && tempValue <= 14)
    {
        Time.zone(tempValue);
        Serial.print("\n\rNew Device time zone:");
        Serial.println(tempValue);
        EEPROM.put(TIME_ZONE_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetAverageTime(void)
{
    Serial.println();
    Serial.print("Current Frequency: ");
    Serial.print(measurements_to_average);
    Serial.println("(~2 second) measurements");
    Serial.print("Enter new amount\n\r");
    Serial.setTimeout(SERIAL_MENU_TIMEOUT);

    String inputString = readSerBufUntilDone();
    int tempValue = inputString.toInt();

    if (tempValue >= 1 && tempValue < 10000)
    {
        Serial.print("\n\rNew Frequency: ");
        Serial.println(tempValue);
        Serial.println("(~2 second) measurements");
        measurements_to_average = tempValue;
        EEPROM.put(MEASUREMENTS_TO_AVG_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCo2Slope(void)
{
    Serial.println();
    Serial.print("Current CO2 slope:");
    Serial.print(String(CO2_slope, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO2 slope\n\r");

    String inputString;
    while(incomingByte != '\r' && incomingByte != '\n')
    {
        if (serBuf.available())
        {
            incomingByte = serBuf.read();
            if (incomingByte != '\r' && incomingByte != '\n')
            {
                inputString += (char)incomingByte;
            }
        }
    }
    float tempfloat = inputString.toFloat();

    if (tempfloat >= 0.5 && tempfloat < 10.0)
    {
        CO2_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew CO2 slope: ");
        Serial.println(String(CO2_slope, 2));

        EEPROM.put(CO2_SLOPE_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCo2Zero(void)
{
    Serial.println();
    Serial.print("Current CO2 zero:");
    Serial.print(CO2_zero);
    Serial.println(" ppm");
    Serial.print("Enter new CO2 Zero\n\r");

    String inputString;
    while(incomingByte != '\r' && incomingByte != '\n')
    {
        if (serBuf.available())
        {
            incomingByte = serBuf.read();
            if (incomingByte != '\r' && incomingByte != '\n')
            {
                inputString += (char)incomingByte;
            }
        }
    }
    int tempValue = inputString.toInt();

    if (tempValue >= -1000 && tempValue < 1000)
    {
        Serial.print("\n\rNew CO2 zero: ");
        Serial.println(tempValue);
        CO2_zero = tempValue;
        EEPROM.put(CO2_ZERO_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoSlopeA(void)
{
    Serial.println();
    Serial.print("Current CO slope:");
    Serial.print(String(CO_slopeA, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO slope\n\r");

    float tempfloat = readSerBufUntilDone().toFloat();

    if (tempfloat >= 0.1 && tempfloat < 5.0)
    {
        CO_slopeA = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew COA slope: ");
        Serial.println(String(CO_slopeA, 2));

        EEPROM.put(CO_SLOPE_A_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoZeroA(void)
{
    Serial.println();
    Serial.print("Current CO_A zero:");
    Serial.print(CO_zeroA);
    Serial.println(" ppm");
    Serial.print("Enter new CO Zero\n\r");

    float tempValue = readSerBufUntilDone().toFloat();

    if (tempValue >= -5000 && tempValue < 5000)
    {
        Serial.print("\n\rNew CO zero: ");
        Serial.println(tempValue);
        CO_zeroA = tempValue;
        EEPROM.put(CO_ZERO_A_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoSlopeB(void)
{
    Serial.println();
    Serial.print("Current CO slope:");
    Serial.print(String(CO_slopeB, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO slope\n\r");

    float tempfloat = readSerBufUntilDone().toFloat();

    if (tempfloat >= 0.1 && tempfloat < 5.0)
    {
        CO_slopeB = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew COB slope: ");
        Serial.println(String(CO_slopeB, 2));

        EEPROM.put(CO_SLOPE_B_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoZeroB(void)
{
    Serial.println();
    Serial.print("Current CO_B zero:");
    Serial.print(CO_zeroB);
    Serial.println(" ppm");
    Serial.print("Enter new CO Zero\n\r");

    float tempValue = readSerBufUntilDone().toFloat();

    if (tempValue >= -5000 && tempValue < 5000)
    {
        Serial.print("\n\rNew COB zero: ");
        Serial.println(tempValue);
        CO_zeroB = tempValue;
        EEPROM.put(CO_ZERO_B_MEM_ADDRESS, tempValue);
    }
    else
    {
        Serial.println("\n\rInvalid value!");
    }
}

String readSerBufUntilDone()
{
    String inputString;
    incomingByte = 0;

    while(incomingByte != '\r' && incomingByte != '\n')
    {
        if (serBuf.available())
        {
            incomingByte = serBuf.read();
            if (incomingByte != '\r' && incomingByte != '\n')
            {
                inputString += (char)incomingByte;
            }
        }
    }
    return inputString;
}

void sendToDataFile(String receivedData)
{
    //Serial.println("Writing the data line to the SD Card: ");
    file.open(String(DEVICE_id) + "_AQSyncData_" + String(Time.year())+ '_' + String(Time.month()) + '_' + String(Time.day()), O_CREAT | O_APPEND | O_WRITE);
    file.println(receivedData);
    file.close();
}

void sendToUploadLater(String receivedData)
{
    Serial.println("Writing the data line to the upload for later file: ");
    file.open("OfflineFile", O_CREAT | O_APPEND | O_WRITE);
    file.println(receivedData);
    file.close();
}

void uploadOfflineData()
{
    char line[1000];
    int n;
    Serial.println("Sending the offline data up");
    file.open("OfflineFile", O_READ);

    while ((n = file.fgets(line, sizeof(line))) > 0) 
    {
        Particle.publish("AQSync", line, PRIVATE);
        Particle.process(); //attempt at ensuring the publish is complete before sleeping
        delay(400);
    }
    file.remove();
    file.close();
}

void printFileToSerial()
{
    Serial.println();
    Serial.println("Give the number of the file you want: ");
    Serial.println();

    String fileName = showAndChooseFiles();
    Serial.println(fileName);

    file.open(fileName, O_READ);

    char line[1000];
    int n;
    while ((n = file.fgets(line, sizeof(line))) > 0) 
    {
        Serial.println(line);
    }
    file1.close();
}

void deleteFiles()
{
    SdFat sd;
    sd.open("/");
    Serial.println();
    Serial.println("Give the number of the file you want to delete: ");
    Serial.println();

    String fileName = showAndChooseFiles();
    Serial.println(fileName);
    //SdFile deleteFile;
    //deleteFile = fileName;
    //file.open(String(fileName), O_READ);

    if (!sd.remove(fileName)) {
        Serial.println("remove failed");
    }

    //deleteFile.remove();
    //file.close();
    Serial.print(String(fileName));
    Serial.println(" has been deleted");
}

String showAndChooseFiles()
{
    int i = 0;
    char * listOfFiles = reinterpret_cast<char*>(malloc(sizeof(char) * 100 /* Fname size */ * 100 /* Num entries */));
    //Make sure the array is clear
    memset(listOfFiles, 0, sizeof(char) * 10000);


    file1.open("/");
    while (file.openNext(&file1, O_RDONLY)) {
        bool isSuccess = file.getName( listOfFiles + (i * 100), 86);

        Serial.print(i);
        Serial.print(": ");
        Serial.println(listOfFiles + (i * 100));
         i++;
        file.close();
    }
    if (file1.getError()) {
        Serial.println("openNext failed");
        file.close();
    } else {
        Serial.println("End of List.");
        file.close();
    }
    int fileLocation = readSerBufUntilDone().toInt();
    int numbers = 100*fileLocation;
    String fileName = String(listOfFiles+numbers);
    free(listOfFiles);
    return String(fileName);
}


void outputSerialMenuOptions(void)
{
    Serial.println("Command:  Description");
    Serial.println("a:  Adjust COA slope");
    Serial.println("b:  Adjust COA zero");
    Serial.println("c:  Adjust COB slope");
    Serial.println("d:  Adjust COB zero");
    Serial.println("e:  Get Device Id");
    Serial.println("f:  Get coreid");
    Serial.println("g: Active Cellular (should be activated anyways)");
    Serial.println("q:  Enable serial debugging");
    Serial.println("r:  Disable serial debugging");
    Serial.println("s:  Activate sending offline data");
    Serial.println("t:  Enter new time and date");
    Serial.println("u:  Enter new time zone");
    Serial.println("v:  Adjust the Device ID");
    Serial.println("w:  Get wifi credentials");
    Serial.println("y:  List files to choose what to delete");
    Serial.println("z:  List files to choose what to print in serial");
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
    Serial.println("A:  Output CO constantly and rapidly");
    Serial.println("B:  Output PM constantly and rapidly");
    Serial.println("C:  Change temperature units to Celsius");
    Serial.println("D:  Disable TMP36 temperature sensor and use BME680 temperature");
    Serial.println("E:  Enable TMP36 temperature sensor and disable BME680 temperature");
    Serial.println("F:  Change temperature units to Fahrenheit");
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
    Serial.println("V:  Calibrate CO2 sensor - must supply ambient level (go outside!)");
    Serial.println("Z:  Output cellular information (CCID, IMEI, etc)");
    Serial.println("!:  Continuous serial output of VOC's");
    Serial.println("@   Enable/Disable Sensible-iot data push.  If enabled, time zone will be ignored - UTC will be used.");
    Serial.println("#   Enable/Disable cartopper power mode.  If enabled, absence of external power will stop cellular.");
    Serial.println("?:  Output this menu");
    Serial.println("x:  Exits this menu");
}

void processAqsyncMessage(String data)
{
    switch (data[0])
    {
        case 'm':
            serialMenu();
            break ;
        case 'Y':
            sendAqsyncData(data.substring(1, data.length()-1));
            break ;
        case 'Q':
            saveDiagnosticData(data.substring(1, data.length()-1));
            break ;
    }

}


bool checkStringIsValid(String data)
{
    if (data.length() < 3)
    {
        return false;
    }

    char checking_message = '\0';
    int checksum = 0;
    int marker = 0;
    while (checking_message != '*' && marker < data.length())
    {
        Serial.print("Stuck in here: ");
        Serial.print(marker);
        Serial.print(" < ");
        Serial.println(data.length());
        checking_message = data[marker];
        checksum ^=  checking_message;
        marker++;
    }
    Serial.println("After while loop. Looking at: ");
    Serial.println(data.substring(data.indexOf('*'), data.length()-1).toInt());
    if (checksum == data.substring(data.indexOf('*'), data.length()-1).toInt())
    {
        String ack = "_"+data.substring(0, data.indexOf("*")-1);
        ack += ack + checksumMaker(ack);
        serBuf.print(ack);
        Serial.println("true");
        return true;
    }
    Serial.println("false");
    return false;
}

String checksumMaker(String data)
{
    String checksumString = "";
    int checksum = 0;
    for (int i = 0; i < data.length(); i++)
    {
        checksum ^= data[i];
    }
    checksumString += '*';
    checksumString += checksum;
    checksumString += '\n';
    return checksumString;
}

void sendAqsyncData(String data)
{
    int s = data.indexOf(':');
    String deviceName = data.substring(2, s-1);
    data.replace("\\", "");
    //This removes the newline characted at the end of the string so it is properly formatted.
    if (data[data.length()-1] == '\r')
    {
        data = data.substring(0, data.length()-2);
    }

    sendToDataFile(data);
    if(Particle.connected())
    {
        Particle.publish("AQSync", data, PRIVATE);
        Particle.process(); //attempt at ensuring the publish is complete before sleeping
        String sendBack = "U "+deviceName+" 1";
        sendBack += sendBack+ checksumMaker(sendBack);
        serBuf.print(sendBack);
    }
    else 
    {
        String sendBack = "U "+deviceName+" 1";
        sendBack += sendBack+ checksumMaker(sendBack);
        serBuf.print(sendBack);
    }
}


void saveDiagnosticData(String data)
{
    int s = data.indexOf(':');
    String deviceName = data.substring(2, s-1);
    bool foundMatch = false;
    for(int i = 0; i < diagnostics.size(); i++)
    {
        int checkDevice = diagnostics[i].indexOf(deviceName);
        if (checkDevice > 0)
        {
            diagnostics[i] = data;
            foundMatch = true;
        }
    }
    if (foundMatch == false)
    {
        diagnostics.push_back(data);
    }
    Serial.print("How many entries into Diagnostics: ");
    Serial.println(diagnostics.size());
}