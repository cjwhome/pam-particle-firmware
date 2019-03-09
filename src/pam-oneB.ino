/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

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
//#include "Serial1/Serial1.h"
#include "SdFat.h"

#define APP_VERSION 4
#define BUILD_VERSION 0

//define constants
#define SEALEVELPRESSURE_HPA (1013.25)
#define LOW_PRESSURE_LIMIT (100)
#define HIGH_PRESSURE_LIMIT (1500)
#define VOLTS_PER_UNIT (0.0008)   //3.3V/4096  3.3 is the adc reference voltage and the adc is 12 bits or 4096
#define VOLTS_PER_PPB (0.0125)  //2.5V/200 ppb this is what you divide the voltage reading by to get ppb in ozone if the ozone monitor is set to 2.5V=200ppb
#define PM_25_CONSTANT_A (1.19)
#define PM_25_CONSTANT_B (0.119)
#define CELCIUS 1
#define FARENHEIT 0
#define TMP36_OFFSET 0.5
#define TMP36_VPDC 0.01 //10mV per degree C

float ads_bitmv = 0.1875; //Bits per mV at defined bit resolution, used to convert ADC value to voltage
//float ads_bitmv = 0.1920;

//enable or disable different parts of the firmware by setting the following values to 1 or 0
#define sd_en 1

//enable AFE 2 (just for testing now using a second CO sensor connected to it)
#define AFE2_en 0

//define addresses of eeprom stored variables
#define DEVICE_ID_MEM_ADDRESS 0
#define CO2_ZERO_MEM_ADDRESS 4
#define CO2_SLOPE_MEM_ADDRESS 8
#define CO_ZERO_MEM_ADDRESS 12
#define CO_SLOPE_MEM_ADDRESS 16
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


//max and min values
#define MIN_DEVICE_ID_NUMBER 1
#define MAX_DEVICE_ID_NUMBER 9999

//#define MEASUREMENTS_TO_AVERAGE 1       //change to 30 for 3 minute uploads



//ble data output

#define BLE_PAYLOAD_SIZE 20     //number of bytes allowed in payload - this is sent to the ESP chip to be output in ble broadcast packets

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
#define PARTICLE_TIME_PACKET_CONSTANT 'Y'   //result of now()
#define OZONE_PACKET_CONSTANT 'O'           //Ozone
#define BATTERY_PACKET_CONSTANT 'x'         //Battery in percentage

#define HEADER_STRING "DEV,CO(ppm),CO2(ppm),VOCs(IAQ),PM1,PM2_5,PM10,T(C),Press(mBar),RH(%),O3(ppb),Batt(%),Snd(db),Latitude,Longitude,Date/Time"


#define NUMBER_OF_SPECIES 11    //total number of species (measurements) being output

#define MAX_COUNTER_INDEX 15000

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


int lmp91000_1_en = B0;     //enable line for the lmp91000 AFE chip for measuring CO
int lmp91000_2_en = B2;
int cellular_en = D5;
int plantower_en = B4;
int power_led_en = D6;
int kill_power = WKP;
int esp_wroom_en = D7;
int blower_en = D2;
int sound_input = B5;  //ozone monitor's voltage output is connected to this input
int co2_en = C5;        //enables the CO2 sensor power




//manually control connection to cellular network
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

//global objects
Adafruit_BME680 bme; // I2C
Telaire_T6713 t6713;  //CO2 sensor
LMP91000 lmp91000;
Adafruit_ADS1115 ads1(0x49); //Set I2C address of ADC1
Adafruit_ADS1115 ads2(0x4A); //Set I2C address of ADC2
FuelGauge fuel;
GPS gps;
PMIC pmic;

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
float CO_float = 0;
float CO_float_2 = 0;
float CO2_float = 0;
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
int ozone_analog_enabled = 0;           //read ozone through analog or from ESP
bool tried_cellular_connect = false;

//used for averaging
float CO_sum = 0;
float CO2_sum = 0;
float O3_sum = 0;
int measurement_count = 0;
double sound_average;


//calibration parameters
float CO2_slope;
int CO2_zero;
float CO_slope;
int CO_zero;
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
int gas_lower_limit = 1200;   // Bad air quality limit
int gas_upper_limit = 50000;  // Good air quality limit
float pm_25_correction_factor;      //based on rh, this corrects pm2.5 according to Zheng et. al 2018
int measurements_to_average = 0;

//serial menu variables
int addr;
uint16_t value;
char recieveStr[5];

//plantower PMS5003 vars
int PM01Value=0;
int PM2_5Value=0;
int PM10Value=0;
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
void writeLogFile(String data);

void outputSerialMenuOptions(void);
void outputToCloud(void);
void echoGps();
void readOzone(void);
float readCO(void);
float getEspOzoneData(void);
void resetEsp(void);
void sendEspSerialCom(char *serial_command);

//test for setting up PMIC manually
void writeRegister(uint8_t reg, uint8_t value) {
    // This would be easier if pmic.writeRegister wasn't private
    Wire3.beginTransmission(PMIC_ADDRESS);
    Wire3.write(reg);
    Wire3.write(value);
    Wire3.endTransmission(true);

}

void outputToCloud(String data){
    String webhook_data = " ";
    CO_sum += CO_float;
    CO2_sum += CO2_float;
    O3_sum += O3_float;
    measurement_count++;

    if(measurement_count == measurements_to_average){
        CO_sum /= measurements_to_average;
        CO2_sum /= measurements_to_average;
        O3_sum /= measurements_to_average;

        measurement_count = 0;
        String webhook_data = String(DEVICE_id) + ",VOC: " + String(bme.gas_resistance / 1000.0, 1) + ", CO: " + CO_sum + ", CO2: " + CO2_sum + ", PM1: " + PM01Value + ",PM2.5: " + corrected_PM_25 + ", PM10: " + PM10Value + ",Temp: " + String(readTemperature(), 1) + ",Press: ";
        webhook_data += String(bme.pressure / 100.0, 1) + ",HUM: " + String(bme.humidity, 1) + ",Snd: " + String(sound_average) + ",O3: " + O3_sum + "\n\r";

        if(Particle.connected() && serial_cellular_enabled){
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

//read all eeprom stored variables
void readStoredVars(void){
    int tempValue;
    EEPROM.get(DEVICE_ID_MEM_ADDRESS, DEVICE_id);
    if(DEVICE_id == -1){
        DEVICE_id = 555;
    }

    EEPROM.get(CO2_SLOPE_MEM_ADDRESS, tempValue);
    CO2_slope = tempValue;
    CO2_slope /= 100;
    EEPROM.get(CO_SLOPE_MEM_ADDRESS, tempValue);
    CO_slope = tempValue;
    CO_slope /= 100;
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
    EEPROM.get(CO_ZERO_MEM_ADDRESS, CO_zero);
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


    //check all values to make sure are within limits
    if(!CO2_slope)
    {
        CO2_slope = 1;
    }
    if(!CO_slope)
    {
        CO_slope = 1;
    }
    if(!PM_1_slope)
    {
        PM_1_slope = 1;
    }
    if(!PM_25_slope)
    {
        PM_25_slope = 1;
    }
    if(!PM_10_slope)
    {
        PM_10_slope = 1;
    }
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
    String init_log; //intialization error log

    setADCSampleTime(ADC_SampleTime_480Cycles);
    //setup i/o
    pinMode(lmp91000_1_en, OUTPUT);
    pinMode(lmp91000_2_en, OUTPUT);
    pinMode(cellular_en, INPUT);
    pinMode(plantower_en, OUTPUT);
    pinMode(power_led_en, OUTPUT);
    pinMode(esp_wroom_en, OUTPUT);
    pinMode(blower_en, OUTPUT);
    pinMode(D4, INPUT);
    pinMode(co2_en, OUTPUT);
    //if user presses power button during operation, reset and it will go to low power mode
    attachInterrupt(D4, System.reset, RISING);



    digitalWrite(lmp91000_1_en, HIGH);
    digitalWrite(lmp91000_2_en, HIGH);
    digitalWrite(power_led_en, HIGH);
    digitalWrite(plantower_en, HIGH);
    digitalWrite(esp_wroom_en, HIGH);
    digitalWrite(blower_en, HIGH);
    digitalWrite(co2_en, HIGH);

    //read all stored variables (calibration parameters)
    readStoredVars();
    //debugging_enabled = 1;  //for testing...
    //initialize serial1 for communication with BLE nano from redbear labs
    Serial1.begin(9600);
    //init serial4 to communicate with Plantower PMS5003
    Serial4.begin(9600);
    Serial5.begin(9600);        //gps is connected to this serial port
    //set the Timeout to 1500ms, longer than the data transmission periodic time of the sensor
    Serial4.setTimeout(5000);
    if(digitalRead(D4)){
      goToSleep();
    }
    //delay for 5 seconds to give time to programmer person for connecting to serial port for debugging
    delay(10000);
    //initialize main serial port for debug output
    Serial.begin(9600);

    // Setup the PMIC manually (resets the BQ24195 charge controller)
    // REG00 Input Source Control Register  (disabled)
    writeRegister(0, 0b00110000);  //0x30

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
    writeRegister(7, 0b01001011);   //0x4B



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


    //setup the AFE
    Serial.println("Starting LMP91000 CO initialization");
    writeLogFile("Starting LMP91000 CO initialization");
    Wire.begin();   //this must be done for the LMP91000
    digitalWrite(lmp91000_1_en, LOW); //enable the chip

    if(lmp91000.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0)
    {
          Serial.println("Couldn't communicate with LMP91000 for CO");
          writeLogFile("Couldn't communicate with LMP91000 for CO");
    }else{
          Serial.println("Initialized LMP91000 for CO");
          writeLogFile("Initialized LMP91000 for CO");
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
    if(Wire.requestFrom(0x49,1) == 0) { //if can't get 1 byte from ADC1, add it to the init error log
      //init_log += "AD1,";
      //digitalWrite(red_status_led, HIGH);
      //delay(200);
      //digitalWrite(red_status_led, LOW);
      //delay(200);
      Serial.println("Could not communicate with Adafruit_ADS1115 for CO");
      writeLogFile("Could not communicate with Adafruit_ADS1115 for CO");
    }
    else{
      ads1.setGain(GAIN_TWOTHIRDS);
    }

    //AFE 2 setup
    #if AFE2_en
    Serial.println("Starting LMP91000 2 initialization");
    writeLogFile("Starting LMP91000 2 initialization");
    Wire.begin();   //this must be done for the LMP91000
    digitalWrite(lmp91000_2_en, LOW); //enable the chip

    if(lmp91000.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0)
    {
          Serial.println("Couldn't communicate with LMP91000 for 2");
          writeLogFile("Couldn't communicate with LMP91000 for 2");
    }else{
          Serial.println("Initialized LMP91000 for 2");
          writeLogFile("Initialized LMP91000 for 2");
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
    if(Wire.requestFrom(0x4A,1) == 0) { //if can't get 1 byte from ADC1, add it to the init error log
      //init_log += "AD1,";
      //digitalWrite(red_status_led, HIGH);
      //delay(200);
      //digitalWrite(red_status_led, LOW);
      //delay(200);
      Serial.println("Could not communicate with Adafruit_ADS1115 for CO");
      writeLogFile("Could not communicate with Adafruit_ADS1115 for CO");
    }
    else{
      ads2.setGain(GAIN_TWOTHIRDS);
    }
    #endif

    if (!bme.begin()) {
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
      writeLogFile("Could not find a valid BME680 sensor, check wiring!");
      //while (1);
    }else{
      Serial.println("Initialized BME Sensor");
      writeLogFile("Initialized BME Sensor");
    }

    if(!t6713.begin()){
      Serial.println("Could not find a valid T6713 sensor, check wiring!");
      writeLogFile("Could not find a valid T6713");
    }
  //Serial.println("before bme setup");
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
//Serial.println("After bme setup");
    //output current time

    //Serial.printf("Time now: %u\n\r", Time.now());
    //Serial.println(Time.timeStr());
    /*Serial.println(String(HEADER_STRING));
    if (sd.begin(CS)){
        file.open(fileName, O_CREAT | O_APPEND | O_WRITE);
        file.println("File Start timestamp: ");
        file.println(Time.timeStr());
        file.println(String(HEADER_STRING));
        file.close();
        file_started = 1;
    }*/
    resetESP();
    Serial.println("ESP reset!");

}

void loop() {
    if(output_only_particles == 1){
        outputParticles();
    }
    //read temp, press, humidity, and TVOCs
    if(debugging_enabled){
      Serial.println("Before reading bme");
      writeLogFile("before reading bme");
    }
    if (! bme.performReading()) {
      Serial.println("Failed to read BME680");
      writeLogFile("Failed to read BME680");
      return;
    }else{
      if(debugging_enabled){
        Serial.printf("Temp=%1.1f, press=%1.1f, rh=%1.1f\n\r", bme.temperature, bme.pressure/100, bme.humidity);
      }
    }

    readGpsStream();


    //read CO values and apply calibration factors
    CO_float = readCO();


    #if AFE2_en
        CO_float_2 = readAlpha2();
    #endif

    //CO_float_2 += CO_zero_2;
    //CO_float_2 *= CO_slope_2;

    CO2_float = readCO2();

    //correct for altitude
    float pressure_correction = bme.pressure/100;
    if(pressure_correction > LOW_PRESSURE_LIMIT && pressure_correction < HIGH_PRESSURE_LIMIT){
        pressure_correction /= SEALEVELPRESSURE_HPA;
        if(debugging_enabled){
            Serial.printf("pressure correction factor for CO2:%1.2f\n\r", pressure_correction);

        }
        CO2_float *= pressure_correction;
    }else{
        Serial.println("Error: Pressure out of range, not using pressure correction for CO2.");
        Serial.printf("Pressure=%1.2f\n\r", pressure_correction);

    }


    if(ozone_enabled){
        readOzone();
    }


    //sound_average = 0;
    calculateAQI();
    sound_average = readSound();
    //read PM values and apply calibration factors
    readPlantower();

    pm_25_correction_factor = PM_25_CONSTANT_A + (PM_25_CONSTANT_B*(readHumidity()/100))/(1 - (readHumidity()/100));
    if(debugging_enabled){
        Serial.printf("pm2.5 correction factor: %1.2f, %1.2f\n\r", pm_25_correction_factor, readHumidity()/100);
    }
    corrected_PM_25 = PM2_5Value * pm_25_correction_factor;

    //getEspWifiStatus();
    outputDataToESP();

    sample_counter = ++sample_counter;
    if(sample_counter == 99)    {
          sample_counter = 0;
    }

    if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        if(debugging_enabled){
            Serial.print("incomming byte:");
            Serial.println(incomingByte);

        }
        Serial.println(incomingByte);
        if(incomingByte == 'm'){
          serialMenu();
        }
    }

    if(serial_cellular_enabled){

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

}

void calculateAQI(void){
    //Calculate humidity contribution to IAQ index
        gas_reference = bme.gas_resistance/100;
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
                    }
                }
                comma_counter++;
            }
        }
    }

}

float readTemperature(void){
    float temperature = 0;
    if(new_temperature_sensor_enabled){
        temperature = analogRead(A1);


        temperature *= VOLTS_PER_UNIT;

        temperature -= TMP36_OFFSET;
        temperature /= TMP36_VPDC;
    }else{
        if(debugging_enabled){
            Serial.println("Temperature reading from BME for Alphasense");

          }
        temperature = bme.temperature;
    }
    //temperature *= 100;
    temperature += temp_zero;       //user input zero offset
    temperature *= temp_slope;

    return temperature;
    //temperature = temperature +
}

float readHumidity(void){
    float humidity = bme.humidity;
    humidity += rh_zero;       //user input zero offset
    humidity *= rh_slope;
    if(humidity > 100)
        humidity = 100;
    return humidity;
    //temperature = temperature +
}
//read sound from
double readSound(void){
    int val;
    double sum = 0;
    float average = 0;
    for(int i=0; i< 10;i++){
        val = analogRead(sound_input);
        sum += val;
        //Serial.print("Sound level: ");
        //Serial.println(val);
    }
    sum = sum/10;
    sum /= 4095;
    sum *= 100;
    return sum;
}
//read Carbon monoxide alphasense sensor
float readCO(void){
    float float_offset;

    CO_float = readAlpha1();
    float_offset = CO_zero;
    float_offset /= 1000;

    CO_float += float_offset;
    CO_float *= CO_slope;

    return CO_float;
}

float readCO2(void){
    //read CO2 values and apply calibration factors
    CO2_float = t6713.readPPM();
    CO2_float += CO2_zero;
    CO2_float *= CO2_slope;
    return CO2_float;
}
float readAlpha1(void){
    //read from CO sensor
    int32_t A0_gas; //gas
    int32_t A1_aux; //aux out
    int32_t A2_temperature; //temperature
    int32_t half_Vref; //half of Vref
    float volt0_gas;
    float volt1_aux;
    float volt2_temperature; //need to code to be able to ensure correct sigfigs..
    float volt_half_Vref;
    float sensorCurrent; // Working Electrode current in microamps (millivolts / Kohms)
    float auxCurrent;
    float correctedCurrent;
    float alpha1_ppmraw;
    String alpha1_ppmRounded;
    if(debugging_enabled){
        Serial.println("Start of alpha read");
    }
    digitalWrite(lmp91000_1_en, LOW);   //enable

    if(Wire.requestFrom(0x49,1) == 0){
      if(debugging_enabled){
        Serial.println("Couldn't communicate with LMP91000");
        writeLogFile("Couldn't communicate with LMP91000");
      }
        //operation_log += "AD1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }else{
        half_Vref = ads1.readADC_SingleEnded(3); //half of Vref
        volt_half_Vref = half_Vref * ads_bitmv;
        if(abs((volt_half_Vref)/1000 - 1.25) > 0.5){
          if(debugging_enabled){
            Serial.printf("Halfvolt: %1.2f\n\r", volt_half_Vref/1000);
            writeLogFile("Halfvolt higher than 0.5");
          }
        }
    }

    if(lmp91000.read(LMP91000_STATUS_REG) == 0){
        if(debugging_enabled){
            Serial.println("Status = 0 from LMP91000 status reg");
            writeLogFile("LMP1000 status = 0");
          }
        //operation_log += "AFE1,";
      //  digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }

    //if(Wire.requestFrom(0x49,1) == 0 || lmp91000.read(LMP91000_STATUS_REG) == 0 || (abs((volt_half_Vref)/1000 - 1.25) > 0.5)){
    if(Wire.requestFrom(0x49,1) == 0 || lmp91000.read(LMP91000_STATUS_REG) == 0){
        alpha1_ppmRounded = "-99";
        volt0_gas = -99;
        volt1_aux = -99;
        volt_half_Vref = -99;
        sensorCurrent = -99;
        auxCurrent = -99;
    }else{
        A0_gas = 0;
        A1_aux = 0;
        A2_temperature = 0;
        half_Vref = 0;
        for(int i=0; i<100; i++){
          A0_gas += ads1.readADC_SingleEnded(0); //gas
          A1_aux += ads1.readADC_SingleEnded(1); //aux out
          A2_temperature += ads1.readADC_SingleEnded(2); //temperature
          half_Vref += ads1.readADC_SingleEnded(3); //half of Vref
        }

        A0_gas = A0_gas / 100;
        A1_aux = A1_aux / 100;
        A2_temperature = A2_temperature / 100;
        half_Vref = half_Vref / 100;

        volt0_gas = A0_gas * ads_bitmv;
        volt1_aux = A1_aux * ads_bitmv;
        volt2_temperature = A2_temperature * ads_bitmv;
        volt_half_Vref = half_Vref * ads_bitmv;

        sensorCurrent = (volt_half_Vref - volt0_gas) / (-1*120); // Working Electrode current in microamps (millivolts / Kohms)
        auxCurrent = (volt_half_Vref - volt1_aux) / (-1*150);
        //{1, -1, -0.76}, //CO-A4 (<=10C, 20C, >=30C)
        if(readTemperature() <= 15){
          correctedCurrent = ((sensorCurrent) - (auxCurrent));
        }
        else if(readTemperature() <= 25){
          correctedCurrent = ((sensorCurrent) - (-1)*(auxCurrent));
        }
        else{
          correctedCurrent = ((sensorCurrent) - (-0.76)*(auxCurrent));
        }
        alpha1_ppmraw = (correctedCurrent / 0.358); //sensitivity .358 nA/ppb - from Alphasense calibration certificate, So .358 uA/ppm
        alpha1_ppmRounded = String(alpha1_ppmraw, 2);
      }

      digitalWrite(lmp91000_1_en, HIGH);  //disable

      if(debugging_enabled){
          Serial.print("CO measurements:  \n\r");
          Serial.printf("A0_gas: %d\n\r", A0_gas);
          Serial.printf("A1_aux: %d\n\r", A1_aux);
          Serial.printf("A2_temp: %d\n\r", A2_temperature);
          Serial.printf("half_vref: %d\n\r", half_Vref);

      }
      return alpha1_ppmraw;
}

float readAlpha2(void){
    //read from CO sensor
    int32_t A0_gas; //gas
    int32_t A1_aux; //aux out
    int32_t A2_temperature; //temperature
    int32_t half_Vref; //half of Vref
    float volt0_gas;
    float volt1_aux;
    float volt2_temperature; //need to code to be able to ensure correct sigfigs..
    float volt_half_Vref;
    float sensorCurrent; // Working Electrode current in microamps (millivolts / Kohms)
    float auxCurrent;
    float correctedCurrent;
    float alpha2_ppmraw;
    String alpha2_ppmRounded;
    if(debugging_enabled){
        Serial.println("Start of alpha 2 read");
    }
    digitalWrite(lmp91000_2_en, LOW);   //enable

    if(Wire.requestFrom(0x4A,1) == 0){
        Serial.println("Couldn't communicate with LMP91000 2");
        //operation_log += "AD1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }else{
        half_Vref = ads2.readADC_SingleEnded(3); //half of Vref
        volt_half_Vref = half_Vref * ads_bitmv;
        if(abs((volt_half_Vref)/1000 - 1.25) > 0.5){
          //operation_log += "AD1_VREF2,";
          //digitalWrite(red_status_led, HIGH);
          //delay(200);
          //digitalWrite(red_status_led, LOW);
          //delay(200);
          Serial.print("half vref2 ads1");
          Serial.println(volt_half_Vref/1000);

        }
    }

    if(lmp91000.read(LMP91000_STATUS_REG) == 0){
        if(debugging_enabled)
            Serial.println("Status == 0 from LMP91000 2 status reg");
        //operation_log += "AFE1,";
      //  digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }

    if(Wire.requestFrom(0x4A,1) == 0 || lmp91000.read(LMP91000_STATUS_REG) == 0 || (abs((volt_half_Vref)/1000 - 1.25) > 0.5)){
        alpha2_ppmRounded = "-99";
        volt0_gas = -99;
        volt1_aux = -99;
        volt_half_Vref = -99;
        sensorCurrent = -99;
        auxCurrent = -99;
    }else{
        A0_gas = 0;
        A1_aux = 0;
        A2_temperature = 0;
        half_Vref = 0;
        for(int i=0; i<100; i++){
          A0_gas += ads2.readADC_SingleEnded(0); //gas
          A1_aux += ads2.readADC_SingleEnded(1); //aux out
          A2_temperature += ads2.readADC_SingleEnded(2); //temperature
          half_Vref += ads2.readADC_SingleEnded(3); //half of Vref
        }

        A0_gas = A0_gas / 100;
        A1_aux = A1_aux / 100;
        A2_temperature = A2_temperature / 100;
        half_Vref = half_Vref / 100;

        volt0_gas = A0_gas * ads_bitmv;
        volt1_aux = A1_aux * ads_bitmv;
        volt2_temperature = A2_temperature * ads_bitmv;
        volt_half_Vref = half_Vref * ads_bitmv;

        sensorCurrent = (volt_half_Vref - volt0_gas) / (-1*120); // Working Electrode current in microamps (millivolts / Kohms)
        auxCurrent = (volt_half_Vref - volt1_aux) / (-1*150);
        //{1, -1, -0.76}, //CO-A4 (<=10C, 20C, >=30C)
        if(readTemperature() <= 15){
          correctedCurrent = ((sensorCurrent) - (auxCurrent));
        }
        else if(readTemperature() <= 25){
          correctedCurrent = ((sensorCurrent) - (-1)*(auxCurrent));
        }
        else if(readTemperature() > 25){
          correctedCurrent = ((sensorCurrent) - (-0.76)*(auxCurrent));
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
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(CO_float, 3);
    csv_output_string += String(CO_float, 3) + ",";
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
    cloud_output_string += String(PM1_PACKET_CONSTANT) + String(PM01Value);
    csv_output_string += String(PM01Value) + ",";
    cloud_output_string += String(PM2PT5_PACKET_CONSTANT) + String(corrected_PM_25, 0);
    csv_output_string += String(corrected_PM_25, 0) + ",";
    cloud_output_string += String(PM10_PACKET_CONSTANT) + String(PM10Value);
    csv_output_string += String(PM10Value) + ",";
    cloud_output_string += String(TEMPERATURE_PACKET_CONSTANT) + String(readTemperature(), 1);
    csv_output_string += String(readTemperature(), 1) + ",";
    cloud_output_string += String(PRESSURE_PACKET_CONSTANT) + String(bme.pressure / 100.0, 1);
    csv_output_string += String(bme.pressure / 100.0, 1) + ",";
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
    if(gps.get_nsIndicator() == 0){
        csv_output_string += "-";
        cloud_output_string += "-";
    }
    csv_output_string += String(gps.get_latitude()) + ",";
    cloud_output_string += String(gps.get_latitude());

    cloud_output_string += String(LONGITUDE_PACKET_CONSTANT);
    if(gps.get_ewIndicator() == 0x01){
        csv_output_string += "-";
        cloud_output_string += "-";
    }
    csv_output_string += String(gps.get_longitude()) + ",";
    cloud_output_string += String(gps.get_longitude());
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
    Serial.println(csv_output_string);

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
            floatBytes.myFloat = CO_float;
        }else if(i == 1){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = CARBON_DIOXIDE_PACKET_CONSTANT;
            floatBytes.myFloat = CO2_float;
        }else if(i == 2){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
            floatBytes.myFloat = fuel.getSoC();
        }else if(i == 3){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
            floatBytes.myFloat = PM01Value;
        }else if(i == 4){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
            floatBytes.myFloat = corrected_PM_25;
        }else if(i == 5){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
            floatBytes.myFloat = PM10Value;
        }else if(i == 6){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = TEMPERATURE_PACKET_CONSTANT;
            floatBytes.myFloat = readTemperature();
        }else if(i == 7){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PRESSURE_PACKET_CONSTANT;
            floatBytes.myFloat = bme.pressure / 100.0;
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

        ble_output_array[19 + i*(BLE_PAYLOAD_SIZE)] = '#';     //delimeter for separating species

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
        if (! bme.performReading()) {
          Serial.println("Failed to read BME680");

        }
        readPlantower();
        readGpsStream();
        CO2_float = t6713.readPPM();

        CO2_float += CO2_zero;
        CO2_float *= CO2_slope;
        //correct for altitude
        float pressure_correction = bme.pressure/100;
        if(pressure_correction > LOW_PRESSURE_LIMIT && pressure_correction < HIGH_PRESSURE_LIMIT){
            pressure_correction /= SEALEVELPRESSURE_HPA;
            CO2_float *= pressure_correction;
        }
        pm_25_correction_factor = PM_25_CONSTANT_A + (PM_25_CONSTANT_B*(readHumidity()/100))/(1 - (readHumidity()/100));
        corrected_PM_25 = PM2_5Value * pm_25_correction_factor;

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
                floatBytes.myFloat = PM01Value;
            }else if(i == 2){
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
                floatBytes.myFloat = corrected_PM_25;
            }else if(i == 3){
                ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
                floatBytes.myFloat = PM10Value;
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
//read from plantower pms 5500
void readPlantower(void){
    if(Serial4.find("B")){    //start to read when detect 0x42
        //if(debugging_enabled)
          //Serial.println("Found a B when reading plantower");
          Serial4.readBytes(buf,LENG);
          if(buf[0] == 0x4d){
              if(checkValue(buf,LENG)){ //All units are ug/m^3
                  //Serial.println("Value is good from pm buff");
                  PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
                  PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
                  PM10Value=transmitPM10(buf); //count PM10 value of the air detector module
              }
          }
      }
      else{
        //Serial.println("Clearing serial buffer from PM measurement");
        while(Serial4.available()){
            char clearBuffer = Serial4.read();
            //Serial.print(clearBuffer);
        }
      }
}
char checkValue(char *thebuf, char leng)  {
    char receiveflag=0;
    int receiveSum=0;

    for(int i=0; i<(leng-2); i++) {
      receiveSum=receiveSum+thebuf[i];
    }
    receiveSum=receiveSum + 0x42;

    if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1])) { //check the serial data
      receiveSum = 0;
      receiveflag = 1;
    }
    return receiveflag;
}
int transmitPM01(char *thebuf)  {
    int PM01Val;
    PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
    return PM01Val;
}
//transmit PM Value to PC
float transmitPM2_5(char *thebuf) {
    float PM2_5Val;
    PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
    return PM2_5Val;
}
//transmit PM Value to PC
int transmitPM10(char *thebuf)  {
    int PM10Val;
    PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module
    return PM10Val;
}

void goToSleep(void){
  //Serial.println("Going to sleep:)");
  digitalWrite(power_led_en, LOW);
  digitalWrite(plantower_en, LOW);
  digitalWrite(esp_wroom_en, LOW);
  digitalWrite(blower_en, LOW);
  digitalWrite(co2_en, LOW);
  System.sleep(D4,FALLING);
  delay(500);
  System.reset();
  //detachInterrupt(D3);
}

void resetESP(void){
  digitalWrite(esp_wroom_en, LOW);
  digitalWrite(plantower_en, LOW);
  digitalWrite(blower_en, LOW);
  digitalWrite(co2_en, LOW);
  delay(1000);
  digitalWrite(esp_wroom_en, HIGH);
  digitalWrite(plantower_en, HIGH);
  digitalWrite(blower_en, HIGH);
  digitalWrite(co2_en, HIGH);
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
            if (! bme.performReading()) {
              Serial.println("Failed to read BME680");
              return;
            }else{
                Serial.printf("TVocs=%1.0f, Temp=%1.1f, press=%1.1f, rh=%1.1f\n\r", bme.gas_resistance/100, bme.temperature, bme.pressure, bme.humidity);
            }
        }

    }else if(incomingByte == '?'){
        outputSerialMenuOptions();
    }
  }
  Serial.println("Exiting serial menu...");

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
    Serial.print("Current Average: ");
    Serial.print(measurements_to_average);
    Serial.println(" 5 second measurements");
    Serial.print("Enter new amount\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= 1 && tempValue < 1000){
        Serial.print("\n\rNew average time: ");
        Serial.println(tempValue);
        Serial.println(" 5 second measurements");
        measurements_to_average = tempValue;
        EEPROM.put(MEASUREMENTS_TO_AVG_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCo2Slope(void){

    Serial.println();
    Serial.print("Current CO2 slope:");
    Serial.print(String(CO2_slope, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO2 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        CO2_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew CO2 slope: ");
        Serial.println(String(CO2_slope,2));

        EEPROM.put(CO2_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCo2Zero(void){
    Serial.println();
    Serial.print("Current CO2 zero:");
    Serial.print(CO2_zero);
    Serial.println(" ppm");
    Serial.print("Enter new CO2 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew CO2 zero: ");
        Serial.println(tempValue);
        CO2_zero = tempValue;
        EEPROM.put(CO2_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoSlope(void){

    Serial.println();
    Serial.print("Current CO slope:");
    Serial.print(String(CO_slope, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        CO_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew CO slope: ");
        Serial.println(String(CO_slope,2));

        EEPROM.put(CO_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetCoZero(void){
    Serial.println();
    Serial.print("Current CO zero:");
    Serial.print(CO_zero);
    Serial.println(" ppb");
    Serial.print("Enter new CO Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -5000 && tempValue < 5000){
        Serial.print("\n\rNew CO zero: ");
        Serial.println(tempValue);
        CO_zero = tempValue;
        EEPROM.put(CO_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm1Slope(void){
    Serial.println();
    Serial.print("Current PM1 slope:");
    Serial.print(String(PM_1_slope, 2));
    Serial.println(" ");
    Serial.print("Enter new PM1 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        PM_1_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew PM1 slope: ");
        Serial.println(String(PM_1_slope, 2));

        EEPROM.put(PM_1_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm1Zero(void){
    Serial.println();
    Serial.print("Current PM1 zero:");
    Serial.print(PM_1_zero);
    Serial.println(" ug/m3");
    Serial.print("Enter new PM1 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew PM1 zero: ");
        Serial.println(tempValue);
        PM_1_zero = tempValue;
        EEPROM.put(PM_1_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm25Slope(void){
    Serial.println();
    Serial.print("Current PM2.5 slope:");
    Serial.print(String(PM_25_slope, 2));
    Serial.println(" ");
    Serial.print("Enter new PM2.5 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        PM_25_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew PM2.5 slope: ");
        Serial.println(String(PM_25_slope,2));

        EEPROM.put(PM_25_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm25Zero(void){
    Serial.println();
    Serial.print("Current PM2.5 zero:");
    Serial.print(PM_25_zero);
    Serial.println(" ug/m3");
    Serial.print("Enter new PM2.5 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew PM2.5 zero: ");
        Serial.println(tempValue);
        PM_25_zero = tempValue;
        EEPROM.put(PM_25_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm10Slope(void){
    Serial.println();
    Serial.print("Current PM10 slope:");
    Serial.print(String(PM_10_slope, 2));
    Serial.println(" ");
    Serial.print("Enter new PM10 slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        PM_10_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew PM10 slope: ");
        Serial.println(String(PM_10_slope,2));

        EEPROM.put(PM_10_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPm10Zero(void){
    Serial.println();
    Serial.print("Current PM10 zero:");
    Serial.print(PM_10_zero);
    Serial.println(" um/m3");
    Serial.print("Enter new PM10 Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew PM10 zero: ");
        Serial.println(tempValue);
        PM_10_zero = tempValue;
        EEPROM.put(PM_10_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetTemperatureSlope(void){
    Serial.println();
    Serial.print("Current Temperature slope:");
    Serial.print(String(temp_slope, 2));
    Serial.println(" Degrees C");
    Serial.print("Enter new Temperature slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        temp_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew Temperature slope: ");
        Serial.println(String(temp_slope,2));

        EEPROM.put(TEMP_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetTemperatureZero(void){
    Serial.println();
    Serial.print("Current Temperature zero:");
    Serial.print(temp_zero);
    Serial.println(" Degrees C");
    Serial.print("Enter new Temperature Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -30 && tempValue < 30){
        Serial.print("\n\rNew Temperature zero: ");
        Serial.println(tempValue);
        temp_zero = tempValue;
        EEPROM.put(TEMP_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPressureSlope(void){
    Serial.println();
    Serial.print("Current Pressure slope:");
    Serial.print(String(pressure_slope, 2));
    Serial.println(" torr");
    Serial.print("Enter new Pressure slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        pressure_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew Pressure slope: ");
        Serial.println(String(pressure_slope,2));

        EEPROM.put(PRESSURE_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetPressureZero(void){
    Serial.println();
    Serial.print("Current Pressure zero:");
    Serial.print(pressure_zero);
    Serial.println(" ppm");
    Serial.print("Enter new Pressure Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("\n\rNew Pressure zero: ");
        Serial.println(tempValue);
        pressure_zero = tempValue;
        EEPROM.put(PRESSURE_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetHumiditySlope(void){
    Serial.println();
    Serial.print("Current RH slope:");
    Serial.print(String(rh_slope, 2));
    Serial.println(" %");
    Serial.print("Enter new RH slope\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 10){
        rh_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("\n\rNew RH slope: ");
        Serial.println(String(rh_slope,2));

        EEPROM.put(RH_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("\n\rInvalid value!");
    }
}

void serialGetHumidityZero(void){
    Serial.println();
    Serial.print("Current RH zero:");
    Serial.print(rh_zero);
    Serial.println(" %");
    Serial.print("Enter new RH Zero\n\r");
    Serial.setTimeout(50000);
    String tempString = Serial.readStringUntil('\r');
    int tempValue = tempString.toInt();

    if(tempValue >= -50 && tempValue < 50){
        Serial.print("\n\rNew RH zero: ");
        Serial.println(tempValue);
        rh_zero = tempValue;
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
    while(!Serial.available()){
        CO_float = readCO();
        Serial.printf("CO: %1.3f ppm\n\r", CO_float);
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
    Serial.println("A:  Ouptput CO constantly and rapidly");
    Serial.println("B:  Output PM constantly and rapidly");
    Serial.println("C:  Change temperature units to Celcius");
    Serial.println("D:  Disable TMP36 temperature sensor and use BME680 temperature");
    Serial.println("E:  Enable TMP36 temperature sensor and disable BME680 temperature");
    Serial.println("F:  Change temperature units to Farenheit");
    Serial.println("G:  Read ozone from analog input (not digitally - board dependent)");
    Serial.println("H:  Read ozone digitally (not through analog input - board dependent)");
    Serial.println("I:  Adjust average time for uploading");
    Serial.println("J:  Reset ESP, CO2, Plantower");
    Serial.println("K:  Continuous serial output of GPS");
    Serial.println("!:  Continuous serial output of VOC's");
    Serial.println("?:  Output this menu");
    Serial.println("x:  Exits this menu");
  }
