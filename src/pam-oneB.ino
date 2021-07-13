
#include "Serial4/Serial4.h"
#include "Serial5/Serial5.h"
#include "gps.h"
#include "inttypes.h"
#include "Particle.h"
#include "PowerCheck.h"
#include "../lib/SdFat/src/SdFat/SdFat.h"
#include "CellularHelper.h"

#include "Wiring.h"

#include "PAMSerial/PAMSerial.h"
#include "PAMSerial/PAMSerialMenu/PAMSerialMenu.h"
#include "PAMSerial/PAMSerialEditEEPROMValue/PAMSerialEditEEPROMValue.h"
#include "PAMSensorManager/PAMSensorManager.h"

#include "Sensors/T6713/T6713.h"
#include "Sensors/TPHFusion/TPHFusion.h"
#include "Sensors/Plantower/Plantower.h"
#include "Sensors/PAMCO/PAMCO.h"
#include "Sensors/108L/108L.h"

#include "SendingData/SendingData.h"

#define APP_VERSION 8
#define BUILD_VERSION 1

#define CELCIUS 1
#define FARENHEIT 0



//enable or disable different parts of the firmware by setting the following values to 1 or 0
#define sd_en 1

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
//#define VOC_PACKET_CONSTANT 'g'             //VOCs as IAQ
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

#define HEADER_STRING "DEV,CO(ppm),CO2(ppm),PM1,PM2_5,PM10,T(C),Press(mBar),RH(%),O3(ppb),Batt(%),Latitude,Longitude,N/A,N/A,Date/Time"


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
PAM_108L pam_108L;

SendingData send_measurements();

time_t watch_time;


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
int abc_logic_enabled = 0;
bool tried_cellular_connect = false;
int battery_threshold_enable;
int CO_socket = 0;

char geolocation_latitude[12] = "999.9999999";
char geolocation_longitude[13] = "99.9999999";
char geolocation_accuracy[6] = "255.0";

int averaging_time = 0;


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
    PAMSerial.printf(rd, "Main received Data!\n\r");
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




//test for setting up PMIC manually
void writeRegister(uint8_t reg, uint8_t value) {
    // This would be easier if pmic.writeRegister wasn't private
    Wire3.beginTransmission(PMIC_ADDRESS);
    Wire3.write(reg);
    Wire3.write(value);
    Wire3.endTransmission(true);

}

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
    // EEPROM.get(VOC_EN_MEM_ADDRESS, voc_enabled);
    EEPROM.get(GAS_LOWER_LIMIT_MEM_ADDRESS, gas_lower_limit);
    EEPROM.get(GAS_UPPER_LIMIT_MEM_ADDRESS, gas_upper_limit);
    EEPROM.get(TIME_ZONE_MEM_ADDRESS, tempValue);
    Time.zone(tempValue);
    EEPROM.get(TEMPERATURE_UNITS_MEM_ADDRESS, temperature_units);
    EEPROM.get(OUTPUT_PARTICLES_MEM_ADDRESS, output_only_particles);
    EEPROM.get(TEMPERATURE_SENSOR_ENABLED_MEM_ADDRESS, new_temperature_sensor_enabled);
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
    // EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
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
    //initialize main serial port for debug output
    Serial.begin(9600);

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

    PAMSerialEditEEPROMValue<int> * averaging_measurement = new PAMSerialEditEEPROMValue<int>(averaging_time, MEASUREMENTS_TO_AVG_MEM_ADDRESS, 300);

    PAMSensorManager *manager = PAMSensorManager::GetInstance();
    manager->addSensor(&t6713);
    manager->addSensor(&tph_fusion);
    manager->addSensor(&plantower);
    manager->addSensor(&pamco);
    manager->addSensor(&pam_108L);
    serial_menu.addResponder(PAMSensorManager::GetInstance()->serial_menu_rd, "Sensor Settings");
    serial_menu.addResponder(averaging_measurement, "Set Averaging Time (seconds)");

    char *csv_header = manager->csvHeader();
    Serial.println(csv_header);
    free(csv_header);

    
    Log.info("System version: %s", (const char*)System.version());

    watch_time = Time.now();

}

void loop() {
    if (Time.now() > watch_time+averaging_time)
    {
        manager->runAllAverages();
        if (serial_cellular_enabled)
        {

        }
        
        watch_time = Time.now();
    }


    PAMSensorManager::GetInstance()->loop();
    PAMSerial.loop();

    if(serial_cellular_enabled)
    {
        status_word.status_int |= 0x01;
        if (Particle.connected() == false && tried_cellular_connect == false) 
        {
            tried_cellular_connect = true;
            Cellular.on();
            Particle.connect();
        }
        else if(Particle.connected() == true){  //this means that it is already connected
            tried_cellular_connect = false;
        }
    }
    else if (Particle.connected() == true) {
          Cellular.off();
    }

    delay(2000);














    // //getEspWifiStatus();

    // // Ask Craig or david about this. Where is this sample_counter shown in the esp app? I can't find this counter. I think it is not needed.
    // sample_counter = ++sample_counter;
    // if(sample_counter == 99)    {
    //       sample_counter = 0;
    // }

    // if(serial_cellular_enabled){
    //     status_word.status_int |= 0x01;
    //   if (Particle.connected() == false && tried_cellular_connect == false) {
    //     tried_cellular_connect = true;
    //       Cellular.on();
    //       Particle.connect();
    //   }else if(Particle.connected() == true){  //this means that it is already connected
    //     tried_cellular_connect = false;
    //   }
    // }else{
    //   if (Particle.connected() == true) {
    //       Cellular.off();
    //   }
    // }

    // //check power
    // powerCheck.loop();

	// //Serial.printf("hasPower=%d hasBattery=%d isCharging=%d\n\r", powerCheck.getHasPower(), powerCheck.getHasBattery(), powerCheck.getIsCharging());
    // if((battery_threshold_enable == 1) && (fuel.getSoC() < BATTERY_THRESHOLD) && (powerCheck.getHasPower() == 0)){
    //     Serial.println("Going to sleep because battery is below 20% charge");
    //     goToSleepBattery();
    // }


    // //send_measurement->output_data_to_esp();
    // //send_measurement->output_data_to_sd();

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
    if(incomingByte == 'q'){
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
    }
    else if(incomingByte == 'R'){
        if(abc_logic_enabled){
            Serial.println("Disabling ABC logic for CO2 sensor");
            abc_logic_enabled = 0;
            EEPROM.put(ABC_ENABLE_MEM_ADDRESS, abc_logic_enabled);
            // t6713.disableABCLogic();
            t6713._t6713.disableABCLogic();
        }
        else{
            Serial.println("ABC logic already disabled");
        }

    }
    else if(incomingByte == 'S'){
        if(!abc_logic_enabled){
            Serial.println("Enabling abc logic for CO2 sensor");
            abc_logic_enabled = 1;
            EEPROM.put(ABC_ENABLE_MEM_ADDRESS, abc_logic_enabled);
            // t6713.enableABCLogic();
            t6713._t6713.enableABCLogic();

        }
        else{
            Serial.println("ABC logic already enabled");
        }
    }
    else if(incomingByte == 'T'){
        if (!hih8120_enabled) {
            Serial.println("Enabling HIH8120 RH sensor");
            hih8120_enabled = 1;
            tph_fusion.enable_hih();
            EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, hih8120_enabled);

        } 
        else {
            Serial.println("Disabling HIH8120 RH sensor");
            hih8120_enabled = 0;
            tph_fusion.disable_hih();
            EEPROM.put(HIH8120_ENABLE_MEM_ADDRESS, hih8120_enabled);
        }

    }
    else if(incomingByte == 'U'){
        if(!CO_socket){
            Serial.println("Now reading CO from U20-Alpha2");
            CO_socket = 1;
            EEPROM.put(CO_SOCKET_MEM_ADDRESS, CO_socket);

        }
        else{
            Serial.println("Now reading CO from U19-Alpha1");
            CO_socket = 0;
            EEPROM.put(CO_SOCKET_MEM_ADDRESS, CO_socket);
        }
    }
    else if(incomingByte == 'V'){
        Serial.println("Reseting the CO2 sensor");
        // t6713.resetSensor();
        t6713._t6713.resetSensor();
    }
    else if(incomingByte == '3'){
        Serial.print("APP Version: ");
        Serial.println(APP_VERSION);
        Serial.print("Build: ");
        Serial.println(BUILD_VERSION);
    }
    else if(incomingByte == '4'){
        if(ozone_enabled == 0){
            Serial.println("Enabling Ozone");
        }
        else{
            Serial.println("Ozone already enabled");
        }
        ozone_enabled = 1;
        EEPROM.put(OZONE_EN_MEM_ADDRESS, ozone_enabled);
    }
    else if(incomingByte == '5'){
        if(ozone_enabled == 1){
            Serial.println("Disabling Ozone");
        }
        else{
            Serial.println("Ozone already disabled");
        }
        ozone_enabled = 0;
        EEPROM.put(OZONE_EN_MEM_ADDRESS, ozone_enabled);
    }
    // else if(incomingByte == '6'){
    //     if(voc_enabled == 0){
    //         Serial.println("Enabling VOC's");
    //     }
    //     else{
    //         Serial.println("VOC's already enabled");
    //     }
    //     voc_enabled = 1;
    //     EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
    // }
    // else if(incomingByte == '7'){
    //     if(voc_enabled == 1){
    //         Serial.println("Disabling VOC's");
    //     }else{
    //         Serial.println("VOC's already disabled");
    //     }
    //     voc_enabled = 0;
    //     EEPROM.put(VOC_EN_MEM_ADDRESS, voc_enabled);
    // }
    else if(incomingByte == '8'){
        Serial.print("Fault: ");
        byte fault = pmic.getFault();
        Serial.println(fault);
        Serial.print("System status: ");
        byte systemStatus = pmic.getSystemStatus();
        Serial.println(systemStatus);

    }
    else if(incomingByte == '9'){
        serialIncreaseChargeCurrent();
    }
    else if(incomingByte == '0'){
        serialIncreaseInputCurrent();
    }
    else if(incomingByte == 'B'){
        if(output_only_particles == 1){
            output_only_particles = 0;
            Serial.println("Outputting normally");
        }
        else{
            output_only_particles = 1;
            Serial.println("Outputting only PM");
        }
        EEPROM.put(OUTPUT_PARTICLES_MEM_ADDRESS, output_only_particles);

    }
    // else if(incomingByte == '!'){

    //     Serial.println("Outputting VOCs continuously!  Press any button to exit...");
    //     while(!Serial.available()){
    //         // if (! bme.performReading()) {
    //         if (! tph_fusion.measure()) {
    //           Serial.println("Failed to read BME680");
    //           return;
    //         }else{
    //             // Serial.printf("TVocs=%1.0f, Temp=%1.1f, press=%1.1f, rh=%1.1f\n\r", bme.gas_resistance/100, bme.temperature, bme.pressure, bme.humidity);
    //             Serial.printf("TVocs=%1.0f, Temp=%1.1f, press=%1.1f, rh=%1.1f\n\r", tph_fusion.voc->adj_value, tph_fusion.temperature->adj_value, tph_fusion.pressure->adj_value, tph_fusion.humidity->adj_value);
    //         }
    //     }
    // }
    else if(incomingByte == 'X'){
        //calibrate CO2 sensor
        //if(debugging_enabled){
            // t6713.calibrate(1);
            t6713._t6713.calibrate(1);
        //}else{
         //   t6713.calibrate(0);
        //}
        
        co2_calibration_timer = 180;        //6 minutes if measurement cycle is 2 seconds
        
    
    }
    else if(incomingByte == 'Z'){
        Serial.println("Getting cellular information, this may take a while...");

        Log.info("IMEI=%s", CellularHelper.getIMEI().c_str());

        Log.info("IMSI=%s", CellularHelper.getIMSI().c_str());

        Log.info("ICCID=%s", CellularHelper.getICCID().c_str());
        //if(serial_cellular_enabled){

        //}else{
        //    Serial.println("Cellular not enabled.  Please enable cellular first!");
        //}
    }
    else if(incomingByte == '?'){
        outputSerialMenuOptions();
    }
  }
  Serial.println("Exiting serial menu...");

}


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




void outputSerialMenuOptions(void){
    Serial.println("Command:  Description");
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
    // Serial.println("6:  Enable VOC's");
    // Serial.println("7:  Disable VOC's");
    Serial.println("8:  Output the PMIC system configuration");
    Serial.println("9:  Increase the charge current by 64 mA");
    Serial.println("0:  Increase the current input limit by 100 mA");
    Serial.println("A:  Ouptput CO constantly and rapidly");
    Serial.println("B:  Output PM constantly and rapidly");
    Serial.println("C:  Change temperature units to Celcius");
    Serial.println("D:  Disable TMP36 temperature sensor and use BME680 temperature");
    Serial.println("E:  Enable TMP36 temperature sensor and disable BME680 temperature");
    Serial.println("F:  Change temperature units to Farenheit");
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
    Serial.println("X:  Calibrate CO2 sensor - must supply ambient level (go outside!)");
    Serial.println("Z:  Output cellular information (ICCID, IMEI, etc)");
    // Serial.println("!:  Continuous serial output of VOC's");
    Serial.println("?:  Output this menu");
    Serial.println("x:  Exits this menu");
}
