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
//#include "Serial1/Serial1.h"
#include "SdFat.h"

//define constants
#define SEALEVELPRESSURE_HPA (1013.25)
#define LOW_PRESSURE_LIMIT (100)
#define HIGH_PRESSURE_LIMIT (1500)
#define VOLTS_PER_UNIT (0.0008)   //3.3V/4096  3.3 is the adc reference voltage and the adc is 12 bits or 4096
#define VOLTS_PER_PPB (0.0125)  //2.5V/200 ppb this is what you divide the voltage reading by to get ppb in ozone if the ozone monitor is set to 2.5V=200ppb

float ads_bitmv = 0.1875; //Bits per mV at defined bit resolution, used to convert ADC value to voltage

//enable or disable different parts of the firmware by setting the following values to 1 or 0
#define sd_en 1

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

//max and min values
#define MIN_DEVICE_ID_NUMBER 1
#define MAX_DEVICE_ID_NUMBER 9999

#define MEASUREMENTS_TO_AVERAGE 1       //change to 30 for 3 minute uploads

//gps sentence
#define TIME_FIELD_INDEX 0
#define LATITUDE_FIELD_INDEX 1
#define NORTH_SOUTH_FIELD_INDEX 2
#define LONGITUDE_FIELD_INDEX 3
#define EAST_WEST_FIELD_INDEX 4
#define GPS_QUALITY_FIELD_INDEX 5
#define NUMBER_OF_SATELLITES_INDEX 6
#define HORZONTAL_DILLUTION_INDEX 7
#define ALTITUDE_FIELD_INDEX 8

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
#define PRESSURE_PACKET_CONSTANT 'P'        //pressure as MILLIBARS
#define HUMIDITY_PACKET_CONSTANT 'h'        //humidity as PERCENTAGE
#define SOUND_PACKET_CONSTANT 's'           //sound as DECIBELS
#define LATITUDE_PACKET_CONSTANT 'a'        //Latitude as DEGREES
#define LONGITUDE_PACKET_CONSTANT 'o'       //Longitude as DEGREES
#define PARTICLE_TIME_PACKET_CONSTANT 'Y'   //result of now()
#define OZONE_PACKET_CONSTANT 'O'           //Ozone
#define BATTERY_PACKET_CONSTANT 'B'         //Battery in percentage


#define NUMBER_OF_SPECIES 12    //total number of species (measurements) being output


//define pin functions
//fix these so they are more consistent!
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define CS A2               //Chip select for SPI/uSD card
#define SLEEP_EN D3

int lmp91000_1_en = B0;     //enable line for the lmp91000 AFE chip for measuring CO
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

//global objects
Adafruit_BME680 bme; // I2C
Telaire_T6713 t6713;  //CO2 sensor
LMP91000 lmp91000;
Adafruit_ADS1115 ads1(0x49); //Set I2C address of ADC1
FuelGauge fuel;

//sdcard
SdFat sd;
SdFile file;
File file1;
String fileName;

//wifi
String ssid; //wifi network name
String password; //wifi network password

//global variables
int counter = 0;
float CO_float = 0;
float CO2_float = 0;
int CO2_value = 0;
float O3_float = 0;
int DEVICE_id = 555;       //default value
int sample_counter = 0;
float tempfloat = 0;
int tempValue;
float air_quality_score = 0;
int esp_wifi_connection_status = 0;


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
float temp_slope;
int temp_zero;
float pressure_slope;
int pressure_zero;
float rh_slope;
int rh_zero;

//serial menu variables
int addr;
uint16_t value;
char recieveStr[5];

//plantower PMS5003 vars
int PM01Value=0;
int PM2_5Value=0;
int PM10Value=0;
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
void check_cal_file(void);
size_t readField(File* file, char* str, size_t size, const char* delim);
void check_wifi_file(void);
void serialMenu();
void serial_get_device_id(void);
void serial_get_co2_zero(void);
void serial_get_co2_zero(void);
void serial_get_co_zero(void);
void serial_get_co_zero(void);
void output_serial_menu_options(void);
void output_to_cloud(void);
void echo_gps();

class GPS {
    double utc_time;
    double latitude;
    double longitude;
    float altitude;
    int16_t latWhole;
    int16_t latFrac;
    int16_t longWhole;
    int16_t longFrac;
    int8_t ns_indicator;              //north south
    int8_t ew_indicator;              //'E' = east, 'W' = west
    int quality_indicator;          //0-2
    int satellites_used;            //0-24
    float horizontal_dillution;     //00.0-99.9

public:
    //(d)dd + (mm.mmmm/60) (* -1 for W and S)
    void set_lat_decimal(String latString, char nsString);
    void set_long_decimal(String longString, char ewString);
    double get_latitude(void);
    double get_longitude(void);
    void set_satellites(String satString);
    void set_horizontalDillution(String hdString);
    int get_satellites(void);
    int get_horizontalDillution(void);
    int16_t get_latitudeWhole(void);
    int16_t get_latitudeFrac(void);
    int16_t get_longitudeWhole(void);
    int16_t get_longitudeFrac(void);
    int8_t get_nsIndicator(void);
    int8_t get_ewIndicator(void);
};

//set decimal value of latitude from NMEA string
void GPS::set_lat_decimal(String latString, char nsString){
    String whole_str = latString.substring(0,2);
    String frac_str = latString.substring(2,10);
    latWhole = whole_str.toInt();
    latFrac = frac_str.toInt();

    int whole_part = whole_str.toInt();
    //Serial.print("Whole part:");
    //Serial.println(whole_part);

    double frac_part = frac_str.toFloat();
    //Serial.print("Frac part:");
    //Serial.println(frac_part, 5);


    latitude = whole_part;
    latitude += (frac_part)/60;
    if(nsString == 'S'){
        ns_indicator = 0;
    }else{
        ns_indicator = 0x80;
    }
}

void GPS::set_long_decimal(String longString, char ewString){
    String whole_str = longString.substring(0,3);
    String frac_str = longString.substring(3,10);

    longWhole = whole_str.toInt();
    longFrac = frac_str.toInt();
    int whole_part = whole_str.toInt();
    //Serial.print("Whole string: ");
    //Serial.println(whole_str);
    //Serial.print("Whole part:");
    //Serial.println(whole_part);

    double frac_part = frac_str.toFloat();
    //Serial.print("Frac part:");
    //Serial.println(frac_part, 5);


    longitude = whole_part;
    longitude += (frac_part)/60;
    if(ewString == 'E'){
      ew_indicator = 0;
    }else{
      ew_indicator = 0x01;
    }
}

void GPS::set_satellites(String satString){
    satellites_used = satString.toInt();
}

void GPS::set_horizontalDillution(String hdString){
    float temp_float = hdString.toFloat();
    temp_float *= 10;

    horizontal_dillution = temp_float;
}

double GPS::get_latitude(void){
    return latitude;
}

double GPS::get_longitude(void){
    return longitude;
}

int GPS::get_satellites(void){
    return satellites_used;
}

int GPS::get_horizontalDillution(void){
    return horizontal_dillution;
}

int16_t GPS::get_latitudeWhole(void){
    return latWhole;
}
int16_t GPS::get_latitudeFrac(void){
    return latFrac;
}
int16_t GPS::get_longitudeWhole(void){
    return longWhole;
}
int16_t GPS::get_longitudeFrac(void){
    return longFrac;
}

int8_t GPS::get_nsIndicator(void){
    return ns_indicator;
}

int8_t GPS::get_ewIndicator(void){
    return ew_indicator;
}
GPS gps;

void output_to_cloud(String data){
    String webhook_data = " ";
    CO_sum += CO_float;
    CO2_sum += CO2_float;
    O3_sum += O3_float;
    measurement_count++;

    if(measurement_count == MEASUREMENTS_TO_AVERAGE){
        CO_sum /= MEASUREMENTS_TO_AVERAGE;
        CO2_sum /= MEASUREMENTS_TO_AVERAGE;
        O3_sum /= MEASUREMENTS_TO_AVERAGE;

        measurement_count = 0;
        String webhook_data = String(DEVICE_id) + ",VOC: " + String(bme.gas_resistance / 1000.0, 1) + ", CO: " + CO_sum + ", CO2: " + CO2_sum + ", PM1: " + PM01Value + ",PM2.5: " + PM2_5Value + ", PM10: " + PM10Value + ",Temp: " + String(bme.temperature, 1) + ",Press: ";
        webhook_data += String(bme.pressure / 100.0, 1) + ",HUM: " + String(bme.humidity, 1) + ",Snd: " + String(sound_average) + ",O3: " + O3_sum + "\n\r";

        if(Particle.connected() && digitalRead(cellular_en)){
            Particle.publish("airdb-camconf", data, PRIVATE);
            Particle.process(); //attempt at ensuring the publish is complete before sleeping
            Serial.println("Published data!");
        }else{
            Serial.println("Couldn't connect to particle");
        }
        CO_sum = 0;
        CO2_sum = 0;
        O3_sum = 0;
    }
}

//read all eeprom stored variables
void readStoredVars(void)
{
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
    EEPROM.get(PM_1_SLOPE_MEM_ADDRESS, PM_1_slope);
    PM_1_slope /= 100;
    EEPROM.get(PM_25_SLOPE_MEM_ADDRESS, PM_25_slope);
    PM_25_slope /= 100;
    EEPROM.get(PM_10_SLOPE_MEM_ADDRESS, PM_10_slope);
    PM_10_slope /= 100;

    EEPROM.get(CO2_ZERO_MEM_ADDRESS, CO2_zero);
    EEPROM.get(CO_ZERO_MEM_ADDRESS, CO_zero);
    EEPROM.get(PM_1_ZERO_MEM_ADDRESS, PM_1_zero);
    EEPROM.get(PM_25_ZERO_MEM_ADDRESS, PM_25_zero);
    EEPROM.get(PM_10_ZERO_MEM_ADDRESS, PM_10_zero);

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

void check_cal_file(void)
{

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

void check_wifi_file(void)
{
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

    //setup i/o
    pinMode(lmp91000_1_en, OUTPUT);
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
    digitalWrite(power_led_en, HIGH);
    digitalWrite(plantower_en, HIGH);
    digitalWrite(esp_wroom_en, HIGH);
    digitalWrite(blower_en, HIGH);
    digitalWrite(co2_en, HIGH);

    //read all stored variables (calibration parameters)
    readStoredVars();
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
    delay(5000);
    //initialize main serial port for debug output
    Serial.begin(9600);


    #if sd_en
    Serial.println("Checking for sd card");

    if (sd.begin(CS)) { //if uSD is functioning and MCP error has not been logged yet.
      file.open("log.txt", O_CREAT | O_APPEND | O_WRITE);
      file.remove();
      file.open("log.txt", O_CREAT | O_APPEND | O_WRITE);
      init_log += "MCP,";
      file.print(init_log);
      file.close();

      //look for a wifi file
      check_wifi_file();
      //look for a calibration file
      check_cal_file();
    }else { //uSD is not functioning
        Serial.println("MCP error log fail. Particle LED should be flashing");
    }
    #endif


    //setup the AFE
    Serial.println("Starting LMP91000 initialization");
    Wire.begin();   //this must be done for the LMP91000
    digitalWrite(lmp91000_1_en, LOW); //enable the chip

    if(lmp91000.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0)
    {
          Serial.println("Couldn't communicate with LMP91000");
    }else{
          Serial.println("Initialized LMP91000");
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
      Serial.println("Could not communicate with Adafruit_ADS1115");
    }
    else{
      ads1.setGain(GAIN_TWOTHIRDS);
    }

    if (!bme.begin()) {
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
      //while (1);
    }

    if(!t6713.begin()){
      Serial.println("Could not find a valid T6713 sensor, check wiring!");
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    //output current time

    Serial.printf("Time now: %u\n", Time.now());

}

void loop() {

    //read temp, press, humidity, and TVOCs
    if (! bme.performReading()) {
      Serial.println("Failed to read BME680");
      return;
    }
    //read_gps();
    read_gps_stream();

    //read CO values and apply calibration factors
    CO_float = read_alpha1();
    CO_float += CO_zero;
    CO_float *= CO_slope;

    //read CO2 values and apply calibration factors
    CO2_float = t6713.readPPM();
    CO2_float += CO2_zero;
    CO2_float *= CO2_slope;
    //correct for altitude
    float pressure_correction = bme.pressure/100;
    if(pressure_correction > LOW_PRESSURE_LIMIT && pressure_correction < HIGH_PRESSURE_LIMIT){
        CO2_float *= pressure_correction/SEALEVELPRESSURE_HPA;
    }else{
        Serial.println("Error: Pressure out of range, not using pressure correction for CO2.");
        Serial.printf("Pressure=%1.2f\n\r", pressure_correction);
    }


    tempValue = analogRead(A0);  // read the analogPin for ozone voltage
    O3_float = tempValue;
    O3_float *= VOLTS_PER_UNIT;           //convert digital reading to voltage
    O3_float /= VOLTS_PER_PPB;            //convert voltage to ppb of ozone


    sound_average = 0;
    calculate_AQI();
    //sound_average = read_sound();
    //read PM values and apply calibration factors
    readPlantower();
    getEspWifiStatus();
    outputDataToESP();

    sample_counter = ++sample_counter;
    if(sample_counter == 99)    {
          sample_counter = 0;
    }

    if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        Serial.println(incomingByte);
        if(incomingByte == 'm'){
          serialMenu();
        }
    }

    if(digitalRead(cellular_en)){
        Serial.println("Cellular is enabled");
      if (Particle.connected() == false) {
          Serial.println("Connecting to cellular network");
          Cellular.on();
          Particle.connect();
      }
    }else{
        Serial.println("Cellular is disabled");
      if (Particle.connected() == true) {
          Serial.println("Disconnecting from cellular network");
          Cellular.off();
      }
    }

}

void calculate_AQI(void){
    //Calculate humidity contribution to IAQ index
    gas_reference = bme.gas_resistance;
      float current_humidity = bme.readHumidity();
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
      int gas_lower_limit = 5000;   // Bad air quality limit
      int gas_upper_limit = 50000;  // Good air quality limit
      if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit;
      if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
      gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;

      //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
      air_quality_score = hum_score + gas_score;


}

void echo_gps(){
    char gps_byte = 0;
    while(!Serial.available()){
        if(Serial5.available() > 0){
            gps_byte = Serial5.read();
            Serial.print(gps_byte);
        }

    }
}

void read_gps_stream(void){
    String gps_sentence = "init";
    int stringFound = 0;
    int error = 0;
    int comma_counter = 0;
    while(!stringFound && !error){
        gps_sentence = Serial5.readStringUntil('\r');
        String prefix_string = gps_sentence.substring(4,7);
        if(prefix_string.equals("GGA")){
            //Serial.println("Found gngga!");
            //Serial.print("prefix string: ");
            //Serial.println(prefix_string);
            //Serial.print("String:");
            //Serial.println(gps_sentence);
            stringFound = 1;
        }else if(gps_sentence.equals("init")){
            error = 1;
            Serial.println("Error reading GPS");
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
                        //Serial.println(utc_string);
                    }
                }else if(comma_counter == LATITUDE_FIELD_INDEX){
                    if(gps_sentence.charAt(a+1)!=','){
                        String latitudeString = gps_sentence.substring(a+1,a+10);
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
                        //Serial.print("Longitude string: ");
                        //Serial.print(longitudeString);
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

float read_temperature(void){
    float temperature = bme.temperature;
    //temperature = temperature +
}
//read sound from
double read_sound(void){
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
    return sum;
}
//read Carbon monoxide alphasense sensor
float read_alpha1(void){
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

    digitalWrite(lmp91000_1_en, LOW);   //enable

    if(Wire.requestFrom(0x49,1) == 0){
        Serial.println("Couldn't communicate with LMP91000");
        //operation_log += "AD1,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }else{
        half_Vref = ads1.readADC_SingleEnded(3); //half of Vref
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
        Serial.println("Status == 0 from LMP91000 status reg");
        //operation_log += "AFE1,";
      //  digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
    }

    if(Wire.requestFrom(0x49,1) == 0 || lmp91000.read(LMP91000_STATUS_REG) == 0 || (abs((volt_half_Vref)/1000 - 1.25) > 0.5)){
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
        if(bme.temperature <= 15){
          correctedCurrent = ((sensorCurrent) - (auxCurrent));
        }
        else if(bme.temperature <= 25){
          correctedCurrent = ((sensorCurrent) - (-1)*(auxCurrent));
        }
        else if(bme.temperature > 25){
          correctedCurrent = ((sensorCurrent) - (-0.76)*(auxCurrent));
        }
        alpha1_ppmraw = (correctedCurrent / 0.358); //sensitivity .358 nA/ppb - from Alphasense calibration certificate, So .358 uA/ppm
        alpha1_ppmRounded = String(alpha1_ppmraw, 2);
      }

      digitalWrite(lmp91000_1_en, HIGH);  //disable


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
      return alpha1_ppmraw;
}




void outputDataToESP()
{
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


    //************Fill the cloud output array *****************************/
    //This is different than the ble packet in that we are putting all of the data that we have in one packet
    //"$1:D555g47.7M-22.050533C550.866638r1R1q2T45.8P844.9h17.2s1842.700000&"
    String cloud_output_string = "";                               //create a clean string
    cloud_output_string += '^';         //start delimeter
    cloud_output_string += String(1) + ":";           //header
    cloud_output_string += String(DEVICE_ID_PACKET_CONSTANT) + String(DEVICE_id);   //device id
    cloud_output_string += String(CARBON_MONOXIDE_PACKET_CONSTANT) + String(CO_float, 3);
    cloud_output_string += String(CARBON_DIOXIDE_PACKET_CONSTANT) + String(CO2_float, 0);
    cloud_output_string += String(VOC_PACKET_CONSTANT) + String(air_quality_score, 1);
    cloud_output_string += String(PM1_PACKET_CONSTANT) + String(PM01Value);
    cloud_output_string += String(PM2PT5_PACKET_CONSTANT) + String(PM2_5Value);
    cloud_output_string += String(PM10_PACKET_CONSTANT) + String(PM10Value);
    cloud_output_string += String(TEMPERATURE_PACKET_CONSTANT) + String(bme.temperature, 1);
    cloud_output_string += String(PRESSURE_PACKET_CONSTANT) + String(bme.pressure / 100.0, 1);
    cloud_output_string += String(HUMIDITY_PACKET_CONSTANT) + String(bme.humidity, 1);
    cloud_output_string += String(OZONE_PACKET_CONSTANT) + String(O3_float, 1);
    cloud_output_string += String(BATTERY_PACKET_CONSTANT) + String(fuel.getSoC(), 1);
    cloud_output_string += String(SOUND_PACKET_CONSTANT) + String(sound_average, 0);
    cloud_output_string += '&';

    if(!esp_wifi_connection_status){
        Serial.println("Attempting to output through LTE connection...");
        output_to_cloud(cloud_output_string);
    }else{
        Serial.println("Sending data to esp to upload via wifi...");
        Serial1.println(cloud_output_string);
    }

    delay(3000);

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
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = VOC_PACKET_CONSTANT;
            floatBytes.myFloat = air_quality_score;
        }else if(i == 3){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM1_PACKET_CONSTANT;
            floatBytes.myFloat = PM01Value;
        }else if(i == 4){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM2PT5_PACKET_CONSTANT;
            floatBytes.myFloat = PM10Value;
        }else if(i == 5){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PM10_PACKET_CONSTANT;
            floatBytes.myFloat = PM10Value;
        }else if(i == 6){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = TEMPERATURE_PACKET_CONSTANT;
            floatBytes.myFloat = bme.temperature;
        }else if(i == 7){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = PRESSURE_PACKET_CONSTANT;
            floatBytes.myFloat = bme.pressure / 100.0;
        }else if(i == 8){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = HUMIDITY_PACKET_CONSTANT;
            floatBytes.myFloat = bme.humidity;
        }else if(i == 9){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = OZONE_PACKET_CONSTANT;
            floatBytes.myFloat = O3_float;
        }else if(i == 10){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = BATTERY_PACKET_CONSTANT;
            floatBytes.myFloat = fuel.getSoC();
        }else if(i == 11){
            ble_output_array[4 + i*(BLE_PAYLOAD_SIZE)] = SOUND_PACKET_CONSTANT;
            floatBytes.myFloat = sound_average;
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
    Serial.print("ESP Wifi connection status is: ");

    Serial.printf("Byte:%X\n\r", yes_or_no);
    //Serial.println(yes_or_no);
    if(yes_or_no == 'y'){
        Serial.println("Connected!");
        esp_wifi_connection_status = 1;
    }else{
        Serial.println("No Connection");
        esp_wifi_connection_status = 0;
    }
}
//send wifi information to the ESP
void sendWifiInfo(void){
    String wifiCredentials = "@" + String(ssid) + "," + String(password) + "&";
    Serial.println("Sending new wifi credentials to ESP");
    Serial1.println(wifiCredentials);
    Serial.println("Success!");
}

/***start of all plantower functions***/
//read from plantower pms 5500
void readPlantower(void){
    if(Serial4.find("B")){    //start to read when detect 0x42
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
        serial_get_co2_slope();
    }else if(incomingByte == 'b'){
        serial_get_co2_zero();
    }else if(incomingByte == 'c'){
        serial_get_co_slope();
    }else if(incomingByte == 'd'){
        serial_get_co_zero();
    }else if(incomingByte == 'e'){
    }else if(incomingByte == 'f'){
    }else if(incomingByte == 'g'){
        echo_gps();
    }else if(incomingByte == 'h'){
    }else if(incomingByte == 'i'){
    }else if(incomingByte == 'j'){
        Serial.println("Getting wifi status from ESP\n\r");
        getEspWifiStatus();
    }else if(incomingByte == 'v'){
        serial_get_device_id();
    }else if(incomingByte == 'w'){
        serial_get_wifi_credentials();

    }else if(incomingByte == '?'){
        output_serial_menu_options();
    }
  }


}


void serial_get_wifi_credentials(void){
    Serial.print("Current stored ssid: ");
    Serial.println(ssid);
    Serial.print("Current stored password: ");
    Serial.println(password);
    Serial.println("Please enter password in order to make changes.\n\r");
    for(int i=0;i<5;i++){
      while(!Serial.available());
      recieveStr[i] = Serial.read();
    }
    String tempString = String(recieveStr);


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

void serial_get_device_id(void){

    Serial.println();
    Serial.print("Current Device ID:");
    Serial.println(DEVICE_id);
    Serial.println("Please enter password in order to change the ID");
    for(int i=0;i<5;i++){
      while(!Serial.available());
      recieveStr[i] = Serial.read();
    }
    String tempString = String(recieveStr);


    if(tempString == "bould"){
        Serial.println("Password correct!");
        Serial.println("Enter new Device ID:");
        for(int i=0;i<5;i++){
          while(!Serial.available());
          recieveStr[i] = Serial.read();
        }
        String tempString = String(recieveStr);
        int tempValue = tempString.toInt();
        Serial.println("");
        if(tempValue > MIN_DEVICE_ID_NUMBER && tempValue < MAX_DEVICE_ID_NUMBER){
            Serial.print("New Device ID:");
            Serial.println(tempValue);
            DEVICE_id = tempValue;
            EEPROM.put(DEVICE_ID_MEM_ADDRESS, DEVICE_id);
        }else{
            Serial.println("Invalid value!");
        }
    }else{
        Serial.println("Incorrect password!");
    }
}

void serial_get_co2_slope(void){

    Serial.println();
    Serial.print("Current CO2 slope:");
    Serial.print(String(CO2_slope, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO2 slope");
    for(int i=0;i<5;i++){
      while(!Serial.available());
      recieveStr[i] = Serial.read();
    }
    String tempString = String(recieveStr);
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        CO2_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("New CO2 slope: ");
        Serial.println(String(CO2_slope,2));

        EEPROM.put(CO2_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("Invalid value!");
    }
}

void serial_get_co2_zero(void){
    Serial.println();
    Serial.print("Current CO2 zero:");
    Serial.print(CO2_zero);
    Serial.println(" ppm");
    Serial.print("Enter new CO2 Zero");
    for(int i=0;i<5;i++){
      while(!Serial.available());
      recieveStr[i] = Serial.read();
    }
    String tempString = String(recieveStr);
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("New CO2 zero: ");
        Serial.println(tempValue);
        CO2_zero = tempValue;
        EEPROM.put(CO2_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("Invalid value!");
    }
}

void serial_get_co_slope(void){

    Serial.println();
    Serial.print("Current CO slope:");
    Serial.print(String(CO_slope, 2));
    Serial.println(" ppm");
    Serial.print("Enter new CO slope");
    for(int i=0;i<5;i++){
      while(!Serial.available());
      recieveStr[i] = Serial.read();
    }
    String tempString = String(recieveStr);
    float tempfloat = tempString.toFloat();
    int tempValue;

    if(tempfloat >= 0.5 && tempfloat < 1.5){
        CO_slope = tempfloat;
        tempfloat *= 100;
        tempValue = tempfloat;
        Serial.print("New CO slope: ");
        Serial.println(String(CO_slope,2));

        EEPROM.put(CO_SLOPE_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("Invalid value!");
    }
}

void serial_get_co_zero(void){
    Serial.println();
    Serial.print("Current CO zero:");
    Serial.print(CO_zero);
    Serial.println(" ppm");
    Serial.print("Enter new CO Zero");
    for(int i=0;i<5;i++){
      while(!Serial.available());
      recieveStr[i] = Serial.read();
    }
    String tempString = String(recieveStr);
    int tempValue = tempString.toInt();

    if(tempValue >= -1000 && tempValue < 1000){
        Serial.print("New CO zero: ");
        Serial.println(tempValue);
        CO_zero = tempValue;
        EEPROM.put(CO_ZERO_MEM_ADDRESS, tempValue);
    }else{
        Serial.println("Invalid value!");
    }
}

void output_serial_menu_options(void)
{
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
    Serial.println("v:  Adjust the Device ID");

    Serial.println("w:  Get wifi credentials");
    Serial.println("?:  Output this menu");
  }
