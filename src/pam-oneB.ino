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

//double utc_time = 0;
//float latitude = 0;
//float longitude = 0;



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
    char ns_indicator;              //north south
    char ew_indicator;              //'E' = east, 'W' = west
    int quality_indicator;          //0-2
    int satellites_used;            //0-24
    float horizontal_dillution;     //00.0-99.9

public:
    //(d)dd + (mm.mmmm/60) (* -1 for W and S)
    void set_lat_decimal(String latString, char nsString);
    void set_long_decimal(String longString, char ewString);
    double get_latitude(void);
    double get_longitude(void);

};

//set decimal value of latitude from NMEA string
void GPS::set_lat_decimal(String latString, char nsString){
    String whole_str = latString.substring(0,2);
    String frac_str = latString.substring(2,10);

    int whole_part = whole_str.toInt();
    //Serial.print("Whole part:");
    //Serial.println(whole_part);

    double frac_part = frac_str.toFloat();
    //Serial.print("Frac part:");
    //Serial.println(frac_part, 5);


    latitude = whole_part;
    latitude += (frac_part)/60;
    if(nsString == 'S'){
        latitude *= -1;
    }
}

void GPS::set_long_decimal(String longString, char ewString){
    String whole_str = longString.substring(0,3);
    String frac_str = longString.substring(3,10);

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
    if(ewString == 'W'){
        longitude *= -1;
    }
}

double GPS::get_latitude(void){
    return latitude;
}

double GPS::get_longitude(void){
    return longitude;
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

        if(Particle.connected() && cellular_en){
            Particle.publish("airdb-pamtest", webhook_data, PRIVATE);
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
    Serial1.begin(9600, SERIAL_8N1);
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
    Serial.print("Time:");
    Serial.println(Time.now());

}

void loop() {

    //read temp, press, humidity, and TVOCs
    if (! bme.performReading()) {
      Serial.println("Failed to read BME680");
      return;
    }
    read_gps();
    //sound_average = read_sound();
    //read CO values and apply calibration factors
    CO_float = read_alpha1();
    CO_float += CO_zero;
    CO_float *= CO_slope;

    //read CO2 values and apply calibration factors
    CO2_float = t6713.readPPM();
    CO2_float += CO2_zero;
    CO2_float *= CO2_slope;

    tempValue = analogRead(A0);  // read the analogPin for ozone voltage
    O3_float = tempValue;
    O3_float *= VOLTS_PER_UNIT;           //convert digital reading to voltage
    O3_float /= VOLTS_PER_PPB;            //convert voltage to ppb of ozone


    //read PM values and apply calibration factors
    sound_average = read_sound();
    readPlantower();
    outputToBLE();
    output_to_cloud("Blah");
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

    if(cellular_en == 1){
      if (Particle.connected() == false) {
          Serial.println("Connecting to cellular network");
          Cellular.on();
          Particle.connect();
      }
    }else{
      if (Particle.connected() == true) {
          Serial.println("Disconnecting from cellular network");
          Cellular.off();
      }
    }

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

void read_gps(){
    int got_start = 0;
    int got_sentence = 0;
    char inData[100]; // Allocate some space for the string
    String gps_sentence;
    int comma_counter;
    //String gps_sentence_type;
    char byte = 0;
    Serial.println("Reading gps");

    while(!got_sentence){
        //wait for '$'
        while(!got_start){
            if(Serial5.available() > 0){
                byte = Serial5.read();
                if(byte=='$'){
                    got_start = 1;
                }
            }
        }
        got_start = 0;
        //read next 5 characters
        int i = 1;
        inData[0] = 35;             //give it a # for noting the start
        int endofline = 0;
        while( endofline != 13){ //read until the next $ is reached
            delay(3);
            if(Serial5.available() > 0){
                inData[i] = Serial5.read();
                endofline = inData[i];
                i++;
                //Serial.println(temp_char);
            }
        }
        inData[i] = '\0';
        gps_sentence = String(inData);
        //Serial.print("Read this string: ");
        //Serial.println(gps_sentence_type);
        int end_of_sentence = 0;

        String prefix_string = gps_sentence.substring(3,6);
        //Serial.print("prefix: ");
        //Serial.println(prefix_string);
        if(prefix_string.equals("GGA")){
            //Serial.println("Found gngga!");
            //Serial.print("Sentence: ");
            //Serial.println(inData);
            got_sentence = 1;
        }
    }

    //parse the gps string into latitude, longitude
    //UTC time is after first comma
    //Latitude is after second comma (ddmm.mmmm)
    //N/S indicator is after 3rd comma
    //longitude is after 4th comma (dddmm.mmmm)
    //E/W indicator is after 5th comma
    //quality is after 6th comma
    gps_sentence = String("$GNGGA,011545.00,3945.81586,N,10514.09384,W,1,08,1.28,1799.4,M,-21.5,M,,*40");
    //
    comma_counter = 0;

    for(int a = 0; a<gps_sentence.length(); a++){
        if(gps_sentence.charAt(a) == ','){
            if(comma_counter == TIME_FIELD_INDEX){
                if(gps_sentence.charAt(a+1)!=','){
                    String utc_string = gps_sentence.substring(a+1,a+11);
                    Serial.print("GPS utc string: ");
                    Serial.println(utc_string);
                }
            }else if(comma_counter == LATITUDE_FIELD_INDEX){
                if(gps_sentence.charAt(a+1)!=','){
                    String latitudeString = gps_sentence.substring(a+1,a+11);
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
                    String longitudeString = gps_sentence.substring(a+1,a+12);
                    //Serial.print("Longitude string: ");
                    //Serial.print(longitudeString);
                    //Serial.print(" ");
                    //Serial.println(gps_sentence.charAt(a+13));
                    gps.set_long_decimal(longitudeString, gps_sentence.charAt(a+13));
                    //Serial.print("Longitude decimal: ");
                    //Serial.println(gps.get_longitude(), 5);
                }
            }
            comma_counter++;
        }
    }


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


void outputToBLE()
{

    String ble_data = "";
    String start = "$";
    String delim = "|";
    String delim1 = "#";
    String end = "X";
    //Phantower PM measurements Designators
    String pm1_measurement = "PM1";
    String pm25_measurement = "PM25";
    String pm10_measurement = "PM10";
    //Alphasense Electrochemical Sensors Designators
    String CO_measurement = "CO";

    String voc_measurement = "VOC";
    //Analog-in Designator
    String analog_jack = "A-in";
    //ELT CO2 Sensor Designator
    String co2_measurement = "CO2";

    //Ozone measurement from analog input connected to 106-L
    String o3_measurement = "O3";

    //BME 280 Temperature, Pressure, Humidity Designators
    String temp_measurement = "Temp";
    String pres_measurement = "Pres";
    String hum_measurement = "rhum";
    //Date and Time Designators
    String date_measurement = "Date";
    String time_measurement = "Time";
    String batteryVoltage_measurement = "Batt";
    //units
    String ppb = "ppb";
    String ppm = "ppm";
    String degC = "C";
    String rh = "%";
    String mbar = "hPa";
    String ugm3 = "ug";
    String chargePercent = "%";
    String resistance = "KOhms";


        ble_data = start;
        ble_data += String(DEVICE_id) + delim + sample_counter + ppm + String(CO_float, 3) + CO_measurement + delim1;  //Alpha 4
        ble_data += String(DEVICE_id) + delim + sample_counter + ppm + String(CO2_float, 0) + co2_measurement + delim1; //ELT CO2
        ble_data += String(DEVICE_id) + delim + sample_counter + resistance + String(bme.gas_resistance / 1000.0, 1) + voc_measurement + delim1;
        ble_data += String(DEVICE_id) + delim + sample_counter + ugm3 + PM01Value + pm1_measurement + delim1; //PM 1
        ble_data += String(DEVICE_id) + delim + sample_counter + ugm3 + PM2_5Value + pm25_measurement + delim1; //PM 2.5
        ble_data += String(DEVICE_id) + delim + sample_counter + ugm3 +  PM10Value + pm10_measurement + delim1; //PM 10
        ble_data += String(DEVICE_id) + delim + sample_counter + degC + String(bme.temperature, 1) + temp_measurement + delim1; //temperature
        ble_data += String(DEVICE_id) + delim + sample_counter + mbar + String(bme.pressure / 100.0, 1) + pres_measurement + delim1; //pressure
        ble_data += String(DEVICE_id) + delim + sample_counter + rh + String(bme.humidity, 1) + hum_measurement + delim1; //humidity
        ble_data += String(DEVICE_id) + delim + sample_counter + ppb + String(O3_float, 1) + o3_measurement + delim1;
        ble_data += String(DEVICE_id) + delim + sample_counter + chargePercent + String(fuel.getSoC(), 1) + batteryVoltage_measurement + delim1; //Battery Voltage
        ble_data += String(DEVICE_id) + delim + sample_counter + "DBs" + String(sound_average, 0) + "Snd" + delim1;
        ble_data += String(DEVICE_id) + delim + sample_counter + "D" + String(gps.get_latitude(), 5) + "L" + delim1;
        ble_data += String(DEVICE_id) + delim + sample_counter + "D" + String(gps.get_longitude(), 5) + "G" + delim1;
        ble_data += end;
        Serial1.print(ble_data);
        Serial.println(ble_data);
        delay(1000);

    //ble_data = "$1|1PPB400CO2#1|2PPB100O3#X";
    /*counter++;
    ble_data = "$123|";
    ble_data += String(counter);
    ble_data += "ppm";
    ble_data += String(counter);
    ble_data += "CO2#X";*/
    //ble_data = "U";

    //transmit to the ble through the serial port
    //Serial1.print(ble_data);


    /*int bytes = Serial1.available();
    if (bytes)
    {
      //Serial.print("Bytes recieved:");
      //Serial.print(bytes);
      //Serial.println();
      for(int x = 0; x < bytes; x++){
        int inByte = Serial1.read();
        Serial.write(inByte);
      }

    }
    Serial.println(ble_data);*/


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
    }else if(incomingByte == 'h'){
    }else if(incomingByte == 'i'){
    }else if(incomingByte == 'v'){
        serial_get_device_id();
    }else if(incomingByte == '?'){
        output_serial_menu_options();
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
    Serial.println("?:  Output this menu");
  }
