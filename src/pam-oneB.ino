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
//#include "Serial5/Serial5.h"
//#include "Serial1/Serial1.h"
#include "SdFat.h"

//enable or disable different parts of the firmware by setting the following values to 1 or 0
#define sd_en 1



#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define CS A2 //Chip select for SPI/uSD card

#define SEALEVELPRESSURE_HPA (1013.25)
SYSTEM_MODE(MANUAL);        //manually control connection to cellular network
Adafruit_BME680 bme; // I2C
Telaire_T6713 t6713;  //CO2 sensor
LMP91000 lmp91000;
Adafruit_ADS1115 ads1(0x49); //Set I2C address of ADC1
float ads_bitmv = 0.1875; //Bits per mV at defined bit resolution, used to convert ADC value to voltage

int co2 = 0;
int lmp91000_1_en = B0;     //enable line for the lmp91000 AFE chip for measuring CO
int cellular_en = D5;
int plantower_en = B4;
int counter = 0;


#if sd_en
SdFat sd;
SdFile file;
File file1;
String fileName;
#endif

void setup() {
  String init_log; //intialization error log
  pinMode(lmp91000_1_en, OUTPUT);
  digitalWrite(lmp91000_1_en, HIGH);
  Serial1.begin(9600, SERIAL_8N1);
  delay(5000);
  Serial.begin(9600);
  //Serial1.begin(9600);  //Serial4 used for communicating with BLE (TX)

  //Serial1.setTimeout(5000);
  //outputToBLE();
  #if sd_en
  Serial.println("In loop checking for sd card");

  if (sd.begin(CS)) { //if uSD is functioning and MCP error has not been logged yet.
    file.open("log.txt", O_CREAT | O_APPEND | O_WRITE);
    file.remove();
    file.open("log.txt", O_CREAT | O_APPEND | O_WRITE);
    init_log += "MCP,";
    file.print(init_log);
    file.close();


  }else { //uSD is not functioning

      Serial.println("MCP error log fail. Particle LED should be flashing");

  }

  #endif


  //outputToBLE();

  /*Serial5.begin(9600);    //testing gps
  Serial.println("reading serial5 from gps");
  long gps_counter=0;
  while(1){
    if (Serial5.available())
    {
      int inByte = Serial5.read();
      if(inByte == '$'){
        gps_counter++;
        Serial.print(gps_counter);
        Serial.print(":");
      }
      Serial.write(inByte);
    }

  }*/
  //setup the AFE
  Serial.println("Starting LMP91000 initialization");
  Wire.begin();   //this must be done for the LMP91000
  digitalWrite(lmp91000_1_en, LOW); //enable the chip

  if(lmp91000.configure(LMP91000_TIA_GAIN_120K | LMP91000_RLOAD_10OHM, LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_50PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_0PCT, LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC) == 0)
  {
        Serial.println("Couldn't communicate with LMP91000");
        //init_log += "AFE";
        //init_log += i-7;
        //init_log += ",";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
  }else{
    Serial.println("Initialized LMP91000");
    Serial.print("STATUS: ");
    Serial.println(lmp91000.read(LMP91000_STATUS_REG),HEX);
    Serial.print("TIACN: ");
    Serial.println(lmp91000.read(LMP91000_TIACN_REG),HEX);
    Serial.print("REFCN: ");
    Serial.println(lmp91000.read(LMP91000_REFCN_REG),HEX);
    Serial.print("MODECN: ");
    Serial.println(lmp91000.read(LMP91000_MODECN_REG),HEX);
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
    while (1);
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

}

void loop() {
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");


  Serial.print("CO2 = ");
  Serial.print(t6713.readPPM());
  Serial.println(" ppm");
  Serial.println();

  //read from CO sensor
  int32_t A0_gas; //gas
  int32_t A1_aux; //aux out
  int32_t A2_temperature; //temperature
  int32_t A3_half_Vref; //half of Vref
  float volt0_gas;
  float volt1_aux;
  float volt2_temperature; //need to code to be able to ensure correct sigfigs..
  float volt3_half_Vref;
  float sensorCurrent; // Working Electrode current in microamps (millivolts / Kohms)
  float auxCurrent;
  float correctedCurrent;
  float alpha4_ppmraw;
  String alpha4_ppmRounded;

  digitalWrite(lmp91000_1_en, LOW);   //enable
    if(Wire.requestFrom(0x49,1) == 0){
      Serial.println("Couldn't communicate with LMP91000");
      //operation_log += "AD1,";
      //digitalWrite(red_status_led, HIGH);
      //delay(200);
      //digitalWrite(red_status_led, LOW);
      //delay(200);
    }
    else{
      A3_half_Vref = ads1.readADC_SingleEnded(3); //half of Vref
      volt3_half_Vref = A3_half_Vref * ads_bitmv;
      if(abs((volt3_half_Vref)/1000 - 1.25) > 0.5){
        //operation_log += "AD1_VREF2,";
        //digitalWrite(red_status_led, HIGH);
        //delay(200);
        //digitalWrite(red_status_led, LOW);
        //delay(200);
        Serial.print("half vref2 ads1");
        Serial.println(volt3_half_Vref/1000);

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
    if(Wire.requestFrom(0x49,1) == 0 || lmp91000.read(LMP91000_STATUS_REG) == 0 || (abs((volt3_half_Vref)/1000 - 1.25) > 0.5)){
      alpha4_ppmRounded = "-99";
      volt0_gas = -99;
      volt1_aux = -99;
      volt3_half_Vref = -99;
      sensorCurrent = -99;
      auxCurrent = -99;
    }
    else{
      A0_gas = 0;
      A1_aux = 0;
      A2_temperature = 0;
      A3_half_Vref = 0;
      for(int i=0; i<100; i++){
        A0_gas += ads1.readADC_SingleEnded(0); //gas
        A1_aux += ads1.readADC_SingleEnded(1); //aux out
        A2_temperature += ads1.readADC_SingleEnded(2); //temperature
        A3_half_Vref += ads1.readADC_SingleEnded(3); //half of Vref
      }

      A0_gas = A0_gas / 100;
      A1_aux = A1_aux / 100;
      A2_temperature = A2_temperature / 100;
      A3_half_Vref = A3_half_Vref / 100;

      volt0_gas = A0_gas * ads_bitmv;
      volt1_aux = A1_aux * ads_bitmv;
      volt2_temperature = A2_temperature * ads_bitmv;
      volt3_half_Vref = A3_half_Vref * ads_bitmv;

      sensorCurrent = (volt3_half_Vref - volt0_gas) / (-1*120); // Working Electrode current in microamps (millivolts / Kohms)
      auxCurrent = (volt3_half_Vref - volt1_aux) / (-1*150);
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
      alpha4_ppmraw = (correctedCurrent / 0.358); //sensitivity .358 nA/ppb - from Alphasense calibration certificate, So .358 uA/ppm
      alpha4_ppmRounded = String(alpha4_ppmraw, 2);
    }
    digitalWrite(lmp91000_1_en, HIGH);  //disable


    Serial.print("CO:  ");
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
    Serial.println("Volts");


  delay(4000);
  outputToBLE();
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
    String alpha1_measurement = "A1";
    String alpha2_measurement = "A2";
    String alpha3_measurement = "A3";
    String alpha4_measurement = "CO";
    //HMOS Designators
    String HMOS_VOC = "TVOCs";
    //Analog-in Designator
    String analog_jack = "A-in";
    //ELT CO2 Sensor Designator
    String co2_measurement = "CO2";
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
    String mbar = "mbar";
    String ugm3 = "ug";
    String volts = "volts";
    String units = "units";

    /*ble_data = start + String(ID) + delim + sample_counter + ppm + alpha4_ppmRounded + alpha4_measurement + delim1;  //Alpha 4
    ble_data += String(ID) + delim + sample_counter + ppm + co2_data + co2_measurement + delim1; //ELT CO2
    ble_data += String(ID) + delim + sample_counter + ugm3 + PM01Value + pm1_measurement + delim1; //PM 1
    ble_data += String(ID) + delim + sample_counter + ugm3 + PM2_5Value + pm25_measurement + delim1; //PM 2.5
    ble_data += String(ID) + delim + sample_counter + ugm3 +  PM10Value + pm10_measurement + delim1; //PM 10
    ble_data += String(ID) + delim + sample_counter + degC + bme_temp + temp_measurement + delim1; //temperature
    ble_data += String(ID) + delim + sample_counter + mbar + bme_pres + pres_measurement + delim1; //pressure
    ble_data += String(ID) + delim + sample_counter + rh + bme_hum + hum_measurement + delim1; //humidity
    ble_data += String(ID) + delim + sample_counter + volts + batvolt_rounded + batteryVoltage_measurement + delim1 + end; //Battery Voltage
    */
    //ble_data = "$1|1PPB400CO2#1|2PPB100O3#X";
    counter++;
    ble_data = "$123|";
    ble_data += String(counter);
    ble_data += "ppm";
    ble_data += String(counter);
    ble_data += "CO2#X";
    //ble_data = "U";

    Serial1.print(ble_data);


    int bytes = Serial1.available();
    if (bytes)
    {
      Serial.print("Bytes recieved:");
      Serial.print(bytes);
      Serial.println();
      for(int x = 0; x < bytes; x++){
        int inByte = Serial1.read();
        Serial.write(inByte);
      }

    }
    //Serial.println(ble_data);


}
