#include "Serial4/Serial4.h"
#include "Serial5/Serial5.h"
#include "inttypes.h"
#include "Particle.h"
#include "PowerCheck.h"

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
#include "Global.h"
#include "MenuFunctions/ResetEEPROM.h"
#include "MenuFunctions/PrintHeader.h"
#include "MenuFunctions/CellularInfo.h"
#include "MenuFunctions/EspReset.h"
#include "MenuFunctions/DateTimeSet.h"
#include "MenuFunctions/DateTimeSet.h"

#define APP_VERSION 8
#define BUILD_VERSION 1

//manually control connection to cellular network
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);



//enable or disable different parts of the firmware by setting the following values to 1 or 0
#define sd_en 1

//define addresses of eeprom stored variables
#include "PAMEEPROM/PAMEEPROM.h"

#define BATTERY_THRESHOLD 20    //if battery is below 20 percent, go into sleep mode

#define SLEEP_EN D3

// PAM Sensors
T6713 t6713;
TPHFusion tph_fusion(0x27, false);
Plantower plantower(Serial4);
PAMCO pamco(ADS1115_1_ADDR, LMP91000_1_EN);
PAM_108L pam_108L;
Global * globalVariables = nullptr;

SendingData * send_measurements = nullptr;
PAMSensorManager *manager = nullptr;

time_t watch_time;
int averaging_time = 0;
EEPROMReset eepromReset = EEPROMReset();
PrintHeader printHeader = PrintHeader();
CellularInfo cellularInfo = CellularInfo();
EspReset espReset = EspReset();
DateTimeSet datetimeSet = DateTimeSet();


PMIC pmic;
PowerCheck powerCheck;
//SerialLogHandler logHandler;

unsigned long lastCheck = 0;
char lastStatus[256];




//global variables
char incomingByte;
int tempValue;
int esp_wifi_connection_status = 0;
int cellular_enabled = 0;
int ozone_enabled = 0;
int temperature_units = 0;
int abc_logic_enabled = 0;
bool tried_cellular_connect = false;
int battery_threshold_enable;

int sleepInterval = 60;  // This is used below for sleep times and is equal to 60 seconds of time.

FuelGauge fuel;

//char gps_status = 0;

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
void serialMenu();
void serialIncreaseInputCurrent(void);

void outputSerialMenuOptions(void);
void echoGps();
void resetEsp(void);

//gps functions
void enableLowPowerGPS(void);
void enableContinuousGPS(void);
void changeFrequency(void);
void sendPacket(byte *packet, byte len);




//test for setting up PMIC manually
void writeRegister(uint8_t reg, uint8_t value) {
    // This would be easier if pmic.writeRegister wasn't private
    Wire3.beginTransmission(PMIC_ADDRESS);
    Wire3.write(reg);
    Wire3.write(value);
    Wire3.endTransmission(true);

}

void buildSerialMenu()
{
    PAMSerialEditEEPROMValue<int> * averaging_measurement = new PAMSerialEditEEPROMValue<int>(averaging_time, MEASUREMENTS_TO_AVG_MEM_ADDRESS, 300);
    PAMSerialEditEEPROMValue<int> * device_id = new PAMSerialEditEEPROMValue<int>(globalVariables->device_id, DEVICE_ID_MEM_ADDRESS, -1);
    PAMSerialEditEEPROMValue<bool> * debugging_enabled = new PAMSerialEditEEPROMValue<bool>(globalVariables->debugging_enabled, DEBUGGING_ENABLED_MEM_ADDRESS, 0);
    PAMSerialEditEEPROMValue<bool> * cellular_enabled = new PAMSerialEditEEPROMValue<bool>(globalVariables->cellular_enabled, CELLULAR_EN_MEM_ADDRESS, 0);
    PAMSerialEditEEPROMValue<bool> * sensible_iot_en = new PAMSerialEditEEPROMValue<bool>(globalVariables->sensible_iot_en, SENSIBLEIOT_ENABLE_MEM_ADDRESS, 0);



    serial_menu.addResponder(PAMSensorManager::GetInstance()->serial_menu_rd, "Sensor Settings");
    serial_menu.addResponder(averaging_measurement, "Set Averaging Time (seconds)");
    serial_menu.addResponder(device_id, "Set Device Id");
    serial_menu.addResponder(debugging_enabled, "Enable/ Disable debugging");
    serial_menu.addResponder(&eepromReset, "Reset EEPROM values");
    serial_menu.addResponder(&printHeader, "Print header for PAM");
    serial_menu.addResponder(&cellularInfo, "Print cellular info for PAM");
    serial_menu.addResponder(cellular_enabled, "Enable/ Disable Cellular");
    serial_menu.addResponder(sensible_iot_en, "Enable/ Disable Sensible");
    serial_menu.addResponder(&espReset, "Reset the ESP, Plantower, CO2 Sensor and Blower");
    serial_menu.addResponder(&datetimeSet, "Set the date time using EPOCH");

    //serial_menu.addResponder()
}


void setup()
{
    Wire.begin();
    Serial.begin(9600);

    globalVariables = Global::GetInstance();
    manager = PAMSensorManager::GetInstance();
    send_measurements = SendingData::GetInstance();

    // cellularInfo = new CellularInfo();
    // if(cellularInfo == nullptr)
    //     Serial.println("Cellular info is null");


    globalVariables->status_word->status_int  = 0;
    globalVariables->status_word->status_int |= (APP_VERSION << 12) & 0xFF00;
    globalVariables->status_word->status_int |= (BUILD_VERSION << 8) & 0xF00;
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


    //delay for 5 seconds to give time to programmer person for connecting to serial port for debugging
    delay(10000);
    //initialize main serial port for debug outpu

    rd = PAMSerial.registerResponder(new TestSerialConnector());
    PAMSerial.pushResponder(rd);
    PAMSerial.printf(rd, "TESTING FROM PAM SERIAL\n\r");
    serial_menu_rd = PAMSerial.registerResponder(&serial_menu);

    resetESP();

    Serial.println("ESP reset!");

    Serial.print("FW Version: ");
    Serial.println(APP_VERSION);
    Serial.print("Build: ");
    Serial.println(BUILD_VERSION);

    enableContinuousGPS();
    buildSerialMenu();

    manager->addSensor(&pamco);
    manager->addSensor(&t6713);
    manager->addSensor(&plantower);
    manager->addSensor(&tph_fusion);
    if (ozone_enabled == true)
    {
            manager->addSensor(&pam_108L);
    }

    send_measurements->addSensors();


    char *csv_header = manager->csvHeader();
    Serial.println(csv_header);
    free(csv_header);

    
    Log.info("System version: %s", (const char*)System.version());

    watch_time = Time.now();

}

void loop() 
{
    if (Time.now() > watch_time+averaging_time)
    {
        Serial.println("Starting avergaing now: ");
        manager->runAllAverages();
        if (globalVariables->cellular_enabled)
        {
            send_measurements->SendDataToParticle();
            if (globalVariables->sensible_iot_en)
            {
                send_measurements->SendDataToSensible();
            }
        }
        send_measurements->SendDataToESP();
        send_measurements->SendDataToSd();
        
        watch_time = Time.now();
    }


    PAMSensorManager::GetInstance()->loop();
    PAMSerial.loop();

    // if(cellular_enabled)
    // {
    //     //globalVariables->status_word->status_int |= 0x01;
    //     if (Particle.connected() == false && tried_cellular_connect == false) 
    //     {
    //         tried_cellular_connect = true;
    //         Cellular.on();
    //         Particle.connect();
    //     }
    //     else if(Particle.connected() == true){  //this means that it is already connected
    //         tried_cellular_connect = false;
    //     }
    // }
    // else if (Particle.connected() == true) {
    //       Cellular.off();
    // }

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

// void echoGps(){
//     char gps_byte = 0;
//     while(!Serial.available()){
//         if(Serial5.available() > 0){
//             gps_byte = Serial5.read();
//             Serial.print(gps_byte);
//         }

//     }
// }

// void readGpsStream(void){
//     String gps_sentence = "init";
//     int stringFound = 0;
//     int error = 0;
//     int comma_counter = 0;
//     while(!stringFound && !error){
//         gps_sentence = Serial5.readStringUntil('\r');
//         String prefix_string = gps_sentence.substring(4,7);
//         if(prefix_string.equals("GGA")){

//             //
//             //Serial.print("prefix string: ");
//             //Serial.println(prefix_string);
//             //Serial.print("String:");
//             //Serial.println(gps_sentence);
//             stringFound = 1;
//         }else if(gps_sentence.equals("init")){
//             error = 1;
//             Serial.println("Error reading GPS");
//             writeLogFile("Error reading GPS");
//         }
//     }
//     if(stringFound){

//         //parse the gps string into latitude, longitude
//         //UTC time is after first comma
//         //Latitude is after second comma (ddmm.mmmm)
//         //N/S indicator is after 3rd comma
//         //longitude is after 4th comma (dddmm.mmmm)
//         //E/W indicator is after 5th comma
//         //quality is after 6th comma
//         //gps_sentence = String("$GNGGA,011545.00,3945.81586,N,10514.09384,W,1,08,1.28,1799.4,M,-21.5,M,,*40");
//         //
//         comma_counter = 0;

//         for(int a = 0; a<gps_sentence.length(); a++){
//             if(gps_sentence.charAt(a) == ','){
//                 if(comma_counter == TIME_FIELD_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String utc_string = gps_sentence.substring(a+1,a+11);
//                         //Serial.print("GPS utc string: ");
//                         if(debugging_enabled){
//                             Serial.print("GPS utc string: ");
//                             Serial.println(utc_string);

//                         }
//                         //Serial.println(utc_string);
//                     }
//                 }else if(comma_counter == LATITUDE_FIELD_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String latitudeString = gps_sentence.substring(a+1,a+10);
//                         if(debugging_enabled){
//                           Serial.print("Latitude string: ");
//                           Serial.print(latitudeString);
//                         }
//                         //Serial.print("Latitude string: ");
//                         //Serial.print(latitudeString);
//                         //Serial.print(" ");
//                         //Serial.println(gps_sentence.charAt(a+12));
//                         gps.set_lat_decimal(latitudeString, gps_sentence.charAt(a+12));
//                         status_word.status_int &= 0xFFF7;
//                         //Serial.print("Latitude decimal: ");
//                         //Serial.println(gps.get_latitude(), 5);
//                     }
//                 }else if(comma_counter == LONGITUDE_FIELD_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String longitudeString = gps_sentence.substring(a+1,a+11);
//                         if(debugging_enabled){
//                           Serial.print("longitude string: ");
//                           Serial.print(longitudeString);
//                         }
//                         //Serial.print(" ");
//                         //Serial.println(gps_sentence.charAt(a+13));
//                         gps.set_long_decimal(longitudeString, gps_sentence.charAt(a+13));
//                         //Serial.print("Longitude decimal: ");
//                         //Serial.println(gps.get_longitude(), 5);
//                     }
//                 }else if(comma_counter == NUMBER_OF_SATELLITES_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String numberOfSatellitesString = gps_sentence.substring(a+1,a+3);
//                         gps.set_satellites(numberOfSatellitesString);
//                     }
//                 }else if(comma_counter == HORZONTAL_DILLUTION_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String hdString = gps_sentence.substring(a+1,a+3);
//                         gps.set_horizontalDillution(hdString);
//                         status_word.status_int &= 0xFFF3;
//                         if(gps.get_horizontalDillution() < 2){
//                             status_word.status_int |= 0x000C;
//                         }else if(gps.get_horizontalDillution() < 5){
//                             status_word.status_int |= 0x0008;
//                         }else if(gps.get_horizontalDillution() < 20){
//                             status_word.status_int |= 0x0004;
//                         }


//                     }
//                 }
//                 comma_counter++;
//             }
//         }
//     }

// }

// void readGpsStreamDate(void){
//     String gps_sentence = "init";
//     int stringFound = 0;
//     int error = 0;
//     int comma_counter = 0;
//     while(!stringFound && !error){
//         gps_sentence = Serial5.readStringUntil('\r');
//         String prefix_string = gps_sentence.substring(4,7);
//         if(prefix_string.equals("RMC")){

//             //
//             //Serial.print("prefix string: ");
//             //Serial.println(prefix_string);
//             //Serial.print("String:");
//             //Serial.println(gps_sentence);
//             stringFound = 1;
//         }else if(gps_sentence.equals("init")){
//             error = 1;
//             Serial.println("Error reading GPS RMC");
//             writeLogFile("Error reading GPS RMC");
//         }
//     }
//     if(stringFound){

//         //parse the gps string into latitude, longitude
//         //UTC time is after first comma
//         //Latitude is after second comma (ddmm.mmmm)
//         //N/S indicator is after 3rd comma
//         //longitude is after 4th comma (dddmm.mmmm)
//         //E/W indicator is after 5th comma
//         //quality is after 6th comma
//         //gps_sentence = String("$GNGGA,011545.00,3945.81586,N,10514.09384,W,1,08,1.28,1799.4,M,-21.5,M,,*40");
//         //
//         comma_counter = 0;

//         for(int a = 0; a<gps_sentence.length(); a++){
//             if(gps_sentence.charAt(a) == ','){
//                 if(comma_counter == DATE_FIELD_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String utc_string = gps_sentence.substring(a+1,a+11);
//                         //Serial.print("GPS utc string: ");
//                         if(debugging_enabled){
//                             Serial.print("GPS utc string: ");
//                             Serial.println(utc_string);

//                         }
//                         //Serial.println(utc_string);
//                     }
//                 }else if(comma_counter == LATITUDE_FIELD_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String latitudeString = gps_sentence.substring(a+1,a+10);
//                         if(debugging_enabled){
//                           Serial.print("Latitude string: ");
//                           Serial.print(latitudeString);
//                         }
//                         //Serial.print("Latitude string: ");
//                         //Serial.print(latitudeString);
//                         //Serial.print(" ");
//                         //Serial.println(gps_sentence.charAt(a+12));
//                         gps.set_lat_decimal(latitudeString, gps_sentence.charAt(a+12));
//                         status_word.status_int &= 0xFFF7;
//                         //Serial.print("Latitude decimal: ");
//                         //Serial.println(gps.get_latitude(), 5);
//                     }
//                 }else if(comma_counter == LONGITUDE_FIELD_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String longitudeString = gps_sentence.substring(a+1,a+11);
//                         if(debugging_enabled){
//                           Serial.print("longitude string: ");
//                           Serial.print(longitudeString);
//                         }
//                         //Serial.print(" ");
//                         //Serial.println(gps_sentence.charAt(a+13));
//                         gps.set_long_decimal(longitudeString, gps_sentence.charAt(a+13));
//                         //Serial.print("Longitude decimal: ");
//                         //Serial.println(gps.get_longitude(), 5);
//                     }
//                 }else if(comma_counter == NUMBER_OF_SATELLITES_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String numberOfSatellitesString = gps_sentence.substring(a+1,a+3);
//                         gps.set_satellites(numberOfSatellitesString);
//                     }
//                 }else if(comma_counter == HORZONTAL_DILLUTION_INDEX){
//                     if(gps_sentence.charAt(a+1)!=','){
//                         String hdString = gps_sentence.substring(a+1,a+3);
//                         gps.set_horizontalDillution(hdString);
//                         status_word.status_int &= 0xFFF3;
//                         if(gps.get_horizontalDillution() < 2){
//                             status_word.status_int |= 0x000C;
//                         }else if(gps.get_horizontalDillution() < 5){
//                             status_word.status_int |= 0x0008;
//                         }else if(gps.get_horizontalDillution() < 20){
//                             status_word.status_int |= 0x0004;
//                         }


//                     }
//                 }
//                 comma_counter++;
//             }
//         }
//     }

// }
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
    //Serial.println(yes_or_no);
    if(yes_or_no == 'y'){
        esp_wifi_connection_status = 1;
    }
    else{
        esp_wifi_connection_status = 0;
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


    if(incomingByte == 'R'){
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
    else if(incomingByte == 'X'){
        t6713._t6713.calibrate(1);
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

void outputSerialMenuOptions(void){
    Serial.println("Command:  Description");
    Serial.println("t:  Enter new time and date");
    Serial.println("u:  Enter new time zone");
    Serial.println("w:  Get wifi credentials");
    Serial.println("y:  Enable cellular");
    Serial.println("z:  Disable cellular");
    Serial.println("3:  Get build version");
    Serial.println("8:  Output the PMIC system configuration");
    Serial.println("9:  Increase the charge current by 64 mA");
    Serial.println("0:  Increase the current input limit by 100 mA");
    Serial.println("J:  Reset ESP, CO2, Plantower");
    Serial.println("M:  Enable 20% battery threshold limiting");
    Serial.println("N:  Disable 20% battery threshold limiting WARNING!!");
    Serial.println("P:  Turn off BATFET");
    Serial.println("Q:  Allow BATFET to turn on");
    Serial.println("R:  Disable ABC logic for CO2 sensor");
    Serial.println("S:  Enable ABC logic for CO2 sensor");
    Serial.println("X:  Calibrate CO2 sensor - must supply ambient level (go outside!)");
    Serial.println("Z:  Output cellular information (ICCID, IMEI, etc)");
}