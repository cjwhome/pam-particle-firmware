#include "Serial4/Serial4.h"
#include "Serial5/Serial5.h"
#include "inttypes.h"
#include "Particle.h"
#include "PowerCheck.h"

#include "Wiring.h"
//define addresses of eeprom stored variables
#include "PAMEEPROM/PAMEEPROM.h"

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
#include "MenuFunctions/TimeZoneSet.h"
#include "MenuFunctions/BuildVersion.h"
#include "MenuFunctions/ContinuousGPS.h"

//manually control connection to cellular network
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

#define BATTERY_THRESHOLD 20    //if battery is below 20 percent, go into sleep mode

Global * globalVariables = nullptr;
SendingData * send_measurements = nullptr;
PAMSensorManager *manager = nullptr;
PowerCheck powerCheck;
FuelGauge fuel;

time_t watch_time;
int averaging_time = 0;
PAMSerialMenu serial_menu;
uint16_t serial_menu_rd;
uint16_t rd = 0;

EEPROMReset eepromReset = EEPROMReset();
PrintHeader printHeader = PrintHeader();
CellularInfo cellularInfo = CellularInfo();
EspReset espReset = EspReset();
DateTimeSet datetimeSet = DateTimeSet();
TimeZoneSet timezoneSet = TimeZoneSet();
BuildVersion buildVersion = BuildVersion();
ContinuousGps continuousGps = ContinuousGps();

void enableContinuousGPS(void);
void sendPacket(byte *packet, byte len);

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









//test for setting up PMIC manually
void writeRegister(uint8_t reg, uint8_t value) {
    // This would be easier if pmic.writeRegister wasn't private
    Wire3.beginTransmission(PMIC_ADDRESS);
    Wire3.write(reg);
    Wire3.write(value);
    Wire3.endTransmission(true);

}

void buildSerialMenu(PAMSerialMenu serial_menu)
{
    PAMSerialEditEEPROMValue<int> * averaging_measurement = new PAMSerialEditEEPROMValue<int>(averaging_time, MEASUREMENTS_TO_AVG_MEM_ADDRESS, 300);
    PAMSerialEditEEPROMValue<int> * device_id = new PAMSerialEditEEPROMValue<int>(globalVariables->device_id, DEVICE_ID_MEM_ADDRESS, -1);
    PAMSerialEditEEPROMValue<bool> * debugging_enabled = new PAMSerialEditEEPROMValue<bool>(globalVariables->debugging_enabled, DEBUGGING_ENABLED_MEM_ADDRESS, 0);
    PAMSerialEditEEPROMValue<bool> * cellular_enabled = new PAMSerialEditEEPROMValue<bool>(globalVariables->cellular_enabled, CELLULAR_EN_MEM_ADDRESS, 0);
    PAMSerialEditEEPROMValue<bool> * sensible_iot_en = new PAMSerialEditEEPROMValue<bool>(globalVariables->sensible_iot_en, SENSIBLEIOT_ENABLE_MEM_ADDRESS, 0);
    PAMSerialEditEEPROMValue<bool> * temperature_units = new PAMSerialEditEEPROMValue<bool>(globalVariables->temperature_units, TEMPERATURE_UNITS_MEM_ADDRESS, 0);
    PAMSerialEditEEPROMValue<bool> * car_topper = new PAMSerialEditEEPROMValue<bool>(globalVariables->car_topper, CAR_TOPPER_POWER_MEM_ADDRESS, 0);



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
    serial_menu.addResponder(&timezoneSet, "Set the time zone for the PAM");
    serial_menu.addResponder(&buildVersion, "Print Version and Build");
    serial_menu.addResponder(temperature_units, "Celcius or Farenheit? (false is Celcius)");
    serial_menu.addResponder(car_topper, "Enable / Disable car topper mode");
    serial_menu.addResponder(&continuousGps, "Read the gps rapidly and continuously");

}


void setup()
{
    Wire.begin();
    Serial.begin(9600);

    globalVariables = Global::GetInstance();
    manager = PAMSensorManager::GetInstance();
    send_measurements = SendingData::GetInstance();
    PMIC pmic;

    // PAM Sensors
    T6713 t6713;
    TPHFusion tph_fusion(0x27, false);
    Plantower plantower(Serial4);
    PAMCO pamco(ADS1115_1_ADDR, LMP91000_1_EN);
    PAM_108L pam_108L;

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

    if((fuel.getSoC() < BATTERY_THRESHOLD) && (powerCheck.getHasPower() == 0)){
        goToSleepBattery();
    }
    //if user presses power button during operation, reset and it will go to low power mode
    attachInterrupt(D4, System.reset, RISING);
    if(digitalRead(D4)){
      goToSleep();
    }

    digitalWrite(POWER_LED_EN, HIGH);
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

    // Resetting the esp
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

    Serial.println("ESP reset!");

    Serial.print("FW Version: ");
    Serial.println(APP_VERSION);
    Serial.print("Build: ");
    Serial.println(BUILD_VERSION);

    enableContinuousGPS();
    buildSerialMenu(serial_menu);



    manager->addSensor(&pamco);
    manager->addSensor(&t6713);
    manager->addSensor(&plantower);
    manager->addSensor(&tph_fusion);
    if (globalVariables->ozone_enabled == true)
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
        Serial.println("Starting averaging now: ");
        manager->runAllAverages();
        if (globalVariables->cellular_enabled)
        {
            globalVariables->status_word->status_int |= 0x01;
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
    //check power
    powerCheck.loop();

    if(globalVariables->car_topper && powerCheck.getHasPower() == 0){
        if ( globalVariables->car_topper_time == 0)
        {
            startCarTopperTimer();
        }
        else if(Time.now() > globalVariables->car_topper_time+1800) // This is thirty minutes without the PAM receiving power.
        {
            goToSleepBattery();
        }
    }
    else if (globalVariables->car_topper_time > 0 && powerCheck.getHasPower() == 1)
    {
        globalVariables->car_topper_time = 0;
    }

    if((fuel.getSoC() < BATTERY_THRESHOLD) && (powerCheck.getHasPower() == 0)){
        Serial.println("Going to sleep because battery is below 20% charge");
        goToSleepBattery();
    }
}

void startCarTopperTimer()
{
    globalVariables->car_topper_time = Time.now();
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

void goToSleep(void){
    //Serial.println("Going to sleep:)");
    digitalWrite(POWER_LED_EN, LOW);
    digitalWrite(PLANTOWER_EN, LOW);
    digitalWrite(ESP_WROOM_EN, LOW);
    digitalWrite(BLOWER_EN, LOW);
    digitalWrite(CO2_EN, LOW);
    digitalWrite(FIVE_VOLT_EN, LOW);
    System.sleep(D4, FALLING, 120);     //every 2 minutes wake up and check if battery voltage is too low
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