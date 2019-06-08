#include "HIH61XX.h"

//  Create an HIH61XX with I2C address 0x27
HIH61XX hih(0x27, D3);

void setup()
{
    Serial.begin(9600);
    // Begin I2C communication
    Wire.begin();
}


void loop()
{
    //  start the sensor
    hih.start();

    //  request an update of the humidity and temperature
    hih.update();

    Serial.print("Humidity: ");
    Serial.print(hih.humidity(), 5);
    Serial.print(" RH (");
    Serial.print(hih.humidity_Raw());
    Serial.println(")");

    Serial.print("Temperature: ");
    Serial.print(hih.temperature(), 5);
    Serial.println(" C (");
    Serial.print(hih.temperature_Raw());
    Serial.println(")");
    delay(100);
}
