
#include "application.h"
#include "Particle.h"
#include "Telaire_T6713.h"

//#define T6713_DEBUG     //comment this out if not want debug

#define ADDR_6713  0x15 // default I2C slave address

Telaire_T6713::Telaire_T6713()
{
  //perform initialization
}

bool Telaire_T6713::begin()
{
  byte func_code, byte_count, MSB, LSB;

  func_code = 0;
  byte_count = 0;
  MSB = 0;
  LSB = 0;
  #ifdef T6713_DEBUG
  Serial.println("Begin Telaire_T6713");
  #endif
  // start I2C
  Wire.beginTransmission(ADDR_6713);

  // Function code = 0x04
  Wire.write(0x04);
  // Starting address (MSB) = 0x13
  Wire.write(0x13);
  // Starting address (LSB) = 0x8B
  Wire.write(0x8B);
  // Input registers to read (MSB) = 0x00
  Wire.write(0x00);
  // Input registers to read (LSB) = 0x01
  Wire.write(0x01);

  // end transmission
  Wire.endTransmission();
  Wire.requestFrom(ADDR_6713, 4);    // request 6 bytes from slave device
  while(Wire.available() == 0);
  func_code = Wire.read();
  byte_count = Wire.read();
  MSB = Wire.read();
  LSB = Wire.read();
  #ifdef T6713_DEBUG
  Serial.print("Func code: ");
  Serial.println(func_code);
  Serial.print("byte count: ");
  Serial.println(byte_count);
  Serial.print("MSB: ");
  Serial.println(MSB);
  Serial.print("LSB: ");
  Serial.println(LSB);
  #endif
  int ppm = MSB*256 + LSB;
  if(!ppm)
  {
    return false;
  }

  return true;
}
// send request to read current gas measurement in ppm
// return status: 0 success
void Telaire_T6713::queryPPM()
{
  byte func_code, byte_count, MSB, LSB;

  func_code = 0;
  byte_count = 0;
  MSB = 0;
  LSB = 0;

  // start I2C
  Wire.beginTransmission(ADDR_6713);

  // Function code = 0x04
  Wire.write(0x04);
  // Starting address (MSB) = 0x13
  Wire.write(0x13);
  // Starting address (LSB) = 0x8B
  Wire.write(0x8B);
  // Input registers to read (MSB) = 0x00
  Wire.write(0x00);
  // Input registers to read (LSB) = 0x01
  Wire.write(0x01);

  // end transmission
  Wire.endTransmission();
  /*Wire.requestFrom(ADDR_6713, 4);    // request 6 bytes from slave device
  while(Wire.available() == 0);
  func_code = Wire.read();
  byte_count = Wire.read();
  MSB = Wire.read();
  LSB = Wire.read();
  Serial.print("Func code: ");
  Serial.println(func_code);
  Serial.print("byte count: ");
  Serial.println(byte_count);
  Serial.print("MSB: ");
  Serial.println(MSB);
  Serial.print("LSB: ");
  Serial.println(LSB);
  int ppm = MSB*256 + LSB;
  Serial.println(ppm);*/


}

// read report of current gas measurement in ppm
int Telaire_T6713::readPPM()
{
  byte func_code, byte_count, MSB, LSB;
  func_code = 0;
  byte_count = 0;
  MSB = 0;
  LSB = 0;
  #ifdef T6713_DEBUG
  Serial.println("reading gas ppm");
  #endif
  
  queryPPM();
  Wire.requestFrom(ADDR_6713, 4);    // request 6 bytes from slave device

   while(Wire.available() == 0);
   func_code = Wire.read();
   byte_count = Wire.read();
   MSB = Wire.read();
   LSB = Wire.read();
   #ifdef T6713_DEBUG
   Serial.print("Func code: ");
   Serial.println(func_code);
   Serial.print("byte count: ");
   Serial.println(byte_count);
   Serial.print("MSB: ");
   Serial.println(MSB);
   Serial.print("LSB: ");
   Serial.println(LSB);
   #endif

   // ppm = MSB*256 + LSB
   return ((MSB<<8)|LSB);
}

void Telaire_T6713::readStatus()
{
  byte func_code, byte_count, MSB, LSB;
  func_code = 0;
  byte_count = 0;
  MSB = 0;
  LSB = 0;
  Wire.beginTransmission(ADDR_6713);

  // Function code = 0x04
 //Wire.write(0x04);
  // Starting address (MSB) = 0x13
  //Wire.write(0x13);
  // Starting address (LSB) = 0x8B
  //Wire.write(0x8A);
  // Input registers to read (MSB) = 0x00
  //Wire.write(0x00);
  // Input registers to read (LSB) = 0x01
  //Wire.write(0x01);

  // Function code = 0x04
  Wire.write(0x04);
  // Starting address (MSB) = 0x13
  Wire.write(0x13);
  // Starting address (LSB) = 0x8B
  Wire.write(0x8A);
  // Input registers to read (MSB) = 0x00
  Wire.write(0x00);
  // Input registers to read (LSB) = 0x01
  Wire.write(0x01);

  // end transmission
   Wire.endTransmission();
   #ifdef T6713_DEBUG
   Serial.println("reading sensor status");
   #endif
   Wire.requestFrom(ADDR_6713, 4);    // request 6 bytes from slave device

   while(Wire.available() == 0);
   func_code = Wire.read();
   byte_count = Wire.read();
   MSB = Wire.read();
   LSB = Wire.read();
   #ifdef T6713_DEBUG
   Serial.print("Func code: ");
   Serial.println(func_code, HEX);
   Serial.print("byte count: ");
   Serial.println(byte_count, HEX);
   Serial.print("MSB: ");
   Serial.println(MSB, BIN);
   Serial.print("LSB: ");
   Serial.println(LSB, BIN);
   #endif
}

void Telaire_T6713::resetSensor()
{
  Wire.beginTransmission(ADDR_6713);

  // Function code = 0x04
  Wire.write(0x05);
  // Starting address (MSB) = 0x13
  Wire.write(0x03);
  // Starting address (LSB) = 0x8B
  Wire.write(0xE8);
  // Input registers to read (MSB) = 0x00
  Wire.write(0xFF);
  // Input registers to read (LSB) = 0x01
  Wire.write(0x00);

  // end transmission
   Wire.endTransmission();
}
