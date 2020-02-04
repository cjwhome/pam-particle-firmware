/***************************************************************************
  This is a library for Telaire 6713 CO2 sensor
  Derived from the BME_680 firmware driver made by adafruit
 ***************************************************************************/
#ifndef __T6713_H__
#define __T6713_H__

#include "application.h"
#include "Particle.h"





/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define T6713_DEFAULT_ADDRESS                0x15





class Telaire_T6713
{
  public:
    Telaire_T6713();
    void queryPPM(void);
    int readPPM(void);
    void readStatus(void);
    void resetSensor(void);
    void calibrate(int debugging);
    /*bool  begin(uint8_t addr = T6713_DEFAULT_ADDRESS);
    float readTemperature(void);
    float readPressure(void);
    float readHumidity(void);
    uint32_t readGas(void);
    float readAltitude(float seaLevel);

    bool setTemperatureOversampling(uint8_t os);
    bool setPressureOversampling(uint8_t os);
    bool setHumidityOversampling(uint8_t os);
    bool setIIRFilterSize(uint8_t fs);
    bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);

    bool performReading(void);

    /// Temperature (Celsius) assigned after calling performReading()
    float temperature;
    /// Pressure (Pascals) assigned after calling performReading()
    float pressure;
    /// Humidity (RH %) assigned after calling performReading()
    float humidity;
    /// Gas resistor (ohms) assigned after calling performReading()
    float gas_resistance;*/
  private:

    /*bool _filterEnabled, _tempEnabled, _humEnabled, _presEnabled, _gasEnabled;
    uint8_t _i2caddr;
    int32_t _sensorID;
    int8_t _cs;

    uint8_t spixfer(uint8_t x);

    struct bme680_dev gas_sensor;*/
};

#endif
