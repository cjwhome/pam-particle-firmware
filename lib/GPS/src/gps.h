
#include "application.h"
//gps sentence
#define TIME_FIELD_INDEX 0
#define LATITUDE_FIELD_INDEX 1
#define NORTH_SOUTH_FIELD_INDEX 2
#define LONGITUDE_FIELD_INDEX 3
#define EAST_WEST_FIELD_INDEX 4
#define GPS_QUALITY_FIELD_INDEX 5
#define NUMBER_OF_SATELLITES_INDEX 6
#define HORIZONTAL_DILUTION_INDEX 7
#define ALTITUDE_FIELD_INDEX 8
#define DATE_FIELD_INDEX 6

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
    float horizontal_dilution;     //00.0-99.9

public:
    //(d)dd + (mm.mmmm/60) (* -1 for W and S)
    void set_lat_decimal(String latString, char nsString);
    void set_long_decimal(String longString, char ewString);
    double get_latitude(void);
    double get_longitude(void);
    void set_satellites(String satString);
    void set_horizontalDilution(String hdString);
    int get_satellites(void);
    int get_horizontalDilution(void);
    int16_t get_latitudeWhole(void);
    int16_t get_latitudeFrac(void);
    int16_t get_longitudeWhole(void);
    int16_t get_longitudeFrac(void);
    int8_t get_nsIndicator(void);
    int8_t get_ewIndicator(void);
};
