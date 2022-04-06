#pragma once

typedef enum class _Composition { 
    Single = 0,
    Multi = 1,
} Composition;

typedef enum class _Units { 
    OTHER = 0,
    PPM = 1,
    PPB = 2,
    V = 3,
    BYTE = 4,
} Units;

typedef enum class _ValueType { 
    Numeric = 0,
    Boolean = 1,
    StatusByte = 2,
    Other = 3,
} ValueType;

typedef enum class _SystemType { 
    AQSync = 0,
    AQLite = 1,
    PAM = 2,
    FourOFive = 3,
    Ozone = 4,
    CO2 = 5,
    Weather = 6,
    // Particle = 7,
} SystemType;

typedef enum class _CalParamType { 
    Other = 0,
    Slope = 1,
    Zero = 2,
    Voltage = 3,
} CalParamType;


typedef struct _CalibrationParam { 
    CalParamType parameters[20];
    char name[30];
} CalibrationParam;

typedef struct _SystemSettings { 
    char name[25];
    char ssid[10];
    SystemType type;
    int primaryUploadFrequency;
    int diagnosticUploadFrequency;
    CalibrationParam calParams[20];
} SystemSettings;

typedef struct _Device { 
    char name[15];
    char ssid[25];
    Units units;
} Device;

typedef struct _Diagnostic { 
    char name[15];
    char ssid[25];
    Units units;
} Diagnostic;

typedef struct _SubSystem { 
    char ssid[25];
    char psid[25];
    SystemSettings settings;
} SubSystem;

typedef struct _SystemTopology { 
    Device devices[20];
    Diagnostic diagnostics[10];
    SubSystem subSystems[5];
} SystemTopology;

typedef struct _DataPoint { 
    char name[25];
    char value[15];
    Units units;
    char ssid[25];
    ValueType valueType;
} DataPoint;

typedef struct _Upload { 
    char sid[10];
    long dateUploaded;
    int pointsTaken;
    DataPoint dataPoints[20];
    bool terminated;
    Composition composition;
} Upload;

typedef struct _SystemManifest { 
    char sid[10];
    bool terminating;
    Composition composition;
    SystemTopology topology;
    SystemSettings settings;
} SystemManifest;