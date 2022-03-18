/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.6-dev */

#ifndef MY_PB_PAM_MY_PB_H_INCLUDED
#define MY_PB_PAM_MY_PB_H_INCLUDED
#include <MY_pb.h>

#if MY_PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _Composition { 
    Composition_SINGLE = 0, 
    Composition_MULTI_PART = 1 
} Composition;

typedef enum _CalParamType { 
    CalParamType_Slope = 0, 
    CalParamType_Zero = 1, 
    CalParamType_Voltage = 2, 
    CalParamType_Other = 3 
} CalParamType;

typedef enum _SystemType { 
    SystemType_AQSYNC = 0, 
    SystemType_AQLITE = 1, 
    SystemType_PAM = 2, 
    SystemType_FOUR_ZERO_FIVE = 3, 
    SystemType_OZONE = 4, 
    SystemType_CO2 = 5, 
    SystemType_WEATHER = 6, 
    SystemType_PARTICLE = 7 
} SystemType;

typedef enum _Units { 
    Units_OTHER = 0, 
    Units_PPM = 1, 
    Units_PPB = 2, 
    Units_V = 3, 
    Units_BYTE = 4 
} Units;

/* Struct definitions */
typedef struct _CalibrationParam { 
    MY_pb_size_t parameter_count;
    CalParamType parameter[20]; 
    char name[31]; 
} CalibrationParam;

typedef struct _DataPoint { 
    char name[26]; 
    char value[12]; 
    Units units; 
    char SSID[26]; 
} DataPoint;

typedef struct _Device { 
    char name[16]; 
    char SSID[26]; 
    Units units; 
} Device;

typedef struct _Diagnostic { 
    char name[26]; 
    char SSID[26]; 
    Units units; 
} Diagnostic;

typedef struct _SystemSettings { 
    char name[26]; 
    char serial[11]; 
    SystemType type; 
    int32_t primaryUploadFrequeny; 
    int32_t diagnosticUploadFrequeny; 
    MY_pb_size_t calParams_count;
    CalibrationParam calParams[20]; 
} SystemSettings;

typedef struct _Upload { 
    char SID[11]; 
    int32_t dateUploaded; 
    int32_t pointsTaken; 
    MY_pb_size_t dataPoints_count;
    DataPoint dataPoints[20]; 
    bool terminated; 
    Composition composition; 
} Upload;

typedef struct _SubSystem { 
    char SSID[26]; 
    bool has_settings;
    SystemSettings settings; 
    char PSID[26]; 
} SubSystem;

typedef struct _SystemTopology { 
    MY_pb_size_t devices_count;
    Device devices[20]; 
    MY_pb_size_t diagnostics_count;
    Diagnostic diagnostics[10]; 
    MY_pb_size_t subSystems_count;
    SubSystem subSystems[3]; 
} SystemTopology;

typedef struct _SystemManifest { 
    char SID[11]; 
    bool has_topology;
    SystemTopology topology; 
    bool has_settings;
    SystemSettings settings; 
    bool terminating; 
    Composition composition; 
} SystemManifest;


/* Helper constants for enums */
#define _Composition_MIN Composition_SINGLE
#define _Composition_MAX Composition_MULTI_PART
#define _Composition_ARRAYSIZE ((Composition)(Composition_MULTI_PART+1))

#define _CalParamType_MIN CalParamType_Slope
#define _CalParamType_MAX CalParamType_Other
#define _CalParamType_ARRAYSIZE ((CalParamType)(CalParamType_Other+1))

#define _SystemType_MIN SystemType_AQSYNC
#define _SystemType_MAX SystemType_PARTICLE
#define _SystemType_ARRAYSIZE ((SystemType)(SystemType_PARTICLE+1))

#define _Units_MIN Units_OTHER
#define _Units_MAX Units_BYTE
#define _Units_ARRAYSIZE ((Units)(Units_BYTE+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define SystemManifest_init_default              {"", false, SystemTopology_init_default, false, SystemSettings_init_default, 0, _Composition_MIN}
#define SystemTopology_init_default              {0, {Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default, Device_init_default}, 0, {Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default, Diagnostic_init_default}, 0, {SubSystem_init_default, SubSystem_init_default, SubSystem_init_default}}
#define SystemSettings_init_default              {"", "", _SystemType_MIN, 0, 0, 0, {CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default, CalibrationParam_init_default}}
#define CalibrationParam_init_default            {0, {_CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN}, ""}
#define SubSystem_init_default                   {"", false, SystemSettings_init_default, ""}
#define Device_init_default                      {"", "", _Units_MIN}
#define Diagnostic_init_default                  {"", "", _Units_MIN}
#define Upload_init_default                      {"", 0, 0, 0, {DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default, DataPoint_init_default}, 0, _Composition_MIN}
#define DataPoint_init_default                   {"", "", _Units_MIN, ""}
#define SystemManifest_init_zero                 {"", false, SystemTopology_init_zero, false, SystemSettings_init_zero, 0, _Composition_MIN}
#define SystemTopology_init_zero                 {0, {Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero, Device_init_zero}, 0, {Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero, Diagnostic_init_zero}, 0, {SubSystem_init_zero, SubSystem_init_zero, SubSystem_init_zero}}
#define SystemSettings_init_zero                 {"", "", _SystemType_MIN, 0, 0, 0, {CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero, CalibrationParam_init_zero}}
#define CalibrationParam_init_zero               {0, {_CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN, _CalParamType_MIN}, ""}
#define SubSystem_init_zero                      {"", false, SystemSettings_init_zero, ""}
#define Device_init_zero                         {"", "", _Units_MIN}
#define Diagnostic_init_zero                     {"", "", _Units_MIN}
#define Upload_init_zero                         {"", 0, 0, 0, {DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero, DataPoint_init_zero}, 0, _Composition_MIN}
#define DataPoint_init_zero                      {"", "", _Units_MIN, ""}

/* Field tags (for use in manual encoding/decoding) */
#define CalibrationParam_parameter_tag           1
#define CalibrationParam_name_tag                2
#define DataPoint_name_tag                       1
#define DataPoint_value_tag                      2
#define DataPoint_units_tag                      3
#define DataPoint_SSID_tag                       4
#define Device_name_tag                          1
#define Device_SSID_tag                          2
#define Device_units_tag                         3
#define Diagnostic_name_tag                      1
#define Diagnostic_SSID_tag                      2
#define Diagnostic_units_tag                     3
#define SystemSettings_name_tag                  1
#define SystemSettings_serial_tag                2
#define SystemSettings_type_tag                  3
#define SystemSettings_primaryUploadFrequeny_tag 4
#define SystemSettings_diagnosticUploadFrequeny_tag 5
#define SystemSettings_calParams_tag             6
#define Upload_SID_tag                           1
#define Upload_dateUploaded_tag                  2
#define Upload_pointsTaken_tag                   3
#define Upload_dataPoints_tag                    4
#define Upload_terminated_tag                    5
#define Upload_composition_tag                   6
#define SubSystem_SSID_tag                       1
#define SubSystem_settings_tag                   2
#define SubSystem_PSID_tag                       3
#define SystemTopology_devices_tag               1
#define SystemTopology_diagnostics_tag           2
#define SystemTopology_subSystems_tag            3
#define SystemManifest_SID_tag                   1
#define SystemManifest_topology_tag              2
#define SystemManifest_settings_tag              3
#define SystemManifest_terminating_tag           4
#define SystemManifest_composition_tag           5

/* Struct field encoding specification for nanopb */
#define SystemManifest_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   SID,               1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  topology,          2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  settings,          3) \
X(a, STATIC,   SINGULAR, BOOL,     terminating,       4) \
X(a, STATIC,   SINGULAR, UENUM,    composition,       5)
#define SystemManifest_CALLBACK NULL
#define SystemManifest_DEFAULT NULL
#define SystemManifest_topology_MSGTYPE SystemTopology
#define SystemManifest_settings_MSGTYPE SystemSettings

#define SystemTopology_FIELDLIST(X, a) \
X(a, STATIC,   REPEATED, MESSAGE,  devices,           1) \
X(a, STATIC,   REPEATED, MESSAGE,  diagnostics,       2) \
X(a, STATIC,   REPEATED, MESSAGE,  subSystems,        3)
#define SystemTopology_CALLBACK NULL
#define SystemTopology_DEFAULT NULL
#define SystemTopology_devices_MSGTYPE Device
#define SystemTopology_diagnostics_MSGTYPE Diagnostic
#define SystemTopology_subSystems_MSGTYPE SubSystem

#define SystemSettings_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   name,              1) \
X(a, STATIC,   SINGULAR, STRING,   serial,            2) \
X(a, STATIC,   SINGULAR, UENUM,    type,              3) \
X(a, STATIC,   SINGULAR, INT32,    primaryUploadFrequeny,   4) \
X(a, STATIC,   SINGULAR, INT32,    diagnosticUploadFrequeny,   5) \
X(a, STATIC,   REPEATED, MESSAGE,  calParams,         6)
#define SystemSettings_CALLBACK NULL
#define SystemSettings_DEFAULT NULL
#define SystemSettings_calParams_MSGTYPE CalibrationParam

#define CalibrationParam_FIELDLIST(X, a) \
X(a, STATIC,   REPEATED, UENUM,    parameter,         1) \
X(a, STATIC,   SINGULAR, STRING,   name,              2)
#define CalibrationParam_CALLBACK NULL
#define CalibrationParam_DEFAULT NULL

#define SubSystem_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   SSID,              1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  settings,          2) \
X(a, STATIC,   SINGULAR, STRING,   PSID,              3)
#define SubSystem_CALLBACK NULL
#define SubSystem_DEFAULT NULL
#define SubSystem_settings_MSGTYPE SystemSettings

#define Device_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   name,              1) \
X(a, STATIC,   SINGULAR, STRING,   SSID,              2) \
X(a, STATIC,   SINGULAR, UENUM,    units,             3)
#define Device_CALLBACK NULL
#define Device_DEFAULT NULL

#define Diagnostic_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   name,              1) \
X(a, STATIC,   SINGULAR, STRING,   SSID,              2) \
X(a, STATIC,   SINGULAR, UENUM,    units,             3)
#define Diagnostic_CALLBACK NULL
#define Diagnostic_DEFAULT NULL

#define Upload_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   SID,               1) \
X(a, STATIC,   SINGULAR, INT32,    dateUploaded,      2) \
X(a, STATIC,   SINGULAR, INT32,    pointsTaken,       3) \
X(a, STATIC,   REPEATED, MESSAGE,  dataPoints,        4) \
X(a, STATIC,   SINGULAR, BOOL,     terminated,        5) \
X(a, STATIC,   SINGULAR, UENUM,    composition,       6)
#define Upload_CALLBACK NULL
#define Upload_DEFAULT NULL
#define Upload_dataPoints_MSGTYPE DataPoint

#define DataPoint_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   name,              1) \
X(a, STATIC,   SINGULAR, STRING,   value,             2) \
X(a, STATIC,   SINGULAR, UENUM,    units,             3) \
X(a, STATIC,   SINGULAR, STRING,   SSID,              4)
#define DataPoint_CALLBACK NULL
#define DataPoint_DEFAULT NULL

extern const MY_pb_msgdesc_t SystemManifest_msg;
extern const MY_pb_msgdesc_t SystemTopology_msg;
extern const MY_pb_msgdesc_t SystemSettings_msg;
extern const MY_pb_msgdesc_t CalibrationParam_msg;
extern const MY_pb_msgdesc_t SubSystem_msg;
extern const MY_pb_msgdesc_t Device_msg;
extern const MY_pb_msgdesc_t Diagnostic_msg;
extern const MY_pb_msgdesc_t Upload_msg;
extern const MY_pb_msgdesc_t DataPoint_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define SystemManifest_fields &SystemManifest_msg
#define SystemTopology_fields &SystemTopology_msg
#define SystemSettings_fields &SystemSettings_msg
#define CalibrationParam_fields &CalibrationParam_msg
#define SubSystem_fields &SubSystem_msg
#define Device_fields &Device_msg
#define Diagnostic_fields &Diagnostic_msg
#define Upload_fields &Upload_msg
#define DataPoint_fields &DataPoint_msg

/* Maximum encoded size of messages (where known) */
#define CalibrationParam_size                    72
#define DataPoint_size                           69
#define Device_size                              46
#define Diagnostic_size                          56
#define SubSystem_size                           1600
#define SystemManifest_size                      7914
#define SystemSettings_size                      1543
#define SystemTopology_size                      6349
#define Upload_size                              1458

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif