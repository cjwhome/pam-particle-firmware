#include "global.h"

Global::Global()
{
    EEPROM.get(DEVICE_ID_MEM_ADDRESS, device_id);
    fileName = String(device_id) + "_" + String(Time.year()) + String(Time.month()) + String(Time.day()) + "_" + String(Time.hour()) + String(Time.minute()) + String(Time.second()) + ".txt";
    logFileName = "log_" + fileName;
    app_version = APP_VERSION;
    build_version = BUILD_VERSION;
}

Global::~Global()
{ 
    isInitialized = false; 
}

Global * Global::instance = nullptr;
bool Global::isInitialized = false;
Global* Global::GetInstance()
{
    if (isInitialized == false) {
        isInitialized = true;
        instance = new Global();
    }
    return instance;
}

void Global::writeLogFile(String data){
  if (sd.begin(CS)){
      Serial.println("Writing data to log file.");
      SdFile::dateTimeCallback(this->dateTime);
      log_file.open(logFileName, O_CREAT | O_APPEND | O_WRITE);
      if(log_file_started == 0){
          log_file.println("File Start timestamp: ");
          log_file.println(Time.timeStr());
          log_file_started = 1;
      }
      log_file.println(data);

      log_file.close();
  }else{
    Serial.println("Unable to write to log file");
  }
}

void Global::dateTime(uint16_t* date, uint16_t* time) {

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(Time.year(), Time.month(), Time.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(Time.hour(), Time.minute(), Time.second());
}