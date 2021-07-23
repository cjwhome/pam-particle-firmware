#include "global.h"

Global::Global()
{
    EEPROM.get(DEVICE_ID_MEM_ADDRESS, device_id);
    fileName = String(device_id) + "_" + String(Time.year()) + String(Time.month()) + String(Time.day()) + "_" + String(Time.hour()) + String(Time.minute()) + String(Time.second()) + ".txt";
    logFileName = "log_" + fileName;
    app_version = APP_VERSION;
    build_version = BUILD_VERSION;
}

Global::~Global() {}

Global * Global::instance = nullptr;
Global* Global::GetInstance()
{
    if (instance == nullptr) {
        instance = new Global();
    }
    return instance;
}

void Global::writeLogFile(String data){
  if (sd.begin(CS)){
      Serial.println("Writing data to log file.");
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