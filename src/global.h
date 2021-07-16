#ifndef __GLOBAL_H__
#define __GLOBAL_H__

class Global {

public:
    static Global* GetInstance();
    int device_id;
    bool ozone_enabled;
    String fileName;

    union{
        uint16_t status_int;
        char byte[2];
    } * status_word;

    private: 

        Global() { }
        ~Global() { }


};

#endif // __SENDING_DATA_H__