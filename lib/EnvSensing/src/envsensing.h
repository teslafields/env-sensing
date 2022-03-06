#ifndef ENVSENSING_H
#define ENVSENSING_H

#include <bluefruit.h>
#include <Adafruit_SCD30.h>

#define UUID_CHR_DESCRIPTOR_ES_MEAS  0x290C
#define ES_MEAS_DESCR_SIZE 11

typedef enum BLECharsProperties ChrProps;
/*
    BLEBroadcast            = 0x01
    BLERead                 = 0x02
    BLEWriteWithoutResponse = 0x04
    BLEWrite                = 0x08
    BLENotify               = 0x10
    BLEIndicate             = 0x20
*/

typedef enum ChrSensingState {
    NONE,
    WRITE,
    NOTIFY
} chrSensingState;

template <class T>
class EnvSensingChr {
  private:
    T chrData;
    int8_t dataOffset;
    uint16_t dataGain;
    chrSensingState state;
    BLECharacteristic chr;
    uint16_t connHdl;
    bool writeFlagged = false;
    //void cccdWriteCallback(uint16_t conn_hdl,
    //        BLECharacteristic* chr, uint16_t cccd_value);

  public:
    T getData();
    T getDataGain();
    void setData(T data);
    void setState(chrSensingState st);
    void setConnectionHandle(uint16_t conn);
    void setup(uint16_t uuid, uint16_t gain, int8_t offset);
    void update(void);
};

class EnvSensingSvc {
    private:
        static uint16_t         connHdl;
        BLEService              _svc;
        EnvSensingChr<int16_t>  temp;
        EnvSensingChr<uint16_t> humid;
        EnvSensingChr<uint32_t> co2lv;
        EnvSensingChr<uint8_t>  batlv;
        /* Sensor SCD30 */
        bool                    sensor_ok = false;
        Adafruit_SCD30          scd30;
    public:
        void service(void);
        void setup(void);
        BLEService& getBLEService(void);
        void updateMeasurements(int16_t t, uint16_t h, uint32_t c, uint8_t b);
        static void connectCallback(uint16_t conn_handle);
        static void disconnectCallback(uint16_t conn_handle, uint8_t reason);
};



void cccdWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);

extern uint8_t envSensingDesc[];

#endif


