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
    static chrSensingState state;
    static BLECharacteristic chr;
    static void cccdWriteCallback(uint16_t conn_hdl,
            BLECharacteristic* chr, uint16_t cccd_value);

  public:
    T getData();
    T getDataGain();
    void setData(T data);
    void setup(uint16_t uuid, uint16_t gain, int8_t offset);
    void update(void);
};

class EnvSensingSvc {
    private:
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
        void updateMeasurements(int16_t t, uint16_t h, uint32_t c, uint8_t b);
};

extern uint8_t envSensingDesc[];

#endif


