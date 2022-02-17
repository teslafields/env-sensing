#ifndef ENVSENSING_H
#define ENVSENSING_H

#include <bluefruit.h>

#define UUID_CHR_DESCRIPTOR_ES_MEAS  0x290C
#define ES_MEAS_DESCR_SIZE 11
#define WRITE_OP 1
#define NOTIFY_OP 2

template <class T>
class EnvSensingChr {
  private:
    T _data;
    uint16_t _gain;

  public:
    EnvSensingChr(uint16_t uuid, uint16_t gain);
    T getData();
    T getDataGain();
    void setData(T data);
    BLECharacteristic chr;
};

class EnvSensingSvc {
    private:
        BLEService _svc;
        void setupChr(BLECharacteristic *chr);
    public:
        EnvSensingChr<int16_t>  temp;
        EnvSensingChr<uint16_t> humid;
        EnvSensingChr<uint32_t> co2lv;
        EnvSensingChr<uint8_t>  batlv;

        void setup(void);
        void updateMeasurements(int16_t t, uint16_t h, uint32_t c, uint8_t b);
        void cccdWriteCallback(uint16_t conn_hdl,
                BLECharacteristic* chr, uint16_t cccd_value);
        void updateChr(BLECharacteristic* chr,
                uint8_t op, uint8_t *d_ptr, uint8_t n);
};

extern uint8_t envSensingDesc[];
#endif


