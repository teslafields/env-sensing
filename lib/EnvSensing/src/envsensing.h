#ifndef ENVSENSING_H
#define ENVSENSING_H

#include <bluefruit.h>
#include <Adafruit_SCD30.h>
#include "datastorage.h"

#define CO2_REFERENCE 409
#define SCD30_DEF_INTERVAL 2

#define UUID_CHR_DESCRIPTOR_ES_MEAS  0x290C
#define ES_MEAS_DESCR_SIZE 11

#define ADV_TIMEOUT        16
#define ADV_FAST_TIMEOUT   ADV_TIMEOUT/2
#define ADV_SVC_DATA_LEN   6

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

struct adv_service_data {
    uint8_t data[ADV_SVC_DATA_LEN];
    uint8_t n;
};

using serviceData = adv_service_data;

template <class T>
class EnvSensingChr {
  private:
    T chrData;
    int8_t gattLenOffset;
    int8_t advLenOffset;
    uint16_t dataGain;
    chrSensingState state;
    BLECharacteristic bleChr;
    uint16_t chrUuid;
    uint16_t connHdl;
    bool writeFlagged = false;

    //void cccdWriteCallback(uint16_t conn_hdl,
    //        BLECharacteristic* chr, uint16_t cccd_value);

  public:
    EnvSensingChr(uint16_t uuid, uint16_t gain, int8_t gatt_offset=0,
            int8_t adv_offset=0);
    T getData();
    T getDataGain();
    void setData(T data);
    void setState(chrSensingState st);
    void setConnectionHandle(uint16_t conn);
    serviceData getAdvServiceData(void);
    void setup();
    uint16_t update(void);
    bool notifyEnabled(void);
    bool notify(uint8_t *data, uint16_t len);
};

class EnvSensingSvc {
    public:
        /* Constructor */
        EnvSensingSvc(void);

        void setup(void);
        void setupSensor(void);
        void startAdvertising(void);
        void service(void);
        BLEService& getBLEService(void);
        void updateMeasurements(int16_t t, uint16_t h, uint32_t c,
                uint8_t b);
        bool storeTemperature(uint32_t ts, int16_t temp_val);
        bool clearStorage(void);
        bool recalibrateSensor(void);

        /* Callbacks */
        static void connectCallback(uint16_t conn_handle);
        static void disconnectCallback(uint16_t conn_handle,
                uint8_t reason);
        static void advertisingStopCallback(void);

    private:
        DataStorage storage;
        static uint16_t         connHdl;
        static bool             justConnected;
        BLEService              svc;
        EnvSensingChr<int16_t>  temp;
        EnvSensingChr<uint16_t> humid;
        EnvSensingChr<uint32_t> co2lv;
        EnvSensingChr<uint8_t>  batlv;
        uint32_t                storeMeasurementInterval = 1800*1000; // ms
        uint32_t                storeMeasurementTs = 0;

        /* Sensor SCD30 */
        bool                sensor_ok = false;
        Adafruit_SCD30      scd30;
        bool                recalibrationFlag = false;
        uint32_t            recalibrationInterval = 300*1000; // ms
        uint32_t            recalibrationTs = 0;

        bool                writeOnce = false;
};



void cccdWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr,
        uint16_t cccd_value);
void blink_led(uint8_t led_pin);

extern uint8_t envSensingDesc[];

#endif

