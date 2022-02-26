#include "envsensing.h"
#include "battery.h"

/*
template <class T>
EnvSensingChr<T>::config(uint16_t uuid, uint16_t gain, int8_t offset) {
    chrData = 0;
    dataGain = gain;
    dataOffset = offset;
    chr = BLECharacteristic(uuid);
}
*/
template <class T>
T EnvSensingChr<T>::getData(void) {
    return chrData;
}

template <class T>
T EnvSensingChr<T>::getDataGain(void) {
    return chrData * dataGain;
}

template <class T>
void EnvSensingChr<T>::setData(T data) {
    chrData = data;
}

template <class T>
void EnvSensingChr<T>::setup(uint16_t uuid, uint16_t gain, int8_t offset) {
    chrData = 0;
    dataGain = gain;
    dataOffset = offset;
    chr.setProperties(CHR_PROPS_NOTIFY);
    chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    chr.setCccdWriteCallback(&EnvSensingChr<T>::cccdWriteCallback);
    chr.begin();
    chr.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &envSensingDesc,
            ES_MEAS_DESCR_SIZE);
}

template <class T>
void EnvSensingChr<T>::cccdWriteCallback(uint16_t conn_hdl,
        BLECharacteristic* chr, uint16_t cccd_value) {
    // Check if notify was enabled
    if (chr->notifyEnabled(conn_hdl)) {
        state = NOTIFY;
    } else {
        state = NONE;
    }
}

template <class T>
void EnvSensingChr<T>::update(void) {
    switch (state) {
        case NOTIFY:
            chr.notify((uint8_t *) &chrData, sizeof(chrData) + dataOffset);
            break;
        case WRITE:
            chr.write((uint8_t *) &chrData, sizeof(chrData) + dataOffset);
            break;
        default:
            break;
    }
}

/* Force compile to those types */
template class EnvSensingChr<uint8_t>;
template class EnvSensingChr<uint16_t>;
template class EnvSensingChr<int16_t>;
template class EnvSensingChr<uint32_t>;

void EnvSensingSvc::setup(void) {
    _svc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
    _svc.begin();
    temp.setup(UUID16_CHR_TEMPERATURE, 100, 0);
    humid.setup(UUID16_CHR_HUMIDITY, 100, 0);
    co2lv.setup(UUID16_CHR_POLLEN_CONCENTRATION, 1, -1);
    batlv.setup(UUID16_CHR_BATTERY_LEVEL, 1, 0);
    if (scd30.begin()) {
        sensor_ok = true;
    }
    Bluefruit.Advertising.addService(_svc);
}

void EnvSensingSvc::updateMeasurements(int16_t t, uint16_t h,
        uint32_t c, uint8_t b) {
    temp.setData(t);
    humid.setData(h);
    co2lv.setData(c);
    batlv.setData(b);
}

void EnvSensingSvc::service(void) {
    float vbat_mv = readVBAT();
    // Convert from raw mv to percentage (based on LIPO chemistry)
    uint8_t vbat_per = mvToPercent(vbat_mv);

    if (sensor_ok) {
        if (scd30.dataReady()) {
            if (scd30.read()) {
                temp.setData((int16_t) (scd30.temperature + 0.5));
                humid.setData((uint16_t) (scd30.relative_humidity + 0.5));
                co2lv.setData((uint32_t) (scd30.CO2 + 0.5));
            }
            Serial.print("Temperature: ");
            Serial.print(temp.getData());
            Serial.print(" C | Humidity: ");
            Serial.print(humid.getData());
            Serial.print(" % | CO2: ");
            Serial.print(co2lv.getData());
            Serial.println(" ppm ");
        }
    }
    Serial.print("Battery: ");
    Serial.print(vbat_mv);
    Serial.print(" mV ");
    Serial.print(vbat_per);
    Serial.println(" %");
    if ( Bluefruit.connected() ) {
        temp.update();
        humid.update();
        co2lv.update();
        batlv.update();
    }
}
