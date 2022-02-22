#include "envsensing.h"



template <class T>
EnvSensingChr<T>::EnvSensingChr(uint16_t uuid, uint16_t gain) {
    _data = 0;
    _gain = gain;
    chr = BLECharacteristic(uuid);
}

template <class T>
T EnvSensingChr<T>::getData(void) {
    return _data;
}

template <class T>
T EnvSensingChr<T>::getDataGain(void) {
    return _data * _gain;
}

template <class T>
void EnvSensingChr<T>::setData(T data) {
    _data = data;
}

/* Force compile to those types */
template class EnvSensingChr<uint8_t>;
template class EnvSensingChr<uint16_t>;
template class EnvSensingChr<int16_t>;
template class EnvSensingChr<uint32_t>;

void EnvSensingSvc::setup(void) {
    _svc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
    _svc.begin();
    setupChr(&temp.chr);
    setupChr(&humid.chr);
    setupChr(&co2lv.chr);
    setupChr(&batlv.chr);
}

typedef void (*write_cccd_cb) (uint16_t conn_hdl,
        BLECharacteristic* chr, uint16_t value);

void EnvSensingSvc::setupChr(BLECharacteristic *chr) {
    write_cccd_cb cb = &EnvSensingSvc::cccdWriteCallback;
    chr->setProperties(CHR_PROPS_NOTIFY);
    chr->setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    chr->setCccdWriteCallback(cb);
    chr->begin();
    chr->addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &envSensingDesc,
            ES_MEAS_DESCR_SIZE);
}

void EnvSensingSvc::updateChr(BLECharacteristic* chr,
        uint8_t op, uint8_t *d_ptr, uint8_t n) {
    if (op == WRITE_OP) {
        chr->write(d_ptr, n);
    }
    else if (op == NOTIFY_OP) {
        if (!chr->notify(d_ptr, n)) {
            uint16_t uuid;
            chr->uuid.get(&uuid);
            Serial.print("ERR: Notify not set in CCCD or not connected! ");
            Serial.println(uuid, HEX);
        }
    }
}

void EnvSensingSvc::updateMeasurements(int16_t t, uint16_t h,
        uint32_t c, uint8_t b) {
    temp.setData(t);
    humid.setData(h);
    co2lv.setData(c);
    batlv.setData(b);
}

void EnvSensingSvc::cccdWriteCallback(uint16_t conn_hdl,
        BLECharacteristic* chr, uint16_t cccd_value) {

    if (chr->uuid == temp.chr.uuid) {
        Serial.print("Temperature Characteristic ");
    }
    else if (chr->uuid == humid.chr.uuid) {
        Serial.print("Humidity Characteristic ");
    }
    else if (chr->uuid == co2lv.chr.uuid) {
        Serial.print("CO2 Characteristic ");
    }
    else if (chr->uuid == batlv.chr.uuid) {
        Serial.print("Battery Level Characteristic ");
    }

    // Check if notify was enabled for the given characteristic
    if (chr->notifyEnabled(conn_hdl)) {
        Serial.println("'Notify' enabled");
    } else {
        Serial.println("'Notify' disabled");
    }
}


