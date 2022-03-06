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
void EnvSensingChr<T>::setState(chrSensingState st) {
    state = st;
}

template <class T>
void EnvSensingChr<T>::setup(uint16_t uuid, uint16_t gain, int8_t offset) {
    chrData = 0;
    dataGain = gain;
    dataOffset = offset;
    chr = BLECharacteristic(uuid);
    chr.setProperties(CHR_PROPS_NOTIFY);
    chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    chr.setCccdWriteCallback(&cccdWriteCallback);
    chr.begin();
    chr.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &envSensingDesc,
            ES_MEAS_DESCR_SIZE);
    writeFlagged = true;
    this->update();
}

//template <class T>
//void EnvSensingChr<T>::cccdWriteCallback(uint16_t conn_hdl,
void cccdWriteCallback(uint16_t conn_hdl,
        BLECharacteristic* chr, uint16_t cccd_value) {
    // Check if notify was enabled
    if (chr->notifyEnabled(conn_hdl)) {
    } else {
    }
}

template <class T>
void EnvSensingChr<T>::update(void) {
    if (chr.notifyEnabled()) {
        state = NOTIFY;
    } else if (writeFlagged) {
        state = WRITE;
        writeFlagged = false;
    } else {
        state = NONE;
    }

    uint16_t ret;
    T data = this->getDataGain();

    switch (state) {
        case NOTIFY:
            ret = chr.notify((uint8_t *) &data,
                    sizeof(chrData) + dataOffset);
            Serial.print("ChrNotify returned: ");
            Serial.println(ret);
            break;
        case WRITE:
            ret = chr.write((uint8_t *) &chrData,
                    sizeof(chrData) + dataOffset);
            Serial.print("ChrWrite returned: ");
            Serial.println(ret);
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

uint16_t EnvSensingSvc::connHdl = 0;

void EnvSensingSvc::setup(void) {
    connHdl = 0;
    _svc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
    _svc.begin();
    temp.setup(UUID16_CHR_TEMPERATURE, 100, 0);
    humid.setup(UUID16_CHR_HUMIDITY, 100, 0);
    co2lv.setup(UUID16_CHR_POLLEN_CONCENTRATION, 1, -1);
    batlv.setup(UUID16_CHR_BATTERY_LEVEL, 1, 0);
    if (scd30.begin()) {
        sensor_ok = true;
    }
}

void EnvSensingSvc::updateMeasurements(int16_t t, uint16_t h,
        uint32_t c, uint8_t b) {
    temp.setData(t);
    humid.setData(h);
    co2lv.setData(c);
    batlv.setData(b);
}

BLEService& EnvSensingSvc::getBLEService(void) {
    return _svc;
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
            Serial.print(" ppm ");
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

void EnvSensingSvc::connectCallback(uint16_t conn_handle)
{
    // Get the reference to current connection
    connHdl = conn_handle;
    BLEConnection* connection = Bluefruit.Connection(conn_handle);
    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));
    Serial.print("Connected to ");
    Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void EnvSensingSvc::disconnectCallback(uint16_t conn_handle, uint8_t reason)
{
    connHdl = 0;
    (void) conn_handle;
    (void) reason;
    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);
}
