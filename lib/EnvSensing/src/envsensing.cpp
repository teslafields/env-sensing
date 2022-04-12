#include "envsensing.h"
#include "battery.h"


template <class T>
EnvSensingChr<T>::EnvSensingChr(uint16_t uuid, uint16_t gain, int8_t gatt_offset,
        int8_t adv_offset) {
    chrData = 0;
    dataGain = gain;
    gattLenOffset = gatt_offset;
    advLenOffset = adv_offset;
    chrUuid = uuid;
    bleChr = BLECharacteristic(uuid);
}

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
serviceData EnvSensingChr<T>::getAdvServiceData(void) {
    serviceData adv_data = {
        .data = {0},
        .n = (uint8_t) (sizeof(T) + advLenOffset),
    };
    uint8_t uuidlow = (uint8_t) chrUuid & 0xff;
    uint8_t uuidhigh = (uint8_t) (chrUuid >> 8) & 0xff;
    adv_data.data[0] = uuidlow;
    adv_data.data[1] = uuidhigh;
    adv_data.n += 2;
    T chr_data = getDataGain();
    uint8_t* ptr_data = (uint8_t *) &chr_data;

    /* Check boundaries of the advertising packet size */
    if (adv_data.n > ADV_SVC_DATA_LEN - 2) {
        adv_data.n = ADV_SVC_DATA_LEN;
    }
    for (int i = 2; i < adv_data.n; i++) {
        adv_data.data[i] = *ptr_data++;
    }
    return adv_data;
}

template <class T>
bool EnvSensingChr<T>::notifyEnabled(void) {
    return bleChr.notifyEnabled();
}

template <class T>
void EnvSensingChr<T>::setup() {
    bleChr.setProperties(CHR_PROPS_NOTIFY);
    bleChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    bleChr.setCccdWriteCallback(&cccdWriteCallback);
    bleChr.begin();
    bleChr.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &envSensingDesc,
            ES_MEAS_DESCR_SIZE);
    writeFlagged = true;
    this->update();
}

void cccdWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr,
        uint16_t cccd_value) {
    // Check if notify was enabled
    if (chr->notifyEnabled(conn_hdl)) {
    } else {
    }
}

template <class T>
bool EnvSensingChr<T>::notify(uint8_t *data, uint16_t len) {
    return bleChr.notify(data, len + gattLenOffset);
}

template <class T>
uint16_t EnvSensingChr<T>::update(void) {
    if (this->notifyEnabled()) {
        state = NOTIFY;
    } else if (writeFlagged) {
        state = WRITE;
        writeFlagged = false;
    } else {
        state = NONE;
    }

    T data = this->getDataGain();
    uint16_t ret = 0;

    switch (state) {
        case NOTIFY:
            ret = (uint16_t) this->notify((uint8_t *) &data, sizeof(chrData));
            break;
        case WRITE:
            ret = bleChr.write((uint8_t *) &chrData,
                    sizeof(chrData) + gattLenOffset);
            break;
        default:
            break;
    }
    return ret;
}

/* Force compile to those types */
template class EnvSensingChr<uint8_t>;
template class EnvSensingChr<uint16_t>;
template class EnvSensingChr<int16_t>;
template class EnvSensingChr<uint32_t>;

uint16_t EnvSensingSvc::connHdl = 0;
bool EnvSensingSvc::justConnected = false;
uint8_t storage_data_buffer[256] = {0};

/* Init ESS and its associated characteristics
 * Environmental Sensing Service UUID is 0x181A
 * GATT Characteristic 0x2A6E Temperature
 * GATT Characteristic 0x2A6F Humidity
 * GATT Characteristic 0x2BD0 CO concentration
 *  But SIGs 16-uuid oficial page does not have CO2, so we use POLLEN
 *  which is an equivalent - 0x2A75.
 */
EnvSensingSvc::EnvSensingSvc() : temp(UUID16_CHR_TEMPERATURE, 100),
        humid(UUID16_CHR_HUMIDITY, 100),
        co2lv(UUID16_CHR_POLLEN_CONCENTRATION, 1, -1, -2),
        batlv(UUID16_CHR_BATTERY_LEVEL, 1) {
    svc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
}

void EnvSensingSvc::updateMeasurements(int16_t t, uint16_t h,
        uint32_t c, uint8_t b) {
    temp.setData(t);
    humid.setData(h);
    co2lv.setData(c);
    batlv.setData(b);
}

uint32_t write_count = 1;

bool EnvSensingSvc::storeTemperature(uint32_t ts, int16_t val_temp) {
    if (storage.is_read_mode())
        if (!storage.open_to_write())
            return false;
    Serial.print(write_count);
    Serial.println(" Writing temperature to file..");
    static uint8_t *d_ptr = (uint8_t *) ts;
    if (!storage.write(d_ptr, sizeof(ts)))
        return false;
    // const uint8_t *sep = (uint8_t *) ";";
    // const uint8_t *del = (uint8_t *) "\n";
    d_ptr = (uint8_t *) &val_temp;
    if (!storage.write((uint8_t *) ";", 1))
        return false;
    if (!storage.write(d_ptr, sizeof(val_temp)))
        return false;
    if (!storage.write((uint8_t *) "\n", 1))
        return false;
    Serial.print("OK ");
    Serial.println(storage.fsize());
    write_count++;
    return true;
}

bool EnvSensingSvc::clearStorage(void) {
    return storage.removeFile();
}

BLEService& EnvSensingSvc::getBLEService(void) {
    return svc;
}

bool EnvSensingSvc::recalibrateSensor(void) {
    bool result = false;
    if (scd30.setMeasurementInterval(2)) {
        if (scd30.forceRecalibrationWithReference(CO2_REFERENCE)) {
            Serial.println("SCD30 recalibration started!");
            recalibrationFlag = true;
            recalibrationTs = millis();
            result = true;
        } else {
            scd30.setMeasurementInterval(SCD30_DEF_INTERVAL);
        }
    }
    return result;
}

void EnvSensingSvc::startAdvertising(void) {
    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(svc);

    serviceData chr_adv_data = temp.getAdvServiceData();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA,
            chr_adv_data.data, chr_adv_data.n);

    chr_adv_data = humid.getAdvServiceData();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA,
            chr_adv_data.data, chr_adv_data.n);

    chr_adv_data = co2lv.getAdvServiceData();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA,
            chr_adv_data.data, chr_adv_data.n);

    chr_adv_data = batlv.getAdvServiceData();
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA,
            chr_adv_data.data, chr_adv_data.n);

#ifdef DEBUG_ADV
    Serial.print("ADV PKT: [");
    uint8_t *data = Bluefruit.Advertising.getData();
    for (int i=0; i<BLE_GAP_ADV_SET_DATA_SIZE_MAX; i++) {
        Serial.print(*data++, HEX);
        Serial.print(" ");
    }
    Serial.println("]");
#endif

    Bluefruit.ScanResponse.addName();

    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     * - Fixed interval: 500 ms -> fast = slow = 500 ms
     * - setInterval argument in units of 0.625 ms
     */
    Bluefruit.Advertising.setInterval(800, 800);
    Bluefruit.Advertising.restartOnDisconnect(true);
    //Bluefruit.Advertising.setStopCallback(adv_stop_callback);
    Bluefruit.Advertising.start(ADV_TIMEOUT);
}

void EnvSensingSvc::setupSensor(void) {
    if (scd30.begin()) {
        sensor_ok = true;
        scd30.setMeasurementInterval(SCD30_DEF_INTERVAL);
    }
}

void EnvSensingSvc::setup(void) {
    connHdl = 0;

    /* Setup the LittleFS storage */
    storage.setup();

    /* Setup ADC */
    setupADC();

    Serial.println("\nBluefruit52 GATT ESS");
    Serial.println("--------------------------");
    Serial.println("Configuring the Environmental Sensing Service");

    /* Init Bluefruit lib */
    Bluefruit.begin();

    Bluefruit.autoConnLed(false);
    Bluefruit.setTxPower(0);

    svc.begin();

    temp.setup();
    humid.setup();
    co2lv.setup();
    batlv.setup();

    /* Set the connect/disconnect callback handlers */
    Bluefruit.Periph.setConnectCallback(&EnvSensingSvc::connectCallback);
    Bluefruit.Periph.setDisconnectCallback(&EnvSensingSvc::disconnectCallback);

    /* Make sure sensor is initialized */
    this->setupSensor();

    /* Init with a valid timestamp */
    storeMeasurementTs = millis();
}

void EnvSensingSvc::service(void) {
    digitalWrite(LED_BLUE, HIGH);

    static float vbat_mv;
    static uint8_t vbat_per;
    static int16_t val_temp;
    static uint16_t val_hum;
    static uint32_t val_co2;
    static uint32_t ts_now;
    static size_t n;
    static uint32_t pos;

    ts_now = millis();

    vbat_mv = readVBAT();
    // Convert from raw mv to percentage (based on LIPO chemistry)
    vbat_per =  mvToPercent(vbat_mv);

    if (!sensor_ok) {
        Serial.println("Sensor not OK, initializing again..");
        this->setupSensor();
    } else {
        if (recalibrationFlag) {
            if (ts_now - recalibrationTs > recalibrationInterval) {
                if (scd30.setMeasurementInterval(SCD30_DEF_INTERVAL)) {
                    Serial.println("SCD30 recalibration finished!");
                    recalibrationFlag = false;
                    blink_led(LED_RED);
                } else
                    Serial.println("SCD30 error seting interval!");

            }
        } else if (scd30.dataReady()) {
            if (scd30.read()) {
                temp.setData((int16_t) (scd30.temperature + 0.5));
                humid.setData((uint16_t) (scd30.relative_humidity + 0.5));
                co2lv.setData((uint32_t) (scd30.CO2 + 0.5));
                batlv.setData(vbat_per);
            }
#ifdef DEBUG_MEASURE
            Serial.print("Temperature: ");
            Serial.print(temp.getData());
            Serial.print(" C | Humidity: ");
            Serial.print(humid.getData());
            Serial.print(" % | CO2: ");
            Serial.print(co2lv.getData());
            Serial.print(" ppm | ");
            Serial.print("Battery: ");
            Serial.print(vbat_mv);
            Serial.print(" mV ");
            Serial.print(vbat_per);
            Serial.println(" %");
#endif
        }
    }

    if ( Bluefruit.connected() ) {
        if (justConnected && storage.is_file_open()) {
            if (temp.notifyEnabled()) {
                Serial.println("Sending out stored values..");
                if (storage.is_write_mode())
                    storage.open_to_read();
                /* Loop here until read returns eof */
                storage.restore_position();
                while (!storage.is_eof()) {
                    pos = storage.ftell();
                    Serial.print("Reading file, current postition: ");
                    Serial.println(pos);
                    n = storage.read(storage_data_buffer, 8);
                    if (n == 0)
                        break;
                    if (!temp.notify(storage_data_buffer + 5, sizeof(val_temp))) {
                        Serial.println("Notify error!");
                        if (storage.fseek_set(pos)) {
                            Serial.print("fseek sucessfully set to: ");
                            Serial.println(pos);
                        }
                        break;
                    }
                    /* Give a short delay between notifies */
                    delay(100);
                }
                storage.save_position();
                /* update the TS to store 30s from now */
                storeMeasurementTs = ts_now;
                justConnected = false;
            }
        }

        temp.update();
        humid.update();
        co2lv.update();
        batlv.update();
    } else if ( !Bluefruit.Advertising.isRunning() ) {
        startAdvertising();
    } else if (ts_now - storeMeasurementTs > storeMeasurementInterval) {
        this->storeTemperature(ts_now, temp.getDataGain());
        storeMeasurementTs = ts_now;
    }
    digitalWrite(LED_BLUE, LOW);
}

/*
 * Callback invoked when a connection is established
 * @param conn_handle connection where this event happens
 */
void EnvSensingSvc::connectCallback(uint16_t conn_handle)
{
    // Get the reference to current connection
    connHdl = conn_handle;
    justConnected = true;
    BLEConnection* connection = Bluefruit.Connection(conn_handle);
    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));
    Serial.print("Connected to ");
    Serial.println(central_name);
}

/*
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void EnvSensingSvc::disconnectCallback(uint16_t conn_handle, uint8_t reason)
{
    connHdl = 0;
    justConnected = false;
    (void) conn_handle;
    (void) reason;
    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);
}

void blink_led(uint8_t led_pin) {
    digitalWrite(led_pin, HIGH);
    delay(250);
    digitalWrite(led_pin, LOW);
    delay(250);
}

