#ifdef SCD30
#include <Adafruit_SCD30.h>
Adafruit_SCD30  scd30;
#endif
#include "battery.h"
#include "envsensing.h"

// UUID Characteristc Descriptor for Environment Sensing Measurement
#define UUID_CHR_DESCRIPTOR_ES_MEAS  0x290C
#define UUID16_CHR_CO_CONCENTRATION  0x2BD0 // Bluefruit lacks the UUID for CO

// https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers
// 0x0059 is Nordic
#define MANUFACTURER_ID    0x0059
#define ADV_TIMEOUT        20
#define ADV_FAST_TIMEOUT   ADV_TIMEOUT/2
#define ADV_SVC_DATA_LEN   5

#define TEMPERATURE_IDX    0
#define HUMIDITY_IDX       1
#define CO2_IDX            2

#define WRITE_OP           1
#define NOTIFY_OP          2

typedef enum CtrlIdx {
    TEMPIDX,
    HUMDIDX,
    CO2CIDX,
    VBATIDX,
    ENDIDX,
} controlIdx;

bool start_notify[ENDIDX] = {false};
uint16_t ess_chr_uuid[ENDIDX] = {
    UUID16_CHR_TEMPERATURE, // 0x2A6E
    UUID16_CHR_HUMIDITY, // 0x2A6F
    UUID16_CHR_POLLEN_CONCENTRATION, // 0x2A75 (0x2BD0)
    UUID16_CHR_BATTERY_LEVEL // 0x2A19
};
int16_t ess_chr_gain[ENDIDX] = {
    100, // Temperature multiplier factor
    100, // Humidity factor
    1, // CO2 factor
    1, // Batter level factor
    // More information about this is found at GATT spec
};

// GATT Characteristic 0x2A6E Temperature
// GATT Characteristic 0x2A6F Humidity
// GATT Characteristic 0x2BD0 CO concentration
// But SIGs 16-uuid oficial page does not have CO2, so we use POLLEN
// which is an equivalent - 0x2A75.
BLECharacteristic ess_chr[ENDIDX] = {
    BLECharacteristic(UUID16_CHR_TEMPERATURE),
    BLECharacteristic(UUID16_CHR_HUMIDITY),
    BLECharacteristic(UUID16_CHR_POLLEN_CONCENTRATION),
    BLECharacteristic(UUID16_CHR_BATTERY_LEVEL),
};
// Environmental Sensing Service is 0x181A
BLEService ess = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);

int16_t temperature = 0;
uint16_t humidity = 0;
uint32_t co2ppm = 0;
uint8_t vbatlv = 0;

EnvSensingChr<int16_t>  temperatureChr(UUID16_CHR_TEMPERATURE, 100);
EnvSensingChr<uint16_t> humidityChr(UUID16_CHR_HUMIDITY, 100);
EnvSensingChr<uint32_t> co2ppmChr(UUID16_CHR_POLLEN_CONCENTRATION, 1);
EnvSensingChr<uint8_t> vbatlvChr(UUID16_CHR_BATTERY_LEVEL, 1);

void startAdv(void);
void setupESS(void);
void adv_stop_callback(void);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void connect_callback(uint16_t conn_handle);
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);

/*
void updateCharacteristic(controlIdx idx, uint8_t op, int32_t value, uint8_t n) {
    if (idx >= ENDIDX)
        return;
    value *= ess_chr_gain[idx];
    uint8_t* data = (uint8_t *) &value;
    BLECharacteristic* chr = &ess_chr[idx];
    if (op == WRITE_OP) {
        chr->write(data, n);
    }
    else if (op == NOTIFY_OP) {
        if (!chr->notify(data, n)) {
            uint16_t uuid;
            chr->uuid.get(&uuid);
            Serial.print("ERR: Notify not set in CCCD or not connected! ");
            Serial.println(uuid, HEX);
        }
    }
}
*/
void updateCharacteristic(BLECharacteristic* chr, uint8_t op, uint8_t *d_ptr, uint8_t n) {
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

void updateAdvertisingData(controlIdx idx, int32_t value, uint8_t n) {
    if (idx >= ENDIDX)
        return;
    uint16_t uuid = ess_chr_uuid[idx];
    uint8_t uuidlow = (uint8_t) uuid & 0xff;
    uint8_t uuidhigh = (uint8_t) (uuid >> 8) & 0xff;
    uint8_t svc_uuid[ADV_SVC_DATA_LEN] = {0};
    svc_uuid[0] = uuidlow;
    svc_uuid[1] = uuidhigh;
    value *= ess_chr_gain[idx];
    uint8_t* data = (uint8_t *) &value;
    n = n > ADV_SVC_DATA_LEN - 2 ? ADV_SVC_DATA_LEN : n + 2;
    for (int i = 2; i < n; i++) {
        svc_uuid[i] = *data;
        data++;
    }
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA, &svc_uuid, n);
}

void setup()
{
    Serial.begin(115200);

    // while ( !Serial ) delay(10);

    Serial.println("\nBluefruit52 GATT ESS");
    Serial.println("--------------------------");

#ifdef SCD30
    if (!scd30.begin()) {
        Serial.println("Failed to find SCD30 chip");
        while (1) { delay(10); }
    }
    Serial.println("SCD30 Found!");
    // if (!scd30.setMeasurementInterval(10)){
    //   Serial.println("Failed to set measurement interval");
    //   while(1){ delay(10);}
    // }
    Serial.print("Measurement Interval: ");
    Serial.print(scd30.getMeasurementInterval());
    Serial.println(" seconds");
#endif

    Bluefruit.begin();

    // Set the connect/disconnect callback handlers
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // BLEService and BLECharacteristic classes
    Serial.println("Configuring the Environmental Sensing Service");
    setupESS();

    // Setup the advertising packet(s)
    Serial.println("Setting up the advertising payload(s)");
    startAdv();

    Serial.println("\nAdvertising");
}

void startAdv(void)
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(ess);

    updateAdvertisingData(TEMPIDX, (int32_t) temperature, sizeof(temperature));
    updateAdvertisingData(HUMDIDX, (int32_t) humidity, sizeof(humidity));
    updateAdvertisingData(CO2CIDX, (int32_t) co2ppm, sizeof(co2ppm)-2); // lets send the 4 bytes

    Bluefruit.ScanResponse.addName();
    /* Start Advertising
       - Enable auto advertising if disconnected
       - Timeout for fast mode is 30 seconds
       - Start(timeout) with timeout = 0 will advertise forever (until connected)
       - Fixed interval: 100 ms -> fast = slow = 100 ms
       */
    Bluefruit.Advertising.setStopCallback(adv_stop_callback);
    Bluefruit.Advertising.restartOnDisconnect(true);
    // Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setInterval(160, 160);    // in unit of 0.625 ms
    // Bluefruit.Advertising.setFastTimeout(ADV_FAST_TIMEOUT);
    Bluefruit.Advertising.start(ADV_TIMEOUT);
}

void setupESS(void)
{
    ess.begin();
    // Descriptor of ESS
    uint8_t esm_desc[11] = { 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 }; 
    /*
    for (int i = TEMPIDX; i < ENDIDX; i++) {
        ess_chr[i].setProperties(CHR_PROPS_NOTIFY);
        ess_chr[i].setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
        ess_chr[i].setCccdWriteCallback(cccd_callback);
        ess_chr[i].begin();
        if (ess_chr[i].addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc))) {
            Serial.println("Error addDescriptor call");
        }
    }
    */
    temperatureChr.chr.setProperties(CHR_PROPS_NOTIFY);
    temperatureChr.chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    temperatureChr.chr.setCccdWriteCallback(cccd_callback);
    temperatureChr.chr.begin();
    temperatureChr.chr.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc));

    humidityChr.chr.setProperties(CHR_PROPS_NOTIFY);
    humidityChr.chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    humidityChr.chr.setCccdWriteCallback(cccd_callback);
    humidityChr.chr.begin();
    humidityChr.chr.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc));

    co2ppmChr.chr.setProperties(CHR_PROPS_NOTIFY);
    co2ppmChr.chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    co2ppmChr.chr.setCccdWriteCallback(cccd_callback);
    co2ppmChr.chr.begin();
    co2ppmChr.chr.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc));

    vbatlvChr.chr.setProperties(CHR_PROPS_NOTIFY);
    vbatlvChr.chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    vbatlvChr.chr.setCccdWriteCallback(cccd_callback);
    vbatlvChr.chr.begin();
    vbatlvChr.chr.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc));

    int16_t tempval = temperatureChr.getDataGain();
    updateCharacteristic(&temperatureChr.chr, WRITE_OP, (uint8_t *) &tempval, 2);

    /*
    updateCharacteristic(&humidityChr.chr, WRITE_OP, (int32_t) humidity, sizeof(humidity));
    updateCharacteristic(&co2ppmChr.chr, WRITE_OP, (int32_t) co2ppm, sizeof(co2ppm)-1);
    updateCharacteristic(&vbatlvChr.chr, WRITE_OP, (int32_t) vbatlv, sizeof(vbatlv));

    updateCharacteristic(TEMPIDX, WRITE_OP, (int32_t) temperature, sizeof(temperature));
    updateCharacteristic(HUMDIDX, WRITE_OP, (int32_t) humidity, sizeof(humidity));
    updateCharacteristic(CO2CIDX, WRITE_OP, (int32_t) co2ppm, sizeof(co2ppm)-1);
    updateCharacteristic(VBATIDX, WRITE_OP, (int32_t) vbatlv, sizeof(vbatlv));
    */
}

void adv_stop_callback(void) {
    Bluefruit.Advertising.clearData();
    startAdv();
}

void connect_callback(uint16_t conn_handle)
{
    // Get the reference to current connection
    BLEConnection* connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);
    digitalWrite(1, LED_CONN);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void) conn_handle;
    (void) reason;

    Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
    Serial.println("Advertising!");
    digitalWrite(0, LED_CONN);
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    controlIdx idx = TEMPIDX;

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == ess_chr[TEMPIDX].uuid) {
        Serial.print("Temperature Characteristic ");
    }
    else if (chr->uuid == ess_chr[HUMDIDX].uuid) {
        idx = HUMDIDX;
        Serial.print("Humidity Characteristic ");
    }
    else if (chr->uuid == ess_chr[CO2CIDX].uuid) {
        idx = CO2CIDX;
        Serial.print("CO2 Characteristic ");
    }
    else if (chr->uuid == ess_chr[VBATIDX].uuid) {
        idx = VBATIDX;
        Serial.print("Battery Level Characteristic ");
    }

    if (chr->notifyEnabled(conn_hdl)) {
        start_notify[idx] = true;
        Serial.println("'Notify' enabled");
    } else {
        start_notify[idx] = false;
        Serial.println("'Notify' disabled");
    }
}

void loop()
{
    float vbat_mv = readVBAT();
    // Convert from raw mv to percentage (based on LIPO chemistry)
    uint8_t vbat_per = mvToPercent(vbat_mv);

#ifdef SCD30
    if (scd30.dataReady()){
        if (scd30.read()){
            temperature = (int16_t) (scd30.temperature + 0.5);
            humidity = (uint16_t) (scd30.relative_humidity + 0.5);
            co2ppm = (uint32_t) (scd30.CO2 + 0.5);
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.print(" C | Humidity: ");
            Serial.print(humidity);
            Serial.print(" % | CO2: ");
            Serial.print(co2ppm);
            Serial.print(" ppm | Battery: ");
            Serial.print(vbat_mv);
            Serial.print(" mV ");
            Serial.print(vbat_per);
            Serial.println(" %");
#endif
            if ( Bluefruit.connected() ) {
                if (start_notify[TEMPIDX]) {
                    temperatureChr.setData(temperature);
                    int16_t tempval = temperatureChr.getDataGain();
                    updateCharacteristic(&temperatureChr.chr, NOTIFY_OP, (uint8_t *) &tempval, 2);
                    // updateCharacteristic(TEMPIDX, NOTIFY_OP, (int32_t) temperature, sizeof(temperature));
                }
                /*
                if (start_notify[HUMDIDX]) {
                    updateCharacteristic(HUMDIDX, NOTIFY_OP, (int32_t) humidity, sizeof(humidity));
                }
                if (start_notify[CO2CIDX]) {
                    updateCharacteristic(CO2CIDX, NOTIFY_OP, (int32_t) co2ppm, sizeof(co2ppm)-1);
                }
                if (start_notify[VBATIDX]) {
                    vbatlv++;
                    updateCharacteristic(VBATIDX, NOTIFY_OP, (int32_t) vbat_per, sizeof(vbat_per));
                }
                */
            }
#ifdef SCD30
        } else {
            Serial.println("Read error!");
        }
    }
#endif
    delay(2000);
}

