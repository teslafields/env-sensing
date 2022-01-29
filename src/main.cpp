/*********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
 *********************************************************************/
#include <bluefruit.h>
#ifdef SCD30
#include <Adafruit_SCD30.h>
// Sensor SCD30
Adafruit_SCD30  scd30;
#endif

// Beacon uses the Manufacturer Specific Data field in the advertising
// packet, which means you must provide a valid Manufacturer ID. Update
// the field below to an appropriate value. For a list of valid IDs see:
// https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers
// 0x004C is Apple
// 0x0822 is Adafruit
// 0x0059 is Nordic
#define MANUFACTURER_ID              0x0059

// UUID Characteristc Descriptor for Environment Sensing Measurement
#define UUID_CHR_DESCRIPTOR_ES_MEAS  0x290C
#define UUID16_CHR_CO_PPM            0x2BD0

#define TEMPERATURE_IDX 0
#define HUMIDITY_IDX    1
#define CO2_IDX         2

#define WRITE_OP  1
#define NOTIFY_OP 2


void startAdv(void);
void setupESS(void);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void connect_callback(uint16_t conn_handle);
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);

// Environmental Sensing Service is 0x181A
BLEService        ess = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
// GATT Characteristic and Object Type 0x2A6E Temperature
BLECharacteristic tmpc = BLECharacteristic(UUID16_CHR_TEMPERATURE);
// GATT Characteristic and Object Type 0x2A6F Humidity
BLECharacteristic humc = BLECharacteristic(UUID16_CHR_HUMIDITY);
// GATT Characteristic and Object Type 0x2BD0 CO concentration
BLECharacteristic co2c = BLECharacteristic(UUID16_CHR_POLLEN_CONCENTRATION);

bool start_notify[3] = {false, false, false};
int16_t temperature = 0;
uint16_t humidity = 0;
uint32_t co2ppm = 0;

void updateCharacteristicMeasure(BLECharacteristic* chr, uint8_t op, uint8_t *data, uint8_t n) {
    if (op == WRITE_OP) {
        chr->write(data, n);
    }
    else if (op == NOTIFY_OP) {
        if (!chr->notify(data, n)) {
            Serial.print("ERR: Notify not set in CCCD or not connected! ");
            uint16_t uuid;
            chr->uuid.get(&uuid);
            Serial.println(uuid, HEX);
        }
    }
}

void updateChTemperature(uint8_t op) {
    // Temperature and Humidity measures have factor multiplied by 0.01
    // by the characteristic, and so we multiply it by 100
    int16_t temp = temperature * 100;
    updateCharacteristicMeasure(&tmpc, op, (uint8_t *) &temp, sizeof(temp));
}

void updateChHumidity(uint8_t op) {
    // Temperature and Humidity measures have factor multiplied by 0.01
    // by the characteristic, and so we multiply it by 100
    uint16_t hum = humidity * 100;
    updateCharacteristicMeasure(&humc, op, (uint8_t *) &hum, sizeof(hum));
}

void updateChCO2(uint8_t op) {
    updateCharacteristicMeasure(&co2c, op, (uint8_t *) &co2ppm, sizeof(co2ppm)-1);
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
    //Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(ess);

    // Add Advertising Service Data for Temperature
    uint8_t uuidlow = (uint8_t) UUID16_CHR_TEMPERATURE;
    uint8_t uuidhigh = (uint8_t) (UUID16_CHR_TEMPERATURE >> 8);
    uint8_t sdatat[4] = {uuidlow, uuidhigh, 0x44, 0xfd};
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA, &sdatat, 4);

    // Add Advertising Service Data for Humidity
    uuidlow = (uint8_t) UUID16_CHR_HUMIDITY;
    uuidhigh = (uint8_t) (UUID16_CHR_HUMIDITY >> 8);
    uint8_t sdatah[4] = {uuidlow, uuidhigh, 0xc4, 0x22};
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA, &sdatah, 4);

    uuidlow = (uint8_t) UUID16_CHR_POLLEN_CONCENTRATION;
    uuidhigh = (uint8_t) (UUID16_CHR_POLLEN_CONCENTRATION >> 8);
    uint8_t sdatac[5] = {uuidlow, uuidhigh, 0x90, 0x55, 0x01};
    Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA, &sdatac, sizeof(sdatac));

    Bluefruit.Advertising.addName();
    /* Start Advertising
       - Enable auto advertising if disconnected
       - Timeout for fast mode is 30 seconds
       - Start(timeout) with timeout = 0 will advertise forever (until connected)

       Apple Beacon specs
       - Type: Non connectable, undirected
       - Fixed interval: 100 ms -> fast = slow = 100 ms
       */
    // Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    // Bluefruit.Advertising.setInterval(160, 160);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void setupESS(void)
{
    ess.begin();

    tmpc.setProperties(CHR_PROPS_NOTIFY);
    tmpc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    tmpc.setCccdWriteCallback(cccd_callback);
    // tmpc.setFixedLen(2);
    tmpc.begin();
    uint8_t esm_desc[11] = { 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 }; 
    if (tmpc.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc))) {
        Serial.println("Error addDescriptor call");
    }

    humc.setProperties(CHR_PROPS_NOTIFY);
    humc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    humc.setCccdWriteCallback(cccd_callback);
    humc.begin();
    if (humc.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc))) {
        Serial.println("Error addDescriptor call");
    }

    co2c.setProperties(CHR_PROPS_NOTIFY);
    co2c.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    co2c.setCccdWriteCallback(cccd_callback);
    co2c.begin();
    if (co2c.addDescriptor(UUID_CHR_DESCRIPTOR_ES_MEAS, &esm_desc, sizeof(esm_desc))) {
        Serial.println("Error addDescriptor call");
    }
    
    updateChTemperature(WRITE_OP);
    updateChHumidity(WRITE_OP);
    updateChCO2(WRITE_OP);
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
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    Serial.print(cccd_value);
    Serial.println("");
    uint8_t idx = TEMPERATURE_IDX;

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == tmpc.uuid) {
        Serial.print("Temperature Characteristic ");
    }
    else if (chr->uuid == humc.uuid) {
        idx = HUMIDITY_IDX;
        Serial.print("Humidity Characteristic ");
    }
    else if (chr->uuid == co2c.uuid) {
        idx = CO2_IDX;
        Serial.print("Humidity Characteristic ");
    }
    if (chr->notifyEnabled(conn_hdl)) {
        // Start incrementing the measurement
        start_notify[idx] = true;
        Serial.println("'Notify' enabled");
    } else {
        // Stop incrementing the measurement
        start_notify[idx] = false;
        Serial.println("'Notify' disabled");
    }
}

void loop()
{
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
            Serial.println(" ppm");
            Serial.println("");
#endif
            if ( Bluefruit.connected() ) {
                if (start_notify[TEMPERATURE_IDX]) {
                    updateChTemperature(NOTIFY_OP);
                    // temperature++;
                }
                if (start_notify[HUMIDITY_IDX]) {
                    updateChHumidity(NOTIFY_OP);
                    // humidity++;
                }
                if (start_notify[CO2_IDX]) {
                    updateChCO2(NOTIFY_OP);
                }
            }
#ifdef SCD30
        } else {
            Serial.println("Read error!");
        }
    }
#endif
    delay(2000);
}

