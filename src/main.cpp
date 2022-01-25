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

void updateESMeasure(uint8_t op);
void startAdv(void);
void setupES(void);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void connect_callback(uint16_t conn_handle);
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);

// Environmental Sensing Service is 0x181A
BLEService        ess = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
// GATT Characteristic and Object Type 0x2A6E Temperature
BLECharacteristic tmpc = BLECharacteristic(UUID16_CHR_TEMPERATURE);
// GATT Characteristic and Object Type 0x2A6F Humidity
BLECharacteristic humc = BLECharacteristic(UUID16_CHR_HUMIDITY);

int16_t temperature = -10;
uint16_t humidity = 52;

void updateESMeasure(uint8_t op) {
    // Temperature and Humidity measures have factor multiplied by 0.01
    // by the characteristic, and so we multiply it by 100
    int16_t temp = temperature * 100;
    uint16_t hum = humidity * 100;
    uint8_t *tp = (uint8_t *) &temp;
    uint8_t *hp = (uint8_t *) &hum;
    if (op == 1) {
        tmpc.write(tp, sizeof(temp));
        humc.write(hp, sizeof(hum));
    }
    else if (op == 2) {
        if (tmpc.notify(tp, sizeof(temp))) {
            Serial.print("Temperature Measurement updated to: ");
            Serial.println(temp);
        }
        else {
            Serial.println("ERROR: Notify not set in CCCD or not connected!");
        }
        if (humc.notify(hp, sizeof(hum))) {
            Serial.print("Humidity Measurement updated to: ");
            Serial.println(hum);
        }
        else {
            Serial.println("ERROR: Notify not set in CCCD or not connected!");
        }
    }
}

void setup()
{
    Serial.begin(115200);

    while ( !Serial ) delay(10);

    Serial.println("\nBluefruit52 GATT ESS Example");
    Serial.println("--------------------------");

    Bluefruit.begin();

    // Set the connect/disconnect callback handlers
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // BLEService and BLECharacteristic classes
    Serial.println("Configuring the Environmental Sensing Service");
    setupES();

    // Setup the advertising packet(s)
    Serial.println("Setting up the advertising payload(s)");
    startAdv();

    Serial.println("\nAdvertising");
}

void startAdv(void)
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    Bluefruit.Advertising.addService(ess);

    // Include Name
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

void setupES(void)
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
    updateESMeasure(1);
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
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == tmpc.uuid) {
        Serial.print("Temperature Characteristic ");
    }
    else if (chr->uuid == humc.uuid) {
        Serial.print("Humidity Characteristic ");
    }
    if (chr->notifyEnabled(conn_hdl)) {
        Serial.println("'Notify' enabled");
    } else {
        Serial.println("'Notify' disabled");
    }
}

void loop()
{
    if ( Bluefruit.connected() ) {
        temperature++;
        humidity++;
        updateESMeasure(2);
    }
    delay(1000);
}

