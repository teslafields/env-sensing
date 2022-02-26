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
// Environmental Sensing Service is 0x181A
// BLEService ess = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
EnvSensingSvc ess;

int16_t temperature = 0;
uint16_t humidity = 0;
uint32_t co2ppm = 0;
uint8_t vbatlv = 0;

void startAdv(void);
void adv_stop_callback(void);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void connect_callback(uint16_t conn_handle);


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

    /*
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
    */
    Bluefruit.begin();

    // Set the connect/disconnect callback handlers
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // BLEService and BLECharacteristic classes
    Serial.println("Configuring the Environmental Sensing Service");
    ess.setup();

    // Setup the advertising packet(s)
    Serial.println("Setting up the advertising payload(s)");
    startAdv();

    Serial.println("\nAdvertising");
}

void startAdv(void)
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);


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

void loop()
{
    ess.service();
    delay(2000);
}

