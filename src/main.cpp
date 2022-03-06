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


void setup()
{
    Serial.begin(115200);

    pinMode(PIN_BUTTON1, INPUT);
    // while ( !Serial ) delay(10);

    Serial.println("\nBluefruit52 GATT ESS");
    Serial.println("--------------------------");

    Bluefruit.begin();

    // Set the connect/disconnect callback handlers
    Bluefruit.Periph.setConnectCallback(&EnvSensingSvc::connectCallback);
    Bluefruit.Periph.setDisconnectCallback(&EnvSensingSvc::disconnectCallback);

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
    Bluefruit.Advertising.addService(ess.getBLEService());
    Bluefruit.Advertising.addName();
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

uint16_t ess_trigger_count = 0;
bool entered_section = false;
void loop()
{
    if (ess_trigger_count >= 4) {
        ess.service();
        ess_trigger_count = 0;
    } else {
        ess_trigger_count++;
    }
    if (digitalRead(PIN_BUTTON1) == LOW && !entered_section) {
        Serial.println("Button pressed!");
        entered_section = true;
    } else if (digitalRead(PIN_BUTTON1) == HIGH && entered_section) {
        entered_section = false;
    }
    delay(500);
}

