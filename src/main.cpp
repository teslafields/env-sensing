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

// GATT Characteristic 0x2A6E Temperature
// GATT Characteristic 0x2A6F Humidity
// GATT Characteristic 0x2BD0 CO concentration
// But SIGs 16-uuid oficial page does not have CO2, so we use POLLEN
// which is an equivalent - 0x2A75.
// Environmental Sensing Service is 0x181A
EnvSensingSvc ess;

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_BUTTON1, INPUT);
    // while ( !Serial ) delay(10);

    ess.setup();

    //Serial.println("Setting up the advertising payload(s)");
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

