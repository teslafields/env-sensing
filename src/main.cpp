#include "battery.h"
#include "envsensing.h"

#define DELAY_TIME     500 // ms
#define BTN_HYSTERESIS 10000/DELAY_TIME // 10s / DELAY_TIME

// https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers
// 0x0059 is Nordic
#define MANUFACTURER_ID    0x0059

EnvSensingSvc ess;
uint16_t ess_period_count = 0;

uint8_t btn_val = 0;
bool btn_pressed_event = false;
uint16_t btn_pressed_count = 0;
uint16_t btn_unpressed_count = 0;


void setup()
{
    Serial.begin(115200);

    pinMode(PIN_BUTTON1, INPUT);
#ifdef WAIT_SERIAL
    while ( !Serial ) delay(10);
#endif

    ess.setup();
}

void loop()
{
    if (ess_period_count >= 4) {
        ess.service();
        ess_period_count = 0;
    } else {
        ess_period_count++;
    }
    btn_val = digitalRead(PIN_BUTTON1);
    if (btn_val == LOW && !btn_pressed_event) {
        Serial.println(" Button pressed!");
        btn_pressed_event = true;
        btn_pressed_count = 0;
    } else {
        if (btn_pressed_event) {
            if (btn_val == HIGH) {
                btn_pressed_event = false;
            } else if (btn_pressed_count >= BTN_HYSTERESIS && !btn_unpressed_count) {
                blink_led(LED_RED);
                if (ess.recalibrateSensor()) {
                    blink_led(LED_RED);
                }
                btn_pressed_event = false;
                btn_unpressed_count = BTN_HYSTERESIS;
            } else {
                btn_pressed_count++;
            }
        } else if (btn_unpressed_count > 0) {
            btn_unpressed_count--;
        }
    }
    delay(DELAY_TIME);
}

