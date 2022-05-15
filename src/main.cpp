#include <Arduino.h>
#include "credentials.h"
#include "def.h"

const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASSWD;

ADC_MODE(ADC_VCC);

void setup()
{
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
}

void loop()
{
    Serial.println(ESP.getVcc());

    digitalWrite(STATUS_LED, LOW);
    delay(200);
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
}